import itertools
import os
import glob
from collections import Counter

import numpy as np
import pandas as pd
from scipy.optimize import linprog, lsq_linear, minimize

from xml_config_builder import build_xml, save_xml, RoutingValidationError, UGVUAVConfig
from config_params import _load_json_db
from config_params import UAV_DB_PATH, UGV_DB_PATH, GRID_MAPPING_UGV, TAU_MIN, TAU_MAX, D_SAFE_CABLE

EPS_RANK = 1e-6
OUT_PATH = None

# Fallback single pose, used only if no poses CSV is found
PX, PY, PZ = 0.5, -0.5, 2.0
QUAT_WXYZ = (1.0, 0.0, 0.0, 0.0)

POSES_CSV = None                   # override path; None -> default below
DEFAULT_POSES_CSV = "create_xml/poses_to_analyze.csv"

MAX_ENUMERATE = 20000              # per-architecture routing cap before capping/sampling

# --- Wrench Feasible Condition (WFC) ---
ENABLE_WFC = True
GROUND_TAU_MIN = TAU_MIN
GROUND_TAU_MAX = TAU_MAX
DRONE_THRUST_MIN = 1.0
DRONE_THRUST_MAX = 44.0            # ~= 4 * kt * MAX_ROTOR_VELOCITY**2

PAYLOAD_MASS = 1.0                 # [kg] -> default gravity-compensation target wrench
G_ACCEL = 9.81
WFC_WRENCH = None                  # None -> [0, 0, payload_mass * g_accel, 0, 0, 0]

UAV_LAYOUT = "triangle"
UAV_CABLE_LENGTH = 1.5             # [m] nominal drone cable length

WFC_RESIDUAL_TOL = 1.0             # combined force+moment tolerance (coarse, backward-compatible)
WFC_FORCE_TOL = None               # separate force-only tolerance [N]
WFC_MOMENT_TOL = None              # separate moment-only tolerance [N*m]

WFC_VERIFY_TOP_K = 10              # 0 = strict/cheap-only mode 
WFC_VERIFY_MAX_ITER = 100

# --- Interference-Free Condition (IFC) ---
ENABLE_IFC = True
D_SAFE = D_SAFE_CABLE


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def quat_to_R(w, x, y, z):
    """Standard (w,x,y,z) unit-quaternion to rotation matrix. Normalizes defensively."""
    n = np.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        raise ValueError("degenerate quaternion (all-zero)")
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z),     2 * (x * y - z * w),     2 * (x * z + y * w)],
        [    2 * (x * y + z * w), 1 - 2 * (x * x + z * z),     2 * (y * z - x * w)],
        [    2 * (x * z - y * w),     2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def node_letters(db, layout):
    return [k for k in db[layout].keys() if k != "symmetry_metadata"]


def max_cables(db, layout, letter):
    return db[layout][letter].get("max_cables", 999)


def enumerate_routings(db, pay_layout, gnd_layout, n_cables=6, max_results=None):
    """
    Exhaustively yields every valid combination of n_cables distinct
    (payload_node, ground_node) edges respecting each node's max_cables.
    """
    pay_letters = node_letters(db, pay_layout)
    gnd_letters = node_letters(db, gnd_layout)
    all_pairs = [(p, g) for p in pay_letters for g in gnd_letters]

    results = []

    def backtrack(chosen, pay_counts, gnd_counts, start_idx):
        if max_results is not None and len(results) >= max_results:
            return
        if len(chosen) == n_cables:
            results.append(tuple(chosen))
            return
        for idx in range(start_idx, len(all_pairs)):
            if max_results is not None and len(results) >= max_results:
                return
            p, g = all_pairs[idx]
            if pay_counts.get(p, 0) >= max_cables(db, pay_layout, p):
                continue
            if gnd_counts.get(g, 0) >= max_cables(db, gnd_layout, g):
                continue
            chosen.append((p, g))
            pay_counts[p] = pay_counts.get(p, 0) + 1
            gnd_counts[g] = gnd_counts.get(g, 0) + 1
            backtrack(chosen, pay_counts, gnd_counts, idx + 1)
            chosen.pop()
            pay_counts[p] -= 1
            gnd_counts[g] -= 1

    backtrack([], {}, {}, 0)
    return results


def count_routings_upper_bound(db, pay_layout, gnd_layout, n_cables=6):
    """Cheap sanity check before a full enumeration: total candidate pairs choose n_cables (loose upper bound)."""
    n_pairs = len(node_letters(db, pay_layout)) * len(node_letters(db, gnd_layout))
    if n_pairs < n_cables:
        return 0
    from math import comb
    return comb(n_pairs, n_cables)

def build_Jp_ground(routing, ugv_db, pay_layout, gnd_layout, payload_pos, payload_R=None):
    payload_R = np.eye(3) if payload_R is None else payload_R
    rows = []
    for p_node, g_node in routing:
        bi = np.array(ugv_db[pay_layout][p_node]["coords"], dtype=float)
        ai = np.array(ugv_db[gnd_layout][g_node]["coords"], dtype=float)
        ai = ai.copy()
        ai[2] = 0.0  # ground anchors sit at z=0

        ri = payload_pos + payload_R @ bi
        li = ai - ri
        norm = np.linalg.norm(li)
        if norm < 1e-9:
            return None  # degenerate: anchor coincides with payload node

        ui = li / norm
        Jp_row = np.concatenate([ui, np.cross(payload_R @ bi, ui)])
        rows.append(Jp_row)

    return np.array(rows)  # (6, 6)


def score_Jp(Jp):
    if Jp is None:
        return None
    svals = np.linalg.svd(Jp, compute_uv=False)
    sigma_min, sigma_max = svals[-1], svals[0]
    if sigma_max < 1e-9:
        return None
    return {
        "conditioning_index": sigma_min / sigma_max,
        "manipulability": float(np.prod(svals)),  # == sqrt(det(Jp Jp^T))
        "sigma_min": sigma_min,
        "sigma_max": sigma_max,
        "rank_ok": bool(sigma_min > EPS_RANK),
    }


def check_wfc(Jp, tau_min, tau_max, W_target):
    """Ground-only, exact-equality WFC check (the literal Eq. 1.9/2.35 formulation)."""
    m = Jp.shape[0]
    tau_min_vec = np.full(m, tau_min) if np.isscalar(tau_min) else np.asarray(tau_min, dtype=float)
    tau_max_vec = np.full(m, tau_max) if np.isscalar(tau_max) else np.asarray(tau_max, dtype=float)

    A_eq = Jp.T
    b_eq = np.asarray(W_target, dtype=float)
    bounds = list(zip(tau_min_vec, tau_max_vec))
    c = np.zeros(m)  # feasibility only

    res = linprog(c, A_eq=A_eq, b_eq=b_eq, bounds=bounds, method="highs")
    if res.success:
        return {"wfc_ok": True, "tau": res.x}
    return {"wfc_ok": False, "tau": None}


def get_uav_hook_offsets(uav_db, uav_layout):
    """Payload-side attachment points for the 3 drone cables."""
    letters = [k for k in uav_db[uav_layout].keys() if k != "symmetry_metadata"]
    return [np.array(uav_db[uav_layout][l]["coords"], dtype=float) for l in letters]


def nominal_drone_positions(uav_hook_offsets, payload_pos, payload_R, uav_cable_length):
    """Places each drone directly above its payload hook - cheap approximation."""
    positions = []
    for bi in uav_hook_offsets:
        ri = payload_pos + payload_R @ bi
        positions.append(ri + np.array([0.0, 0.0, uav_cable_length]))
    return positions


def build_Jp_full(ground_routing, ugv_db, pay_layout, gnd_layout,
                   uav_hook_offsets, drone_positions, payload_pos, payload_R):
    """Full (n_ground + n_drone, 6) Jacobian - ground rows first, then drone rows."""
    rows = []
    for p_node, g_node in ground_routing:
        bi = np.array(ugv_db[pay_layout][p_node]["coords"], dtype=float)
        ai = np.array(ugv_db[gnd_layout][g_node]["coords"], dtype=float).copy()
        ai[2] = 0.0
        ri = payload_pos + payload_R @ bi
        li = ai - ri
        ui = li / np.linalg.norm(li)
        rows.append(np.concatenate([ui, np.cross(payload_R @ bi, ui)]))

    for bi, ai in zip(uav_hook_offsets, drone_positions):
        ri = payload_pos + payload_R @ bi
        li = ai - ri
        ui = li / np.linalg.norm(li)
        rows.append(np.concatenate([ui, np.cross(payload_R @ bi, ui)]))

    return np.array(rows)


def check_joint_wfc(Jp_full, n_ground, n_drone,
                     ground_tau_min, ground_tau_max,
                     drone_thrust_min, drone_thrust_max,
                     W_target, residual_tol=1.0,
                     residual_force_tol=None, residual_moment_tol=None):
    """
    Joint (9-cable) WFC check: least-squares tension solve (matches the live
    controller's actual formulation) accepted within a residual tolerance,
    rather than exact equality. Splits the residual into force/moment and
    lateral/vertical-force components so failures can be diagnosed.
    """
    m = Jp_full.shape[0]
    assert m == n_ground + n_drone, "Jp_full row count must match n_ground + n_drone"

    A = Jp_full.T
    b = np.asarray(W_target, dtype=float)
    lb = [ground_tau_min] * n_ground + [drone_thrust_min] * n_drone
    ub = [ground_tau_max] * n_ground + [drone_thrust_max] * n_drone

    res = lsq_linear(A, b, bounds=(lb, ub), method="bvls")
    mismatch = A @ res.x - b
    residual = float(np.linalg.norm(mismatch))
    residual_force = float(np.linalg.norm(mismatch[:3]))
    residual_moment = float(np.linalg.norm(mismatch[3:]))
    residual_force_lateral = float(np.linalg.norm(mismatch[:2]))
    residual_force_vertical = float(abs(mismatch[2]))

    ok = residual <= residual_tol
    if residual_force_tol is not None:
        ok = ok and (residual_force <= residual_force_tol)
    if residual_moment_tol is not None:
        ok = ok and (residual_moment <= residual_moment_tol)

    return {
        "joint_wfc_ok": ok,
        "residual": residual,
        "residual_force": residual_force,
        "residual_moment": residual_moment,
        "residual_force_lateral": residual_force_lateral,
        "residual_force_vertical": residual_force_vertical,
        "tau": res.x,
    }


def optimize_drone_positions_for_wfc(uav_hook_offsets, payload_pos, payload_R, uav_cable_length,
                                      ground_routing, ugv_db, pay_layout, gnd_layout,
                                      ground_tau_min, ground_tau_max,
                                      drone_thrust_min, drone_thrust_max, W_target,
                                      max_iter=200):
    """
    Searches each drone's position on the sphere of radius uav_cable_length 
    around its hook to minimize the joint WFC residual.
    """
    n_drone = len(uav_hook_offsets)
    n_ground = len(ground_routing)
    hooks_world = [payload_pos + payload_R @ bi for bi in uav_hook_offsets]

    def angles_to_positions(angles_flat):
        positions = []
        for i in range(n_drone):
            theta, phi = angles_flat[2 * i], angles_flat[2 * i + 1]
            direction = np.array([
                np.sin(theta) * np.cos(phi),
                np.sin(theta) * np.sin(phi),
                np.cos(theta),
            ])
            positions.append(hooks_world[i] + direction * uav_cable_length)
        return positions

    def objective(angles_flat):
        drone_positions = angles_to_positions(angles_flat)
        Jp_full = build_Jp_full(ground_routing, ugv_db, pay_layout, gnd_layout,
                                 uav_hook_offsets, drone_positions, payload_pos, payload_R)
        wfc = check_joint_wfc(Jp_full, n_ground=n_ground, n_drone=n_drone,
                               ground_tau_min=ground_tau_min, ground_tau_max=ground_tau_max,
                               drone_thrust_min=drone_thrust_min, drone_thrust_max=drone_thrust_max,
                               W_target=W_target, residual_tol=0.0)
        return wfc["residual"]

    x0 = np.zeros(2 * n_drone)  # theta=0 for all -> starts exactly at nominal_drone_positions
    res = minimize(objective, x0, method="Nelder-Mead",
                    options={"xatol": 1e-3, "fatol": 1e-3, "maxiter": max_iter})

    best_positions = angles_to_positions(res.x)
    Jp_full_best = build_Jp_full(ground_routing, ugv_db, pay_layout, gnd_layout,
                                  uav_hook_offsets, best_positions, payload_pos, payload_R)
    best_wfc = check_joint_wfc(Jp_full_best, n_ground=n_ground, n_drone=n_drone,
                                ground_tau_min=ground_tau_min, ground_tau_max=ground_tau_max,
                                drone_thrust_min=drone_thrust_min, drone_thrust_max=drone_thrust_max,
                                W_target=W_target, residual_tol=0.0)
    return best_positions, best_wfc["residual"], best_wfc


# ---------------------------------------------------------------------------
# INTERFERENCE-FREE CONDITION (IFC) - Eq. 2.20
# ---------------------------------------------------------------------------

def segment_segment_distance(p1, p2, q1, q2):
    """Exact minimum distance between two 3D line segments [p1,p2] and [q1,q2]."""
    d1 = p2 - p1
    d2 = q2 - q1
    r = p1 - q1
    a = np.dot(d1, d1)
    e = np.dot(d2, d2)
    f = np.dot(d2, r)
    EPS = 1e-12

    if a <= EPS and e <= EPS:
        return np.linalg.norm(p1 - q1)

    if a <= EPS:
        s = 0.0
        t = np.clip(f / e, 0.0, 1.0)
    else:
        c = np.dot(d1, r)
        if e <= EPS:
            t = 0.0
            s = np.clip(-c / a, 0.0, 1.0)
        else:
            b = np.dot(d1, d2)
            denom = a * e - b * b
            s = np.clip((b * f - c * e) / denom, 0.0, 1.0) if denom > EPS else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t = 0.0
                s = np.clip(-c / a, 0.0, 1.0)
            elif t > 1.0:
                t = 1.0
                s = np.clip((b - c) / a, 0.0, 1.0)

    c1 = p1 + s * d1
    c2 = q1 + t * d2
    return float(np.linalg.norm(c1 - c2))


def check_ifc(routing, ugv_db, pay_layout, gnd_layout, payload_pos, payload_R, d_safe):

    segments = []
    for p_node, g_node in routing:
        bi = np.array(ugv_db[pay_layout][p_node]["coords"], dtype=float)
        ai = np.array(ugv_db[gnd_layout][g_node]["coords"], dtype=float)
        ai = ai.copy()
        ai[2] = 0.0
        ri = payload_pos + payload_R @ bi
        segments.append((ri, ai, p_node, g_node))

    # cable-cable clearance - skip pairs sharing a payload or ground node
    min_cc = None
    n_skipped = 0
    for (r1, a1, p1n, g1n), (r2, a2, p2n, g2n) in itertools.combinations(segments, 2):
        if p1n == p2n or g1n == g2n:
            n_skipped += 1
            continue
        d = segment_segment_distance(r1, a1, r2, a2)
        min_cc = d if min_cc is None else min(min_cc, d)

    ok = (min_cc is None) or (min_cc >= d_safe)

    return {
        "ifc_ok": ok,
        "min_cable_cable_dist": min_cc,
        "n_pairs_skipped_shared_anchor": n_skipped,
    }


# ---------------------------------------------------------------------------
# Per-routing, per-pose scoring
# ---------------------------------------------------------------------------

def score_routing_across_poses(routing, ugv_db, pay_layout, gnd_layout, poses,
                                enable_wfc=False, tau_min=5.0, tau_max=100.0,
                                wfc_wrench=None, uav_hook_offsets=None,
                                uav_cable_length=1.5, drone_thrust_min=1.0,
                                drone_thrust_max=44.0, wfc_residual_tol=1.0,
                                wfc_force_tol=None, wfc_moment_tol=None,
                                wfc_gate=True,
                                enable_ifc=True, d_safe=D_SAFE_CABLE,
                                stats=None):
    """
    Scores one routing at every (position, R) in `poses` and returns the
    WORST-CASE result across them. Returns None if the routing fails WCC
    or IFC at any pose; a WFC failure's effect depends on wfc_gate.
    """
    if wfc_wrench is None:
        wfc_wrench = np.zeros(6)
    if enable_wfc and uav_hook_offsets is None:
        raise ValueError("enable_wfc=True requires uav_hook_offsets (see get_uav_hook_offsets)")

    per_pose_scores = []
    per_pose_wfc = []
    per_pose_ifc = []
    any_wfc_soft_fail = False

    for payload_pos, payload_R in poses:
        Jp = build_Jp_ground(routing, ugv_db, pay_layout, gnd_layout, payload_pos, payload_R)
        score = score_Jp(Jp)
        if score is None or not score["rank_ok"]:
            if stats is not None:
                stats["wcc_fail"] = stats.get("wcc_fail", 0) + 1
            return None  # fails WCC

        if enable_ifc:
            ifc = check_ifc(routing, ugv_db, pay_layout, gnd_layout, payload_pos, payload_R, d_safe)
            if not ifc["ifc_ok"]:
                if stats is not None:
                    stats["ifc_fail"] = stats.get("ifc_fail", 0) + 1
                return None  # fails IFC
            per_pose_ifc.append(ifc)

        if enable_wfc:
            drone_positions = nominal_drone_positions(uav_hook_offsets, payload_pos, payload_R,
                                                        uav_cable_length)
            Jp_full = build_Jp_full(routing, ugv_db, pay_layout, gnd_layout,
                                     uav_hook_offsets, drone_positions, payload_pos, payload_R)
            wfc = check_joint_wfc(Jp_full, n_ground=len(routing), n_drone=len(uav_hook_offsets),
                                   ground_tau_min=tau_min, ground_tau_max=tau_max,
                                   drone_thrust_min=drone_thrust_min, drone_thrust_max=drone_thrust_max,
                                   W_target=wfc_wrench, residual_tol=wfc_residual_tol,
                                   residual_force_tol=wfc_force_tol, residual_moment_tol=wfc_moment_tol)
            if not wfc["joint_wfc_ok"]:
                if stats is not None:
                    stats["wfc_fail"] = stats.get("wfc_fail", 0) + 1
                    stats.setdefault("wfc_fail_residuals", []).append(wfc["residual"])
                    stats.setdefault("wfc_fail_residual_force", []).append(wfc["residual_force"])
                    stats.setdefault("wfc_fail_residual_moment", []).append(wfc["residual_moment"])
                    stats.setdefault("wfc_fail_residual_force_lateral", []).append(wfc["residual_force_lateral"])
                    stats.setdefault("wfc_fail_residual_force_vertical", []).append(wfc["residual_force_vertical"])
                if wfc_gate:
                    return None  # fails WFC (strict mode)
                any_wfc_soft_fail = True
            per_pose_wfc.append(wfc)

        per_pose_scores.append(score)

    if stats is not None:
        stats["passed"] = stats.get("passed", 0) + 1

    result = {
        "conditioning_index": min(s["conditioning_index"] for s in per_pose_scores),
        "manipulability": min(s["manipulability"] for s in per_pose_scores),
        "sigma_min": min(s["sigma_min"] for s in per_pose_scores),
        "sigma_max": max(s["sigma_max"] for s in per_pose_scores),
        "rank_ok": True,
        "n_poses_checked": len(per_pose_scores),
    }

    if enable_wfc:
        result["wfc_ok"] = not any_wfc_soft_fail
        result["max_wfc_residual"] = max(w["residual"] for w in per_pose_wfc)
        result["max_wfc_residual_force"] = max(w["residual_force"] for w in per_pose_wfc)
        result["max_wfc_residual_moment"] = max(w["residual_moment"] for w in per_pose_wfc)
    if enable_ifc:
        cc_vals = [d["min_cable_cable_dist"] for d in per_pose_ifc if d["min_cable_cable_dist"] is not None]
        result["ifc_ok"] = True
        result["min_cable_cable_dist"] = min(cc_vals) if cc_vals else None

    return result


def screen_architecture(ugv_db, pay_layout, gnd_layout, poses, max_enumerate=20000,
                         enable_wfc=False, tau_min=5.0, tau_max=100.0, wfc_wrench=None,
                         uav_hook_offsets=None, uav_cable_length=1.5,
                         drone_thrust_min=1.0, drone_thrust_max=44.0, wfc_residual_tol=1.0,
                         wfc_force_tol=None, wfc_moment_tol=None,
                         wfc_verify_top_k=0, wfc_verify_max_iter=100,
                         enable_ifc=True, d_safe=D_SAFE_CABLE):
    """
    Picks the routing with the best worst-case manipulability across all
    poses, among routings the ones that pass

    wfc_verify_top_k=0: strict mode - a WFC failure discards the routing
    immediately (same behavior/cost as before this feature existed).

    wfc_verify_top_k>0: two-phase workflow - first sweep with wfc_gate=False
    to find every WCC+IFC survivor regardless of the cheap WFC outcome; if
    nothing fully passes, re-check the top-K of those by manipulability
    using the expensive optimize_drone_positions_for_wfc, promoting the
    first one that passes real tolerances. Rescues routings whose only
    problem was a lateral-force/moment mismatch a repositioned drone can fix.
    """
    upper_bound = count_routings_upper_bound(ugv_db, pay_layout, gnd_layout)
    if upper_bound == 0:
        return None

    if upper_bound <= max_enumerate:
        routings = enumerate_routings(ugv_db, pay_layout, gnd_layout)
    else:
        routings = enumerate_routings(ugv_db, pay_layout, gnd_layout, max_results=max_enumerate)

    use_two_phase = enable_wfc and wfc_verify_top_k > 0

    stats = {"wcc_fail": 0, "wfc_fail": 0, "ifc_fail": 0, "passed": 0}
    best = None
    pending_candidates = []  # (manipulability, routing, score) - only used if use_two_phase

    for routing in routings:
        gnd_counts = Counter(g for (_, g) in routing)
        if any(count >= 3 for count in gnd_counts.values()):
            continue  # no ground node may take 3+ cables

        score = score_routing_across_poses(
            routing, ugv_db, pay_layout, gnd_layout, poses,
            enable_wfc=enable_wfc, tau_min=tau_min, tau_max=tau_max, wfc_wrench=wfc_wrench,
            uav_hook_offsets=uav_hook_offsets, uav_cable_length=uav_cable_length,
            drone_thrust_min=drone_thrust_min, drone_thrust_max=drone_thrust_max,
            wfc_residual_tol=wfc_residual_tol, wfc_force_tol=wfc_force_tol, wfc_moment_tol=wfc_moment_tol,
            wfc_gate=not use_two_phase,
            enable_ifc=enable_ifc, d_safe=d_safe,
            stats=stats,
        )
        if score is None:
            continue
        if use_two_phase and not score.get("wfc_ok", True):
            pending_candidates.append((score["manipulability"], routing, score))
            continue
        if best is None or score["manipulability"] > best["manipulability"]:
            best = {**score, "routing": routing}

    n_wfc_verify_attempted = 0
    n_wfc_verify_rescued = 0
    if best is None and use_two_phase and pending_candidates:
        pending_candidates.sort(key=lambda t: t[0], reverse=True)
        for manipulability, routing, score in pending_candidates[:wfc_verify_top_k]:
            n_wfc_verify_attempted += 1
            all_poses_pass = True
            for payload_pos, payload_R in poses:
                _, residual, wfc_result = optimize_drone_positions_for_wfc(
                    uav_hook_offsets, payload_pos, payload_R, uav_cable_length,
                    routing, ugv_db, pay_layout, gnd_layout,
                    tau_min, tau_max, drone_thrust_min, drone_thrust_max, wfc_wrench,
                    max_iter=wfc_verify_max_iter,
                )
                # re-check against the REAL configured tolerances, not the
                # internal residual_tol=0.0 used to drive the search itself
                ok = residual <= wfc_residual_tol
                if wfc_force_tol is not None:
                    ok = ok and (wfc_result["residual_force"] <= wfc_force_tol)
                if wfc_moment_tol is not None:
                    ok = ok and (wfc_result["residual_moment"] <= wfc_moment_tol)
                if not ok:
                    all_poses_pass = False
                    break
            if all_poses_pass:
                n_wfc_verify_rescued += 1
                best = {**score, "routing": routing, "wfc_ok": True, "wfc_rescued_by_reposition": True}
                break  # sorted by manipulability descending - first success is best available

    wfc_residuals = stats.get("wfc_fail_residuals", [])
    wfc_force_residuals = stats.get("wfc_fail_residual_force", [])
    wfc_moment_residuals = stats.get("wfc_fail_residual_moment", [])
    wfc_lateral_residuals = stats.get("wfc_fail_residual_force_lateral", [])
    wfc_vertical_residuals = stats.get("wfc_fail_residual_force_vertical", [])
    wfc_residual_summary = {
        "wfc_fail_residual_min": float(np.min(wfc_residuals)) if wfc_residuals else None,
        "wfc_fail_residual_median": float(np.median(wfc_residuals)) if wfc_residuals else None,
        "wfc_fail_residual_max": float(np.max(wfc_residuals)) if wfc_residuals else None,
        "wfc_fail_residual_force_median": float(np.median(wfc_force_residuals)) if wfc_force_residuals else None,
        "wfc_fail_residual_moment_median": float(np.median(wfc_moment_residuals)) if wfc_moment_residuals else None,
        "wfc_fail_residual_force_lateral_median": float(np.median(wfc_lateral_residuals)) if wfc_lateral_residuals else None,
        "wfc_fail_residual_force_vertical_median": float(np.median(wfc_vertical_residuals)) if wfc_vertical_residuals else None,
        "n_wfc_verify_attempted": n_wfc_verify_attempted,
        "n_wfc_verify_rescued": n_wfc_verify_rescued,
    }

    if best is None:
        return {
            "pay_layout": pay_layout, "gnd_layout": gnd_layout,
            "n_routings_checked": len(routings), "n_poses_checked": len(poses), "feasible": False,
            "n_wcc_fail": stats["wcc_fail"], "n_wfc_fail": stats["wfc_fail"], "n_ifc_fail": stats["ifc_fail"],
            **wfc_residual_summary,
        }

    return {
        "pay_layout": pay_layout, "gnd_layout": gnd_layout,
        "n_routings_checked": len(routings), "feasible": True,
        "n_wcc_fail": stats["wcc_fail"], "n_wfc_fail": stats["wfc_fail"], "n_ifc_fail": stats["ifc_fail"],
        **wfc_residual_summary,
        **{k: v for k, v in best.items() if k != "routing"},
        "best_routing": "|".join(f"{p}-{g}" for p, g in best["routing"]),
    }


def screen_all(poses, max_enumerate=20000,
                enable_wfc=False, tau_min=5.0, tau_max=100.0, wfc_wrench=None,
                uav_layout="triangle", uav_cable_length=1.5,
                drone_thrust_min=1.0, drone_thrust_max=44.0, wfc_residual_tol=1.0,
                wfc_force_tol=None, wfc_moment_tol=None,
                wfc_verify_top_k=0, wfc_verify_max_iter=100,
                enable_ifc=True, d_safe=D_SAFE_CABLE):
    ugv_db = _load_json_db(UGV_DB_PATH)
    ugv_db.pop("rectangle-same", None)  # known unstable in simulation; excluded from re-screening

    layouts = [l for l in GRID_MAPPING_UGV.keys() if l != "rectangle-same"]

    uav_hook_offsets = None
    if enable_wfc:
        uav_db = _load_json_db(UAV_DB_PATH)
        uav_hook_offsets = get_uav_hook_offsets(uav_db, uav_layout)
        print(f"Joint WFC enabled: using UAV layout '{uav_layout}' "
              f"({len(uav_hook_offsets)} drone cables) alongside the ground routing.")
        if wfc_verify_top_k > 0:
            print(f"Two-phase WFC verification enabled: up to {wfc_verify_top_k} "
                  f"best-by-manipulability WCC+IFC survivors per architecture will get the "
                  f"expensive drone-repositioning check if nothing passes the cheap one.")

    rows = []
    total = len(layouts) * len(layouts)
    for i, (pay_layout, gnd_layout) in enumerate(itertools.product(layouts, layouts), 1):
        print(f"[{i}/{total}] {pay_layout} / {gnd_layout} ...", end=" ")
        result = screen_architecture(
            ugv_db, pay_layout, gnd_layout, poses, max_enumerate=max_enumerate,
            enable_wfc=enable_wfc, tau_min=tau_min, tau_max=tau_max, wfc_wrench=wfc_wrench,
            uav_hook_offsets=uav_hook_offsets, uav_cable_length=uav_cable_length,
            drone_thrust_min=drone_thrust_min, drone_thrust_max=drone_thrust_max,
            wfc_residual_tol=wfc_residual_tol, wfc_force_tol=wfc_force_tol, wfc_moment_tol=wfc_moment_tol,
            wfc_verify_top_k=wfc_verify_top_k, wfc_verify_max_iter=wfc_verify_max_iter,
            enable_ifc=enable_ifc, d_safe=d_safe,
        )
        if result is None:
            print("no valid pairs (fewer nodes than 6 cables need)")
            continue
        print(f"checked {result['n_routings_checked']} routings across {len(poses)} pose(s), feasible={result['feasible']}")
        rows.append(result)

    df = pd.DataFrame(rows)
    if not df.empty and "conditioning_index" in df.columns:
        df = df.sort_values(by=["sigma_min", "conditioning_index"], ascending=[False, False]).reset_index(drop=True)
    return df


def build_poses_from_csv(path):
    poses_df = pd.read_csv(path)
    poses = []
    for _, r in poses_df.iterrows():
        pos = np.array([r["px"], r["py"], r["pz"]], dtype=float)
        R = quat_to_R(r["quat_w"], r["quat_x"], r["quat_y"], r["quat_z"])
        poses.append((pos, R))
    return poses


def print_infeasibility_report(df):
    """Prints a diagnostic breakdown when every architecture came back infeasible."""
    n_wcc = int(df["n_wcc_fail"].sum()) if "n_wcc_fail" in df.columns else None
    n_wfc = int(df["n_wfc_fail"].sum()) if "n_wfc_fail" in df.columns else None
    n_ifc = int(df["n_ifc_fail"].sum()) if "n_ifc_fail" in df.columns else None

    print("\n" + "=" * 80)
    print("[NO FEASIBLE ARCHITECTURES] Every (pay_layout, gnd_layout) pair had")
    print("zero routings survive. Rejection counts across ALL architectures checked:")
    print(f"    failed WCC (rank-deficient / degenerate)                                   : {n_wcc}")
    print(f"    failed WFC (residual mismatch > tolerance, joint 9-cable solve)             : {n_wfc}")
    print(f"    failed IFC (cable-cable clearance < d_safe)                                : {n_ifc}")

    if n_wfc and "wfc_fail_residual_median" in df.columns:
        med_vals = df["wfc_fail_residual_median"].dropna()
        min_vals = df["wfc_fail_residual_min"].dropna()
        max_vals = df["wfc_fail_residual_max"].dropna()
        force_med = df.get("wfc_fail_residual_force_median", pd.Series(dtype=float)).dropna()
        moment_med = df.get("wfc_fail_residual_moment_median", pd.Series(dtype=float)).dropna()
        lateral_med = df.get("wfc_fail_residual_force_lateral_median", pd.Series(dtype=float)).dropna()
        vertical_med = df.get("wfc_fail_residual_force_vertical_median", pd.Series(dtype=float)).dropna()

        if len(med_vals) > 0:
            print(f"    WFC failure residuals actually seen (combined): "
                  f"smallest={min_vals.min():.3g}, "
                  f"typical(median of per-arch medians)={med_vals.median():.3g}, "
                  f"largest={max_vals.max():.3g}")
            if len(force_med) > 0 and len(moment_med) > 0:
                print(f"    Split by type - typical FORCE mismatch: {force_med.median():.3g} N, "
                      f"typical MOMENT mismatch: {moment_med.median():.3g} N*m")
            if len(lateral_med) > 0 and len(vertical_med) > 0:
                print(f"    Force further split - typical LATERAL (XY) mismatch: {lateral_med.median():.3g} N, "
                      f"typical VERTICAL (Z) mismatch: {vertical_med.median():.3g} N")
                print("    LATERAL mismatch is diagnostic: nominal_drone_positions locks every "
                      "drone's cable direction to (0,0,1), so drones structurally CANNOT "
                      "contribute lateral force. If lateral dominates, tightening a force "
                      "tolerance won't fix it - drones need to be REPOSITIONED (see "
                      "optimize_drone_positions_for_wfc).")


def generate_top_xml_configs(df, target_output_dir, chosen_uav_layout="triangle"):
    uav_geo_db = _load_json_db(UAV_DB_PATH)
    ugv_geo_db = _load_json_db(UGV_DB_PATH)

    print("\nGenerating MuJoCo XML configurations for the top architectures...")
    top_feasible_df = df[df["feasible"] == True].head(5)

    for idx, row in top_feasible_df.iterrows():
        routing_pairs = [pair.split("-") for pair in row["best_routing"].split("|")]
        routing_dict = {
            i + 4: {"payload_anchor": p, "ground_anchor": g}
            for i, (p, g) in enumerate(routing_pairs)
        }

        config = UGVUAVConfig(
            pay_layout=row["pay_layout"],
            gnd_layout=row["gnd_layout"],
            uav_layout=chosen_uav_layout,
            routing=routing_dict,
            mirror_gnd_x=False,
            mirror_gnd_y=False,
            scale_mode="Normal",
        )

        try:
            xml_content = build_xml(config, ugv_geo_db, uav_geo_db)
            file_path, msg = save_xml(config, xml_content, ugv_geo_db, out_dir=target_output_dir)
            if msg:
                print(f"  [SKIPPED Top {idx+1}] {msg}")
            else:
                print(f"  [SUCCESS Top {idx+1}] Saved config to {file_path}")
        except RoutingValidationError as e:
            print(f"  [VALIDATION ERROR Top {idx+1}] {e}")
        except Exception as e:
            print(f"  [ERROR Top {idx+1}] Failed generation: {e}")


def main():
    if POSES_CSV:
        poses = build_poses_from_csv(POSES_CSV)
        print(f"Evaluating across {len(poses)} poses from {POSES_CSV}")
    elif os.path.exists(DEFAULT_POSES_CSV):
        poses = build_poses_from_csv(DEFAULT_POSES_CSV)
        print(f"Evaluating across {len(poses)} poses from default file: {DEFAULT_POSES_CSV}")
    else:
        R = quat_to_R(*QUAT_WXYZ)
        pos = np.array([PX, PY, PZ])
        poses = [(pos, R)]
        print(f"Default CSV not found at {DEFAULT_POSES_CSV}. Evaluating single fallback pose instead.")

    wfc_wrench = WFC_WRENCH
    if wfc_wrench is None:
        # Static gravity-compensation wrench (F=[0,0,mg], M=0)
        wfc_wrench = np.array([0.0, 0.0, PAYLOAD_MASS * G_ACCEL, 0.0, 0.0, 0.0])
        print(f"Using default static WFC target wrench (gravity compensation only): {wfc_wrench}")

    df = screen_all(
        poses, max_enumerate=MAX_ENUMERATE,
        enable_wfc=ENABLE_WFC, tau_min=GROUND_TAU_MIN, tau_max=GROUND_TAU_MAX, wfc_wrench=wfc_wrench,
        uav_layout=UAV_LAYOUT, uav_cable_length=UAV_CABLE_LENGTH,
        drone_thrust_min=DRONE_THRUST_MIN, drone_thrust_max=DRONE_THRUST_MAX,
        wfc_residual_tol=WFC_RESIDUAL_TOL, wfc_force_tol=WFC_FORCE_TOL, wfc_moment_tol=WFC_MOMENT_TOL,
        wfc_verify_top_k=WFC_VERIFY_TOP_K, wfc_verify_max_iter=WFC_VERIFY_MAX_ITER,
        enable_ifc=ENABLE_IFC, d_safe=D_SAFE,
    )

    if df.empty:
        print("\nNo architectures found during screening.")
        return

    if "conditioning_index" not in df.columns or not df["feasible"].any():
        print_infeasibility_report(df)
        return

    df = df.sort_values(by=["conditioning_index", "manipulability"], ascending=[False, False]).reset_index(drop=True)

    # Check whether identical results already exist, to avoid redundant work
    base_dir = "mujoco"
    existing_folders = glob.glob(os.path.join(base_dir, "mujoco_outputs*"))
    print("\nChecking existing results directories for identical data...")
    for folder in existing_folders:
        csv_path = os.path.join(folder, "ground_screening_results.csv")
        if os.path.exists(csv_path):
            try:
                existing_df = pd.read_csv(csv_path)
                if df.shape == existing_df.shape and df.equals(existing_df):
                    print("\n" + "=" * 80)
                    print("[SKIPPED] Identical screening results already exist.")
                    print(f"--> Please refer to this existing folder: '{folder}'")
                    print("=" * 80 + "\n")
                    return
            except Exception:
                continue

    counter = 1
    while os.path.exists(os.path.join(base_dir, f"mujoco_outputs_{counter}")):
        counter += 1
    target_output_dir = os.path.join(base_dir, f"mujoco_outputs_{counter}")
    os.makedirs(target_output_dir, exist_ok=True)
    print(f"--> No identical results found. Creating new directory: '{target_output_dir}'")

    csv_out_path = OUT_PATH if OUT_PATH else os.path.join(target_output_dir, "ground_screening_results.csv")
    os.makedirs(os.path.dirname(csv_out_path), exist_ok=True)
    df.to_csv(csv_out_path, index=False)
    print(f"\nWrote {len(df)} architecture results to {csv_out_path}")

    print("\nTop 5 architectures by worst-case conditioning index (each already using its best routing):")
    print(df.head(5).to_string(index=False))

    generate_top_xml_configs(df, target_output_dir, chosen_uav_layout="triangle")


if __name__ == "__main__":
    main()
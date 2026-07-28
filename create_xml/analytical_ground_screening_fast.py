import itertools
import os
import glob
import shutil
from collections import Counter
from concurrent.futures import ProcessPoolExecutor, as_completed

import numpy as np
import pandas as pd
from scipy.optimize import linprog, lsq_linear, minimize

from xml_config_builder import build_xml, save_xml, RoutingValidationError, UGVUAVConfig
from config_params import _load_json_db
from config_params import *
from acts_simulator import THRUST_MAX

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


def enumerate_routings(db, pay_layout, gnd_layout, n_cables=6, max_results=None, max_gnd_share=None):
    """
    Exhaustively yields every valid combination of n_cables distinct
    (payload_node, ground_node) edges respecting each node's max_cables.

    max_gnd_share, if given, caps how many cables any single ground node may
    take (previously enforced as a post-hoc filter in screen_architecture -
    pushing it in here means we never even generate/store the violators).
    """
    pay_letters = node_letters(db, pay_layout)
    gnd_letters = node_letters(db, gnd_layout)
    all_pairs = [(p, g) for p in pay_letters for g in gnd_letters]

    gnd_cap = {g: max_cables(db, gnd_layout, g) for g in gnd_letters}
    if max_gnd_share is not None:
        for g in gnd_cap:
            gnd_cap[g] = min(gnd_cap[g], max_gnd_share - 1)

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
            if gnd_counts.get(g, 0) >= gnd_cap[g]:
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


# ---------------------------------------------------------------------------
# Per-pose edge-row cache
#
# The old code recomputed bi/ai/ri/li/ui/cross(...) from scratch for every
# cable, in every routing, at every pose - even though the number of
# *distinct* (payload_node, ground_node) edges is tiny compared to the
# number of routings that reuse them. We precompute every edge's Jacobian
# row once per pose and look it up when assembling a routing's Jp.
# ---------------------------------------------------------------------------

def precompute_ground_edge_rows(ugv_db, pay_layout, gnd_layout, payload_pos, payload_R):
    """dict[(p_node, g_node)] -> 6-vector Jacobian row, for this one pose."""
    pay_letters = node_letters(ugv_db, pay_layout)
    gnd_letters = node_letters(ugv_db, gnd_layout)

    bi_cache = {p: np.array(ugv_db[pay_layout][p]["coords"], dtype=float) for p in pay_letters}
    ai_cache = {}
    for g in gnd_letters:
        ai = np.array(ugv_db[gnd_layout][g]["coords"], dtype=float).copy()
        ai[2] = 0.0
        ai_cache[g] = ai

    rows = {}
    ri_cache = {}
    for p, bi in bi_cache.items():
        ri_cache[p] = payload_pos + payload_R @ bi
    Rb_cache = {p: payload_R @ bi for p, bi in bi_cache.items()}

    for p in pay_letters:
        ri = ri_cache[p]
        Rb = Rb_cache[p]
        for g in gnd_letters:
            ai = ai_cache[g]
            li = ai - ri
            norm = np.linalg.norm(li)
            if norm < 1e-9:
                continue  # degenerate edge: anchor coincides with payload node - never usable
            ui = li / norm
            rows[(p, g)] = np.concatenate([ui, np.cross(Rb, ui)])

    return rows, ri_cache, ai_cache


def precompute_drone_rows(uav_hook_offsets, payload_pos, payload_R, uav_cable_length):
    """
    Drone Jacobian rows for one pose. These do NOT depend on the ground
    routing at all (only on pose + UAV layout + nominal drone placement),
    so they should be computed once per pose - not once per routing.
    """
    drone_positions = nominal_drone_positions(uav_hook_offsets, payload_pos, payload_R, uav_cable_length)
    rows = []
    for bi, ai in zip(uav_hook_offsets, drone_positions):
        ri = payload_pos + payload_R @ bi
        li = ai - ri
        ui = li / np.linalg.norm(li)
        rows.append(np.concatenate([ui, np.cross(payload_R @ bi, ui)]))
    return np.array(rows), drone_positions


def build_Jp_ground(routing, ugv_db, pay_layout, gnd_layout, payload_pos, payload_R=None, edge_rows=None):
    """
    Kept for standalone/back-compat use. If edge_rows (from
    precompute_ground_edge_rows) is supplied, this is an O(n_cables)
    dict lookup instead of recomputing geometry.
    """
    if edge_rows is not None:
        rows = []
        for pg in routing:
            row = edge_rows.get(pg)
            if row is None:
                return None
            rows.append(row)
        return np.array(rows)

    payload_R = np.eye(3) if payload_R is None else payload_R
    rows = []
    for p_node, g_node in routing:
        bi = np.array(ugv_db[pay_layout][p_node]["coords"], dtype=float)
        ai = np.array(ugv_db[gnd_layout][g_node]["coords"], dtype=float)
        ai = ai.copy()
        ai[2] = 0.0

        ri = payload_pos + payload_R @ bi
        li = ai - ri
        norm = np.linalg.norm(li)
        if norm < 1e-9:
            return None

        ui = li / norm
        Jp_row = np.concatenate([ui, np.cross(payload_R @ bi, ui)])
        rows.append(Jp_row)

    return np.array(rows)


def score_Jp(Jp):
    if Jp is None:
        return None
    svals = np.linalg.svd(Jp, compute_uv=False)
    sigma_min, sigma_max = svals[-1], svals[0]
    if sigma_max < 1e-9:
        return None
    return {
        "conditioning_index_ground": sigma_min / sigma_max,
        "manipulability_ground": float(np.prod(svals)),
        "sigma_min": sigma_min,
        "sigma_max": sigma_max,
        "rank_ok": bool(sigma_min > EPS_RANK),
    }


def batched_wcc_scores(Jp_stack):
    """
    Vectorized WCC scoring for many routings at once at a single pose.
    Jp_stack: (N, 6, 6) array. Returns per-routing dict of the same shape
    score_Jp would give, computed via ONE batched SVD call instead of N.
    """
    svals = np.linalg.svd(Jp_stack, compute_uv=False)  # (N, 6)
    sigma_min = svals[:, -1]
    sigma_max = svals[:, 0]
    manipulability = np.prod(svals, axis=1)
    conditioning_index = np.divide(sigma_min, sigma_max,
                                    out=np.full_like(sigma_min, np.nan), where=sigma_max > 1e-9)
    rank_ok = sigma_min > EPS_RANK
    valid = sigma_max > 1e-9
    return {
        "conditioning_index": conditioning_index,
        "manipulability": manipulability,
        "sigma_min": sigma_min,
        "sigma_max": sigma_max,
        "rank_ok": rank_ok & valid,
    }


def check_wfc(Jp, tau_min, tau_max, W_target):
    """Ground-only, exact-equality WFC check (the literal Eq. 1.9/2.35 formulation)."""
    m = Jp.shape[0]
    tau_min_vec = np.full(m, tau_min) if np.isscalar(tau_min) else np.asarray(tau_min, dtype=float)
    tau_max_vec = np.full(m, tau_max) if np.isscalar(tau_max) else np.asarray(tau_max, dtype=float)

    A_eq = Jp.T
    b_eq = np.asarray(W_target, dtype=float)
    bounds = list(zip(tau_min_vec, tau_max_vec))
    c = np.zeros(m)

    res = linprog(c, A_eq=A_eq, b_eq=b_eq, bounds=bounds, method="highs")
    if res.success:
        return {"wfc_ok": True, "tau": res.x}
    return {"wfc_ok": False, "tau": None}


def get_uav_hook_offsets(uav_db, uav_layout):
    letters = [k for k in uav_db[uav_layout].keys() if k != "symmetry_metadata"]
    return [np.array(uav_db[uav_layout][l]["coords"], dtype=float) for l in letters]


def nominal_drone_positions(uav_hook_offsets, payload_pos, payload_R, uav_cable_length):
    positions = []
    for bi in uav_hook_offsets:
        ri = payload_pos + payload_R @ bi
        positions.append(ri + np.array([0.0, 0.0, uav_cable_length]))
    return positions


def build_Jp_full(ground_routing, ugv_db, pay_layout, gnd_layout,
                   uav_hook_offsets, drone_positions, payload_pos, payload_R,
                   edge_rows=None, drone_rows=None):
    """Full (n_ground + n_drone, 6) Jacobian - ground rows first, then drone rows."""
    if edge_rows is not None:
        ground_part = [edge_rows[pg] for pg in ground_routing]
    else:
        ground_part = []
        for p_node, g_node in ground_routing:
            bi = np.array(ugv_db[pay_layout][p_node]["coords"], dtype=float)
            ai = np.array(ugv_db[gnd_layout][g_node]["coords"], dtype=float).copy()
            ai[2] = 0.0
            ri = payload_pos + payload_R @ bi
            li = ai - ri
            ui = li / np.linalg.norm(li)
            ground_part.append(np.concatenate([ui, np.cross(payload_R @ bi, ui)]))

    if drone_rows is not None:
        drone_part = list(drone_rows)
    else:
        drone_part = []
        for bi, ai in zip(uav_hook_offsets, drone_positions):
            ri = payload_pos + payload_R @ bi
            li = ai - ri
            ui = li / np.linalg.norm(li)
            drone_part.append(np.concatenate([ui, np.cross(payload_R @ bi, ui)]))

    return np.array(ground_part + drone_part)


def check_joint_wfc(Jp_full, n_ground, n_drone,
                     ground_tau_min, ground_tau_max,
                     drone_thrust_min, drone_thrust_max,
                     W_target, residual_tol=1.0,
                     residual_force_tol=None, residual_moment_tol=None):
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
                                      max_iter=200, edge_rows=None, x0=None):
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
                                 uav_hook_offsets, drone_positions, payload_pos, payload_R,
                                 edge_rows=edge_rows)
        wfc = check_joint_wfc(Jp_full, n_ground=n_ground, n_drone=n_drone,
                               ground_tau_min=ground_tau_min, ground_tau_max=ground_tau_max,
                               drone_thrust_min=drone_thrust_min, drone_thrust_max=drone_thrust_max,
                               W_target=W_target, residual_tol=0.0)
        return wfc["residual"]

    # Warm-starting from a nearby pose's solution (if supplied) converges
    # faster than always restarting at theta=0 for every pose.
    x0 = np.zeros(2 * n_drone) if x0 is None else np.asarray(x0, dtype=float)
    res = minimize(objective, x0, method="Nelder-Mead",
                    options={"xatol": 1e-3, "fatol": 1e-3, "maxiter": max_iter})

    best_positions = angles_to_positions(res.x)
    Jp_full_best = build_Jp_full(ground_routing, ugv_db, pay_layout, gnd_layout,
                                  uav_hook_offsets, best_positions, payload_pos, payload_R,
                                  edge_rows=edge_rows)
    best_wfc = check_joint_wfc(Jp_full_best, n_ground=n_ground, n_drone=n_drone,
                                ground_tau_min=ground_tau_min, ground_tau_max=ground_tau_max,
                                drone_thrust_min=drone_thrust_min, drone_thrust_max=drone_thrust_max,
                                W_target=W_target, residual_tol=0.0)
    return best_positions, best_wfc["residual"], best_wfc, res.x


# ---------------------------------------------------------------------------
# INTERFERENCE-FREE CONDITION (IFC) - Eq. 2.20
# ---------------------------------------------------------------------------

def segment_segment_distance(p1, p2, q1, q2):
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


def check_cable_exit_angle(routing, ugv_db, pay_layout, gnd_layout, payload_pos, payload_R,
                            min_exit_angle_deg=15.0, face_normal_local=None,
                            ri_cache=None, ai_cache=None):
    if face_normal_local is None:
        face_normal_local = np.array([0.0, 0.0, -1.0])
    world_normal = payload_R @ face_normal_local
    min_sin = np.sin(np.radians(min_exit_angle_deg))

    worst_margin = None
    for p_node, g_node in routing:
        if ri_cache is not None and ai_cache is not None:
            ri = ri_cache[p_node]
            ai = ai_cache[g_node]
        else:
            bi = np.array(ugv_db[pay_layout][p_node]["coords"], dtype=float)
            ai = np.array(ugv_db[gnd_layout][g_node]["coords"], dtype=float)
            ai = ai.copy()
            ai[2] = 0.0
            ri = payload_pos + payload_R @ bi
        direction = ai - ri
        norm = np.linalg.norm(direction)
        if norm < 1e-9:
            continue
        u = direction / norm
        sin_angle = np.dot(u, world_normal)
        margin = sin_angle - min_sin
        worst_margin = margin if worst_margin is None else min(worst_margin, margin)

    ok = (worst_margin is None) or (worst_margin >= 0.0)
    return {"exit_angle_ok": ok, "min_exit_angle_margin": worst_margin}


def check_ifc(routing, ugv_db, pay_layout, gnd_layout, payload_pos, payload_R, d_safe,
              check_exit_angle=False, min_exit_angle_deg=15.0, face_normal_local=None,
              ri_cache=None, ai_cache=None):

    segments = []
    for p_node, g_node in routing:
        if ri_cache is not None and ai_cache is not None:
            ri = ri_cache[p_node]
            ai = ai_cache[g_node]
        else:
            bi = np.array(ugv_db[pay_layout][p_node]["coords"], dtype=float)
            ai = np.array(ugv_db[gnd_layout][g_node]["coords"], dtype=float)
            ai = ai.copy()
            ai[2] = 0.0
            ri = payload_pos + payload_R @ bi
        segments.append((ri, ai, p_node, g_node))

    min_cc = None
    n_skipped = 0
    for (r1, a1, p1n, g1n), (r2, a2, p2n, g2n) in itertools.combinations(segments, 2):
        if p1n == p2n or g1n == g2n:
            n_skipped += 1
            continue
        d = segment_segment_distance(r1, a1, r2, a2)
        min_cc = d if min_cc is None else min(min_cc, d)

    cable_cable_ok = (min_cc is None) or (min_cc >= d_safe)

    if check_exit_angle:
        angle_result = check_cable_exit_angle(routing, ugv_db, pay_layout, gnd_layout,
                                               payload_pos, payload_R,
                                               min_exit_angle_deg=min_exit_angle_deg,
                                               face_normal_local=face_normal_local,
                                               ri_cache=ri_cache, ai_cache=ai_cache)
        exit_angle_ok = angle_result["exit_angle_ok"]
        min_exit_angle_margin = angle_result["min_exit_angle_margin"]
    else:
        exit_angle_ok = True
        min_exit_angle_margin = None

    ok = cable_cable_ok and exit_angle_ok

    return {
        "ifc_ok": ok,
        "cable_cable_ok": cable_cable_ok,
        "exit_angle_ok": exit_angle_ok,
        "min_cable_cable_dist": min_cc,
        "min_exit_angle_margin": min_exit_angle_margin,
        "n_pairs_skipped_shared_anchor": n_skipped,
    }


# ---------------------------------------------------------------------------
# Per-architecture screening (now cache- and batch-aware)
# ---------------------------------------------------------------------------

def screen_architecture(ugv_db, pay_layout, gnd_layout, poses, max_enumerate=20000,
                         enable_wfc=False, tau_min=5.0, tau_max=100.0, wfc_wrench=None,
                         uav_hook_offsets=None, uav_cable_length=1.5,
                         drone_thrust_min=1.0, drone_thrust_max=44.0, wfc_residual_tol=1.0,
                         wfc_force_tol=None, wfc_moment_tol=None,
                         wfc_verify_top_k=0, wfc_verify_max_iter=100,
                         enable_ifc=True, d_safe=D_SAFE_CABLE,
                         check_exit_angle=False, min_exit_angle_deg=15.0, face_normal_local=None,
                         max_gnd_share=MAX_GND_SHARE):
    upper_bound = count_routings_upper_bound(ugv_db, pay_layout, gnd_layout)
    if upper_bound == 0:
        return None

    if upper_bound <= max_enumerate:
        routings = enumerate_routings(ugv_db, pay_layout, gnd_layout, max_gnd_share=max_gnd_share)
    else:
        routings = enumerate_routings(ugv_db, pay_layout, gnd_layout, max_results=max_enumerate,
                                       max_gnd_share=max_gnd_share)

    if not routings:
        return {
            "pay_layout": pay_layout, "gnd_layout": gnd_layout,
            "n_routings_checked": 0, "n_poses_checked": len(poses), "feasible": False,
            "n_wcc_fail": 0, "n_wfc_fail": 0, "n_ifc_fail": 0,
            "n_ifc_fail_cable_cable": 0, "n_ifc_fail_exit_angle": 0,
        }

    # --- Precompute per-pose geometry once (not once per routing) ---
    pose_edge_rows, pose_ri_cache, pose_ai_cache, pose_drone_rows, pose_drone_pos = [], [], [], [], []
    for payload_pos, payload_R in poses:
        edge_rows, ri_cache, ai_cache = precompute_ground_edge_rows(
            ugv_db, pay_layout, gnd_layout, payload_pos, payload_R)
        pose_edge_rows.append(edge_rows)
        pose_ri_cache.append(ri_cache)
        pose_ai_cache.append(ai_cache)
        if enable_wfc:
            drone_rows, drone_positions = precompute_drone_rows(
                uav_hook_offsets, payload_pos, payload_R, uav_cable_length)
            pose_drone_rows.append(drone_rows)
            pose_drone_pos.append(drone_positions)
        else:
            pose_drone_rows.append(None)
            pose_drone_pos.append(None)

    # --- Vectorized WCC pre-filter: one batched SVD call per pose instead
    # of one SVD call per (routing, pose) pair. Any routing referencing a
    # degenerate edge (missing from edge_rows) is dropped immediately. ---
    valid_mask = np.ones(len(routings), dtype=bool)
    wcc_scores_per_pose = []  # list over poses of dict-of-arrays, aligned with `routings` order (NaN where invalid)

    for pose_idx in range(len(poses)):
        edge_rows = pose_edge_rows[pose_idx]
        Jp_list = []
        pose_valid = np.zeros(len(routings), dtype=bool)
        idx_map = []
        for i, routing in enumerate(routings):
            if not valid_mask[i]:
                continue
            rows = [edge_rows.get(pg) for pg in routing]
            if any(r is None for r in rows):
                valid_mask[i] = False
                continue
            Jp_list.append(np.array(rows))
            idx_map.append(i)

        scores = {
            "conditioning_index": np.full(len(routings), np.nan),
            "manipulability": np.full(len(routings), np.nan),
            "sigma_min": np.full(len(routings), np.nan),
            "sigma_max": np.full(len(routings), np.nan),
        }
        if Jp_list:
            Jp_stack = np.stack(Jp_list)  # (n_valid, 6, 6)
            batch = batched_wcc_scores(Jp_stack)
            for j, i in enumerate(idx_map):
                if not batch["rank_ok"][j]:
                    valid_mask[i] = False
                    continue
                scores["conditioning_index"][i] = batch["conditioning_index"][j]
                scores["manipulability"][i] = batch["manipulability"][j]
                scores["sigma_min"][i] = batch["sigma_min"][j]
                scores["sigma_max"][i] = batch["sigma_max"][j]
        wcc_scores_per_pose.append(scores)

    n_wcc_fail = int(len(routings) - np.count_nonzero(valid_mask))

    use_two_phase = enable_wfc and wfc_verify_top_k > 0

    stats = {"wcc_fail": n_wcc_fail, "wfc_fail": 0, "ifc_fail": 0, "passed": 0}
    best = None
    pending_candidates = []

    for i, routing in enumerate(routings):
        if not valid_mask[i]:
            continue

        per_pose_scores, per_pose_wfc, per_pose_ifc = [], [], []
        any_wfc_soft_fail = False
        failed = False

        for pose_idx, (payload_pos, payload_R) in enumerate(poses):
            scores = wcc_scores_per_pose[pose_idx]
            if np.isnan(scores["conditioning_index"][i]):
                failed = True
                break
            score = {
                "conditioning_index": scores["conditioning_index"][i],
                "manipulability": scores["manipulability"][i],
                "sigma_min": scores["sigma_min"][i],
                "sigma_max": scores["sigma_max"][i],
                "rank_ok": True,
            }

            if enable_ifc:
                ifc = check_ifc(routing, ugv_db, pay_layout, gnd_layout, payload_pos, payload_R, d_safe,
                                 check_exit_angle=check_exit_angle, min_exit_angle_deg=min_exit_angle_deg,
                                 face_normal_local=face_normal_local,
                                 ri_cache=pose_ri_cache[pose_idx], ai_cache=pose_ai_cache[pose_idx])
                if not ifc["ifc_ok"]:
                    stats["ifc_fail"] += 1
                    if not ifc["cable_cable_ok"]:
                        stats["ifc_fail_cable_cable"] = stats.get("ifc_fail_cable_cable", 0) + 1
                        if ifc["min_cable_cable_dist"] is not None:
                            stats.setdefault("ifc_fail_cc_dists", []).append(ifc["min_cable_cable_dist"])
                    if check_exit_angle and not ifc["exit_angle_ok"]:
                        stats["ifc_fail_exit_angle"] = stats.get("ifc_fail_exit_angle", 0) + 1
                        if ifc["min_exit_angle_margin"] is not None:
                            stats.setdefault("ifc_fail_angle_margins", []).append(ifc["min_exit_angle_margin"])
                    failed = True
                    break
                per_pose_ifc.append(ifc)

            if enable_wfc:
                edge_rows = pose_edge_rows[pose_idx]
                drone_rows = pose_drone_rows[pose_idx]
                drone_positions = pose_drone_pos[pose_idx]
                Jp_full = build_Jp_full(routing, ugv_db, pay_layout, gnd_layout,
                                         uav_hook_offsets, drone_positions, payload_pos, payload_R,
                                         edge_rows=edge_rows, drone_rows=drone_rows)
                wfc = check_joint_wfc(Jp_full, n_ground=len(routing), n_drone=len(uav_hook_offsets),
                                       ground_tau_min=tau_min, ground_tau_max=tau_max,
                                       drone_thrust_min=drone_thrust_min, drone_thrust_max=drone_thrust_max,
                                       W_target=wfc_wrench, residual_tol=wfc_residual_tol,
                                       residual_force_tol=wfc_force_tol, residual_moment_tol=wfc_moment_tol)
                if not wfc["joint_wfc_ok"]:
                    stats["wfc_fail"] += 1
                    stats.setdefault("wfc_fail_residuals", []).append(wfc["residual"])
                    stats.setdefault("wfc_fail_residual_force", []).append(wfc["residual_force"])
                    stats.setdefault("wfc_fail_residual_moment", []).append(wfc["residual_moment"])
                    stats.setdefault("wfc_fail_residual_force_lateral", []).append(wfc["residual_force_lateral"])
                    stats.setdefault("wfc_fail_residual_force_vertical", []).append(wfc["residual_force_vertical"])
                    if not use_two_phase:
                        failed = True
                        break
                    any_wfc_soft_fail = True
                per_pose_wfc.append(wfc)

            per_pose_scores.append(score)

        if failed:
            continue

        stats["passed"] += 1
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
            if check_exit_angle:
                angle_vals = [d["min_exit_angle_margin"] for d in per_pose_ifc if d["min_exit_angle_margin"] is not None]
                result["min_exit_angle_margin"] = min(angle_vals) if angle_vals else None

        if use_two_phase and not result.get("wfc_ok", True):
            pending_candidates.append((result["manipulability"], routing, result))
            continue

        if best is None or result["manipulability"] > best["manipulability"]:
            best = {**result, "routing": routing}

    n_wfc_verify_attempted = 0
    n_wfc_verify_rescued = 0
    if best is None and use_two_phase and pending_candidates:
        pending_candidates.sort(key=lambda t: t[0], reverse=True)
        for manipulability, routing, score in pending_candidates[:wfc_verify_top_k]:
            n_wfc_verify_attempted += 1
            all_poses_pass = True
            x0 = None  # warm-start across poses within the same routing
            for pose_idx, (payload_pos, payload_R) in enumerate(poses):
                _, residual, wfc_result, x_sol = optimize_drone_positions_for_wfc(
                    uav_hook_offsets, payload_pos, payload_R, uav_cable_length,
                    routing, ugv_db, pay_layout, gnd_layout,
                    tau_min, tau_max, drone_thrust_min, drone_thrust_max, wfc_wrench,
                    max_iter=wfc_verify_max_iter, edge_rows=pose_edge_rows[pose_idx], x0=x0,
                )
                x0 = x_sol  # warm start next pose from this one's solution
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
                break

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

    # How far off were the IFC failures, actually? Distinguishes "just
    # barely under d_safe / a couple degrees short" (loosen the threshold)
    # from "cables are essentially coincident / wildly negative exit angle"
    # (a geometry or normal-direction problem, not a threshold-tuning one).
    ifc_cc_dists = stats.get("ifc_fail_cc_dists", [])
    ifc_angle_margins = stats.get("ifc_fail_angle_margins", [])
    ifc_margin_summary = {
        "ifc_fail_cc_dist_min": float(np.min(ifc_cc_dists)) if ifc_cc_dists else None,
        "ifc_fail_cc_dist_median": float(np.median(ifc_cc_dists)) if ifc_cc_dists else None,
        "ifc_fail_cc_dist_max": float(np.max(ifc_cc_dists)) if ifc_cc_dists else None,
        "ifc_fail_angle_margin_min": float(np.min(ifc_angle_margins)) if ifc_angle_margins else None,
        "ifc_fail_angle_margin_median": float(np.median(ifc_angle_margins)) if ifc_angle_margins else None,
        "ifc_fail_angle_margin_max": float(np.max(ifc_angle_margins)) if ifc_angle_margins else None,
    }

    if best is None:
        return {
            "pay_layout": pay_layout, "gnd_layout": gnd_layout,
            "n_routings_checked": len(routings), "n_poses_checked": len(poses), "feasible": False,
            "n_wcc_fail": stats["wcc_fail"], "n_wfc_fail": stats["wfc_fail"], "n_ifc_fail": stats["ifc_fail"],
            "n_ifc_fail_cable_cable": stats.get("ifc_fail_cable_cable", 0),
            "n_ifc_fail_exit_angle": stats.get("ifc_fail_exit_angle", 0),
            **wfc_residual_summary,
            **ifc_margin_summary,
        }

    return {
        "pay_layout": pay_layout, "gnd_layout": gnd_layout,
        "n_routings_checked": len(routings), "feasible": True,
        "n_wcc_fail": stats["wcc_fail"], "n_wfc_fail": stats["wfc_fail"], "n_ifc_fail": stats["ifc_fail"],
        "n_ifc_fail_cable_cable": stats.get("ifc_fail_cable_cable", 0),
        "n_ifc_fail_exit_angle": stats.get("ifc_fail_exit_angle", 0),
        **wfc_residual_summary,
        **ifc_margin_summary,
        **{k: v for k, v in best.items() if k != "routing"},
        "best_routing": "|".join(f"{p}-{g}" for p, g in best["routing"]),
    }


# ---------------------------------------------------------------------------
# Top-level sweep - architectures are independent, so run them in a
# process pool instead of a plain sequential loop.
# ---------------------------------------------------------------------------

def _screen_one_architecture_worker(args):
    (pay_layout, gnd_layout, ugv_db_path, uav_db_path, poses, max_enumerate,
     enable_wfc, tau_min, tau_max, wfc_wrench, uav_layout, uav_cable_length,
     drone_thrust_min, drone_thrust_max, wfc_residual_tol, wfc_force_tol, wfc_moment_tol,
     wfc_verify_top_k, wfc_verify_max_iter, enable_ifc, d_safe,
     check_exit_angle, min_exit_angle_deg, face_normal_local, max_gnd_share) = args

    # Each worker process reloads the (small) JSON DBs itself - cheap, and
    # avoids pickling large/unpicklable state across the process boundary.
    ugv_db = _load_json_db(ugv_db_path)
    ugv_db.pop("rectangle-same", None)

    uav_hook_offsets = None
    if enable_wfc:
        uav_db = _load_json_db(uav_db_path)
        uav_hook_offsets = get_uav_hook_offsets(uav_db, uav_layout)

    result = screen_architecture(
        ugv_db, pay_layout, gnd_layout, poses, max_enumerate=max_enumerate,
        enable_wfc=enable_wfc, tau_min=tau_min, tau_max=tau_max, wfc_wrench=wfc_wrench,
        uav_hook_offsets=uav_hook_offsets, uav_cable_length=uav_cable_length,
        drone_thrust_min=drone_thrust_min, drone_thrust_max=drone_thrust_max,
        wfc_residual_tol=wfc_residual_tol, wfc_force_tol=wfc_force_tol, wfc_moment_tol=wfc_moment_tol,
        wfc_verify_top_k=wfc_verify_top_k, wfc_verify_max_iter=wfc_verify_max_iter,
        enable_ifc=enable_ifc, d_safe=d_safe,
        check_exit_angle=check_exit_angle, min_exit_angle_deg=min_exit_angle_deg,
        face_normal_local=face_normal_local, max_gnd_share=max_gnd_share,
    )
    return pay_layout, gnd_layout, result


def screen_all(poses, max_enumerate=20000,
                enable_wfc=False, tau_min=5.0, tau_max=100.0, wfc_wrench=None,
                uav_layout="triangle", uav_cable_length=1.5,
                drone_thrust_min=1.0, drone_thrust_max=44.0, wfc_residual_tol=1.0,
                wfc_force_tol=None, wfc_moment_tol=None,
                wfc_verify_top_k=0, wfc_verify_max_iter=100,
                enable_ifc=True, d_safe=D_SAFE_CABLE,
                check_exit_angle=False, min_exit_angle_deg=15.0, face_normal_local=None,
                max_gnd_share=MAX_GND_SHARE, n_workers=N_WORKERS):
    ugv_db = _load_json_db(UGV_DB_PATH)
    ugv_db.pop("rectangle-same", None)

    layouts = [l for l in GRID_MAPPING_UGV.keys() if l != "rectangle-same"]

    if enable_wfc:
        uav_db = _load_json_db(UAV_DB_PATH)
        n_hooks = len(get_uav_hook_offsets(uav_db, uav_layout))
        print(f"Joint WFC enabled: using UAV layout '{uav_layout}' "
              f"({n_hooks} drone cables) alongside the ground routing.")
        if wfc_verify_top_k > 0:
            print(f"Two-phase WFC verification enabled: up to {wfc_verify_top_k} "
                  f"best-by-manipulability WCC+IFC survivors per architecture will get the "
                  f"expensive drone-repositioning check if nothing passes the cheap one.")

    pairs = list(itertools.product(layouts, layouts))
    total = len(pairs)

    tasks = [
        (pay_layout, gnd_layout, UGV_DB_PATH, UAV_DB_PATH, poses, max_enumerate,
         enable_wfc, tau_min, tau_max, wfc_wrench, uav_layout, uav_cable_length,
         drone_thrust_min, drone_thrust_max, wfc_residual_tol, wfc_force_tol, wfc_moment_tol,
         wfc_verify_top_k, wfc_verify_max_iter, enable_ifc, d_safe,
         check_exit_angle, min_exit_angle_deg, face_normal_local, max_gnd_share)
        for pay_layout, gnd_layout in pairs
    ]

    rows = []
    n_done = 0

    if n_workers and n_workers > 1 and total > 1:
        with ProcessPoolExecutor(max_workers=n_workers) as pool:
            futures = {pool.submit(_screen_one_architecture_worker, t): t for t in tasks}
            for fut in as_completed(futures):
                pay_layout, gnd_layout, result = fut.result()
                n_done += 1
                if result is None:
                    print(f"[{n_done}/{total}] {pay_layout} / {gnd_layout} ... "
                          "no valid pairs (fewer nodes than 6 cables need)")
                    continue
                print(f"[{n_done}/{total}] {pay_layout} / {gnd_layout} ... "
                      f"checked {result['n_routings_checked']} routings across {len(poses)} pose(s), "
                      f"feasible={result['feasible']}")
                rows.append(result)
    else:
        for i, t in enumerate(tasks, 1):
            pay_layout, gnd_layout, result = _screen_one_architecture_worker(t)
            print(f"[{i}/{total}] {pay_layout} / {gnd_layout} ...", end=" ")
            if result is None:
                print("no valid pairs (fewer nodes than 6 cables need)")
                continue
            print(f"checked {result['n_routings_checked']} routings across {len(poses)} pose(s), "
                  f"feasible={result['feasible']}")
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


def print_infeasibility_report(df, wfc_residual_tol, d_safe, check_exit_angle, min_exit_angle_deg):
    n_wcc = int(df["n_wcc_fail"].sum()) if "n_wcc_fail" in df.columns else None
    n_wfc = int(df["n_wfc_fail"].sum()) if "n_wfc_fail" in df.columns else None
    n_ifc = int(df["n_ifc_fail"].sum()) if "n_ifc_fail" in df.columns else None
    n_ifc_cc = int(df["n_ifc_fail_cable_cable"].sum()) if "n_ifc_fail_cable_cable" in df.columns else None
    n_ifc_angle = int(df["n_ifc_fail_exit_angle"].sum()) if "n_ifc_fail_exit_angle" in df.columns else None

    print("\n" + "=" * 80)
    print("[NO FEASIBLE ARCHITECTURES] Every (pay_layout, gnd_layout) pair had")
    print("zero routings survive. Rejection counts across ALL architectures checked:")
    print(f"    failed WCC (rank-deficient / degenerate)                                   : {n_wcc}")
    print(f"    failed WFC (residual mismatch > tolerance, joint 9-cable solve)             : {n_wfc}")
    if check_exit_angle:
        print(f"    failed IFC (cable-cable clearance or exit angle)                            : {n_ifc}")
        print(f"        - of which, cable-cable clearance specifically                         : {n_ifc_cc}")
        print(f"        - of which, exit angle specifically                                     : {n_ifc_angle}")
    else:
        print(f"    failed IFC (cable-cable clearance < d_safe; exit-angle check is OFF)        : {n_ifc}")

    if n_ifc and "ifc_fail_cc_dist_median" in df.columns:
        cc_min = df["ifc_fail_cc_dist_min"].dropna()
        cc_med = df["ifc_fail_cc_dist_median"].dropna()
        cc_max = df["ifc_fail_cc_dist_max"].dropna()
        if len(cc_med) > 0:
            print(f"    Cable-cable clearance ACTUALLY achieved on failures (d_safe={d_safe} m): "
                  f"closest={cc_min.min():.4g} m, typical={cc_med.median():.4g} m, "
                  f"farthest-that-still-failed={cc_max.max():.4g} m")
            print("    -> if 'typical' is only slightly below d_safe, a small relaxation of d_safe "
                  "will likely fix most of these. If it's far below (e.g. cables nearly coincident "
                  "or negative), that's a geometry issue (nodes too close together / cables "
                  "converging at the payload), not a threshold-tuning one.")
    if check_exit_angle and n_ifc and "ifc_fail_angle_margin_median" in df.columns:
        am_min = df["ifc_fail_angle_margin_min"].dropna()
        am_med = df["ifc_fail_angle_margin_median"].dropna()
        am_max = df["ifc_fail_angle_margin_max"].dropna()
        if len(am_med) > 0:
            # margin = sin(actual_angle) - sin(min_exit_angle_deg); negative means it failed
            print(f"    Exit-angle margin (sin-space) on failures (threshold={min_exit_angle_deg} deg): "
                  f"worst={am_min.min():.4g}, typical={am_med.median():.4g}, "
                  f"best-that-still-failed={am_max.max():.4g}")
            print("    -> if 'typical' is only slightly negative, lowering min_exit_angle_deg a "
                  "few degrees will likely fix most of these. If it's strongly negative across the "
                  "board (e.g. cables exiting near-parallel to the face, or on the wrong side of it), "
                  "double check FACE_NORMAL_LOCAL actually matches your payload's real mounting face "
                  "orientation - a flipped or mismatched normal fails almost every routing regardless "
                  "of geometry or pose.")

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

    if check_exit_angle:
        print(f"If IFC dominates: check whether it's d_safe (cable-cable, currently {d_safe} m) "
              f"or the exit-angle threshold (currently {min_exit_angle_deg} deg) - try disabling "
              "IFC (or just CHECK_EXIT_ANGLE) to isolate the cause, then relax whichever is too "
              "strict for your geometry.")
    else:
        print(f"If IFC dominates: check whether d_safe (currently {d_safe} m, cable-cable "
              "clearance only - exit-angle check is OFF) is too strict for your geometry - try "
              "disabling IFC to confirm it's the cause, then relax d_safe if needed.")
    print(f"If WFC dominates: the residual tolerance (currently {wfc_residual_tol}) is a "
          "placeholder - try loosening it, or check cable length and drone thrust bounds are "
          "realistic; also verify get_uav_hook_offsets' schema assumption matches your "
          "uav_configuration_database.json.")
    print("=" * 80 + "\n")


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


def copy_file_to_folder(source_file_path, destination_folder_path):
    if not os.path.exists(source_file_path):
        raise FileNotFoundError(f"Source file not found: {source_file_path}")

    os.makedirs(destination_folder_path, exist_ok=True)

    try:
        shutil.copy2(source_file_path, destination_folder_path)
        file_name = os.path.basename(source_file_path)
        print(f"Successfully copied '{file_name}' to '{destination_folder_path}'")
    except IOError as e:
        print(f"Failed to copy file due to error: {e}")


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
        wfc_wrench = np.array([0.0, 0.0, PAYLOAD_MASS * G_ACCEL, 0.0, 0.0, 0.0])
        print(f"Using default static WFC target wrench (gravity compensation only): {wfc_wrench}")

    df = screen_all(
        poses, max_enumerate=MAX_ENUMERATE,
        enable_wfc=ENABLE_WFC, tau_min=GROUND_TAU_MIN, tau_max=GROUND_TAU_MAX, wfc_wrench=wfc_wrench,
        uav_layout=UAV_LAYOUT, uav_cable_length=UAV_CABLE_LENGTH,
        drone_thrust_min=DRONE_THRUST_MIN, drone_thrust_max=THRUST_MAX,
        wfc_residual_tol=WFC_RESIDUAL_TOL, wfc_force_tol=WFC_FORCE_TOL, wfc_moment_tol=WFC_MOMENT_TOL,
        wfc_verify_top_k=WFC_VERIFY_TOP_K, wfc_verify_max_iter=WFC_VERIFY_MAX_ITER,
        enable_ifc=ENABLE_IFC, d_safe=D_SAFE,
        check_exit_angle=CHECK_EXIT_ANGLE, min_exit_angle_deg=MIN_EXIT_ANGLE_DEG,
        face_normal_local=FACE_NORMAL_LOCAL,
        max_gnd_share=MAX_GND_SHARE, n_workers=N_WORKERS,
    )

    if df.empty:
        print("\nNo architectures found during screening.")
        return

    if "conditioning_index" not in df.columns or not df["feasible"].any():
        print_infeasibility_report(df, WFC_RESIDUAL_TOL, D_SAFE, CHECK_EXIT_ANGLE, MIN_EXIT_ANGLE_DEG)
        return

    df = df.sort_values(by=["conditioning_index", "manipulability"], ascending=[False, False]).reset_index(drop=True)

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
    copy_file_to_folder(DEFAULT_POSES_CSV, f"mujoco/mujoco_outputs_{counter}")


if __name__ == "__main__":
    main()
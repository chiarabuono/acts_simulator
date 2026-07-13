import argparse
import itertools

import numpy as np
import pandas as pd
from scipy.optimize import linprog, minimize_scalar

from xml_config_builder import build_xml, save_xml, RoutingValidationError, UGVUAVConfig
from config_params import _load_json_db
from config_params import UAV_DB_PATH, UGV_DB_PATH, GRID_MAPPING_UGV, TAU_MIN, TAU_MAX, D_SAFE

import os
import glob

EPS_RANK = 1e-6


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
        ai[2] = 0.0  # ground anchors sit at z=0, matching the wizard's XML generation

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
        "manipulability": float(np.prod(svals)),  # sqrt(det(Jp Jp^T)) == product of singular values
        "sigma_min": sigma_min,
        "sigma_max": sigma_max,
        "rank_ok": bool(sigma_min > EPS_RANK),
    }


# ---------------------------------------------------------------------------
# WRENCH FEASIBLE CONDITION (WFC) - Eq. 1.9 / 2.35
# ---------------------------------------------------------------------------

def check_wfc(Jp, tau_min, tau_max, W_target):
    m = Jp.shape[0]
    tau_min_vec = np.full(m, tau_min) if np.isscalar(tau_min) else np.asarray(tau_min, dtype=float)
    tau_max_vec = np.full(m, tau_max) if np.isscalar(tau_max) else np.asarray(tau_max, dtype=float)

    A_eq = Jp.T  # (6, m)
    b_eq = np.asarray(W_target, dtype=float)
    bounds = list(zip(tau_min_vec, tau_max_vec))
    c = np.zeros(m)  # feasibility only - no preference among feasible tau

    res = linprog(c, A_eq=A_eq, b_eq=b_eq, bounds=bounds, method="highs")
    if res.success:
        return {"wfc_ok": True, "tau": res.x}
    return {"wfc_ok": False, "tau": None}


def segment_segment_distance(p1, p2, q1, q2):
    """
    Exact minimum distance between two 3D line segments [p1,p2] and [q1,q2].
    Standard closed-form closest-point algorithm (Ericson, "Real-Time
    Collision Detection"). Used for the cable-cable clearance term of
    Eq. 2.20: d_ij = dist(cable_i, cable_j) >= d_safe.
    """
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
                            min_exit_angle_deg=15.0, face_normal_local=None):
    """
    Returns dict:
        exit_angle_ok            : bool
        min_exit_angle_margin    : float or None - smallest (sin(actual
                                    angle) - sin(min_exit_angle_deg)) across
                                    all cables; negative means a violation
    """
    if face_normal_local is None:
        face_normal_local = np.array([0.0, 0.0, -1.0])
    world_normal = payload_R @ face_normal_local
    min_sin = np.sin(np.radians(min_exit_angle_deg))

    worst_margin = None
    for p_node, g_node in routing:
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
        sin_angle = np.dot(u, world_normal)  # projection onto the outward normal
        margin = sin_angle - min_sin
        worst_margin = margin if worst_margin is None else min(worst_margin, margin)

    ok = (worst_margin is None) or (worst_margin >= 0.0)
    return {"exit_angle_ok": ok, "min_exit_angle_margin": worst_margin}


def check_ifc(routing, ugv_db, pay_layout, gnd_layout, payload_pos, payload_R,
              d_safe, min_exit_angle_deg=15.0, face_normal_local=None):

    segments = []  # (r_i, a_i, p_node, g_node) per cable, world frame
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

    # cable exit angle vs. the payload's attachment face
    angle_result = check_cable_exit_angle(routing, ugv_db, pay_layout, gnd_layout,
                                           payload_pos, payload_R,
                                           min_exit_angle_deg=min_exit_angle_deg,
                                           face_normal_local=face_normal_local)

    ok = True
    if min_cc is not None and min_cc < d_safe:
        ok = False
    if not angle_result["exit_angle_ok"]:
        ok = False

    return {
        "ifc_ok": ok,
        "min_cable_cable_dist": min_cc,
        "min_exit_angle_margin": angle_result["min_exit_angle_margin"],
        "n_pairs_skipped_shared_anchor": n_skipped,
    }


def score_routing_across_poses(routing, ugv_db, pay_layout, gnd_layout, poses,
                                enable_wfc=False, tau_min=5.0, tau_max=100.0,
                                wfc_wrench=None,
                                enable_ifc=True, d_safe=D_SAFE,
                                min_exit_angle_deg=15.0, face_normal_local=None,
                                stats=None):
    """
    Scores one routing at every (position, R) in `poses` and returns the
    WORST-CASE result across them (min manipulability, min conditioning
    index) - so a routing only looks good here if it's good everywhere it
    was checked, not just at whichever pose happened to flatter it.
    Returns None if the routing is infeasible (degenerate, rank-deficient,
    WFC-infeasible, or IFC-violating) at ANY of the given poses.

    """
    if wfc_wrench is None:
        wfc_wrench = np.zeros(6)

    per_pose_scores = []
    per_pose_wfc = []
    per_pose_ifc = []

    for payload_pos, payload_R in poses:
        Jp = build_Jp_ground(routing, ugv_db, pay_layout, gnd_layout, payload_pos, payload_R)
        score = score_Jp(Jp)
        if score is None or not score["rank_ok"]:
            if stats is not None:
                stats["wcc_fail"] = stats.get("wcc_fail", 0) + 1
            return None 

        if enable_wfc:
            wfc = check_wfc(Jp, tau_min, tau_max, wfc_wrench)
            if not wfc["wfc_ok"]:
                if stats is not None:
                    stats["wfc_fail"] = stats.get("wfc_fail", 0) + 1
                return None 
            per_pose_wfc.append(wfc)

        if enable_ifc:
            ifc = check_ifc(routing, ugv_db, pay_layout, gnd_layout, payload_pos, payload_R,
                             d_safe, min_exit_angle_deg=min_exit_angle_deg,
                             face_normal_local=face_normal_local)
            if not ifc["ifc_ok"]:
                if stats is not None:
                    stats["ifc_fail"] = stats.get("ifc_fail", 0) + 1
                return None
            per_pose_ifc.append(ifc)

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
        result["wfc_ok"] = True 
    if enable_ifc:
        cc_vals = [d["min_cable_cable_dist"] for d in per_pose_ifc if d["min_cable_cable_dist"] is not None]
        angle_vals = [d["min_exit_angle_margin"] for d in per_pose_ifc if d["min_exit_angle_margin"] is not None]
        result["ifc_ok"] = True
        result["min_cable_cable_dist"] = min(cc_vals) if cc_vals else None
        result["min_exit_angle_margin"] = min(angle_vals) if angle_vals else None

    return result


# ---------------------------------------------------------------------------
# Full screening loop
# ---------------------------------------------------------------------------

def screen_architecture(ugv_db, pay_layout, gnd_layout, poses, max_enumerate=20000, seed=0,
                         enable_wfc=False, tau_min=5.0, tau_max=100.0, wfc_wrench=None,
                         enable_ifc=True, d_safe=D_SAFE,
                         min_exit_angle_deg=15.0, face_normal_local=None):
    """
    Picks the routing with the best WORST-CASE manipulability across all poses,
    among routings that pass WCC (always) and, if enabled, WFC and IFC too.
    """
    upper_bound = count_routings_upper_bound(ugv_db, pay_layout, gnd_layout)
    if upper_bound == 0:
        return None

    if upper_bound <= max_enumerate:
        routings = enumerate_routings(ugv_db, pay_layout, gnd_layout)
    else:
        routings = enumerate_routings(ugv_db, pay_layout, gnd_layout, max_results=max_enumerate)

    stats = {"wcc_fail": 0, "wfc_fail": 0, "ifc_fail": 0, "passed": 0}
    best = None
    for routing in routings:
        gnd_nodes_used = [g for (p, g) in routing]
        from collections import Counter
        gnd_counts = Counter(gnd_nodes_used)

        if any(count >= 3 for count in gnd_counts.values()):
            continue

        score = score_routing_across_poses(
            routing, ugv_db, pay_layout, gnd_layout, poses,
            enable_wfc=enable_wfc, tau_min=tau_min, tau_max=tau_max, wfc_wrench=wfc_wrench,
            enable_ifc=enable_ifc, d_safe=d_safe,
            min_exit_angle_deg=min_exit_angle_deg, face_normal_local=face_normal_local,
            stats=stats,
        )
        if score is None:
            continue
        if best is None or score["manipulability"] > best["manipulability"]:
            best = {**score, "routing": routing}

    if best is None:
        return {
            "pay_layout": pay_layout, "gnd_layout": gnd_layout,
            "n_routings_checked": len(routings), "n_poses_checked": len(poses), "feasible": False,
            "n_wcc_fail": stats["wcc_fail"], "n_wfc_fail": stats["wfc_fail"], "n_ifc_fail": stats["ifc_fail"],
        }

    return {
        "pay_layout": pay_layout, "gnd_layout": gnd_layout,
        "n_routings_checked": len(routings), "feasible": True,
        "n_wcc_fail": stats["wcc_fail"], "n_wfc_fail": stats["wfc_fail"], "n_ifc_fail": stats["ifc_fail"],
        **{k: v for k, v in best.items() if k != "routing"},
        "best_routing": "|".join(f"{p}-{g}" for p, g in best["routing"]),
    }


def screen_all(poses, max_enumerate=20000, seed=0,
                enable_wfc=False, tau_min=5.0, tau_max=100.0, wfc_wrench=None,
                enable_ifc=True, d_safe=D_SAFE,
                min_exit_angle_deg=15.0, face_normal_local=None):
    ugv_db = _load_json_db(UGV_DB_PATH)
    ugv_db.pop("rectangle-same", None)

    pay_layouts = list(GRID_MAPPING_UGV.keys())
    gnd_layouts = list(GRID_MAPPING_UGV.keys())

    if "rectangle-same" in pay_layouts:
        pay_layouts.remove("rectangle-same")
    if "rectangle-same" in gnd_layouts:
        gnd_layouts.remove("rectangle-same")

    rows = []
    total = len(pay_layouts) * len(gnd_layouts)
    for i, (pay_layout, gnd_layout) in enumerate(itertools.product(pay_layouts, gnd_layouts), 1):
        print(f"[{i}/{total}] {pay_layout} / {gnd_layout} ...", end=" ")
        result = screen_architecture(
            ugv_db, pay_layout, gnd_layout, poses, max_enumerate=max_enumerate, seed=seed,
            enable_wfc=enable_wfc, tau_min=tau_min, tau_max=tau_max, wfc_wrench=wfc_wrench,
            enable_ifc=enable_ifc, d_safe=d_safe,
            min_exit_angle_deg=min_exit_angle_deg, face_normal_local=face_normal_local,
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


def main():
    ap = argparse.ArgumentParser()
    
    ap.add_argument("--out", type=str, default=None, help="Optional specific output CSV path override")
    ap.add_argument("--px", type=float, default=0.5, help="single-pose x (used only if CSVs are missing)")
    ap.add_argument("--py", type=float, default=-0.5, help="single-pose y")
    ap.add_argument("--pz", type=float, default=2.0, help="single-pose z")
    ap.add_argument("--quat", type=float, nargs=4, default=[1.0, 0.0, 0.0, 0.0],
                     metavar=("W", "X", "Y", "Z"), help="single-pose quaternion, w x y z")
    ap.add_argument("--poses-csv", type=str, default=None,
                     help="CSV with columns px,py,pz,quat_w,quat_x,quat_y,quat_z; overrides default file")
    ap.add_argument("--max-enumerate", type=int, default=20000,
                     help="per-architecture cap; falls back to a large sample beyond this")
    ap.add_argument("--seed", type=int, default=0)

    # --- WFC (Eq. 1.9 / 2.35) ---
    ap.add_argument("--enable-wfc", action="store_true",
                     help="Enable Wrench Feasible Condition check. OFF by default: this check "
                          "demands EXACT equality (J_p^T tau = W_target) from the 6 ground "
                          "cables ALONE, but your live controller (utils_optimization.py's "
                          "lsq_linear) solves for all 9 cables JOINTLY as a least-squares best "
                          "fit, with drones absorbing whatever the ground cables can't provide. "
                          "There's no principled single W_target to hand this check that matches "
                          "how the live system actually behaves - see conversation history for "
                          "the full reasoning. A routing this rejects may work fine live. Only "
                          "enable this once you've decided on an explicit ground/drone wrench "
                          "split, or built a joint (9-cable) version of this check instead.")
    ap.add_argument("--tau-min", type=float, default=TAU_MIN,
                     help=f"minimum admissible cable tension [N], from config_params.TAU_MIN (default {TAU_MIN})")
    ap.add_argument("--tau-max", type=float, default=TAU_MAX,
                     help=f"maximum admissible cable tension [N], from config_params.TAU_MAX (default {TAU_MAX} - "
                          f"still a placeholder, see config_params.py)")
    ap.add_argument("--wfc-wrench", type=float, nargs=6, default=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                     metavar=("FX", "FY", "FZ", "MX", "MY", "MZ"),
                     help="target ground-cable wrench W_target for the WFC check - "
                          "REPLACE with what your tension planner actually asks the ground cables for (Eq. 2.35)")

    ap.add_argument("--disable-ifc", dest="enable_ifc", action="store_false",
                     help="Turn OFF the Interference-Free Condition check (it's ON by default now "
                          "that config_params has a real D_SAFE value)")
    ap.set_defaults(enable_ifc=True)
    ap.add_argument("--d-safe", type=float, default=D_SAFE,
                     help=f"minimum clearance [m] between cables (cable-cable only) (default {D_SAFE})")

    ap.add_argument("--face-normal-local", type=float, nargs=3, default=[0.0, 0.0, -1.0],
                     metavar=("NX", "NY", "NZ"),
                     help="outward normal of the payload's attachment face, in the payload's own "
                          "body frame. Default (0,0,-1) assumes all attachments are on the bottom "
                          "facade, per utils_optimization.py's stated assumption - change this if "
                          "that's not true for your setup")

    args = ap.parse_args()

    default_csv_path = "create_xml/poses_to_analyze.csv"

    if args.poses_csv:
        poses = build_poses_from_csv(args.poses_csv)
        print(f"Evaluating across {len(poses)} poses from user-specified {args.poses_csv}")
    elif os.path.exists(default_csv_path):
        poses = build_poses_from_csv(default_csv_path)
        print(f"Evaluating across {len(poses)} poses from default file: {default_csv_path}")
    else:
        R = quat_to_R(*args.quat)
        pos = np.array([args.px, args.py, args.pz])
        poses = [(pos, R)]
        print(f"⚠️ Default CSV not found at {default_csv_path}. Evaluating single command-line pose instead.")

    face_normal_local = np.array(args.face_normal_local, dtype=float)
    wfc_wrench = np.array(args.wfc_wrench, dtype=float)

    # 2. Run the exhaustive screening loop in memory
    df = screen_all(
        poses, max_enumerate=args.max_enumerate, seed=args.seed,
        enable_wfc=args.enable_wfc, tau_min=args.tau_min, tau_max=args.tau_max, wfc_wrench=wfc_wrench,
        enable_ifc=args.enable_ifc, d_safe=args.d_safe,
        min_exit_angle_deg=args.min_exit_angle_deg, face_normal_local=face_normal_local,
    )

    if df.empty:
        print("\nNo architectures found during screening.")
        return

    if "conditioning_index" not in df.columns or not df["feasible"].any():
        # Every architecture came back infeasible - no routing survived WCC/WFC/IFC
        # for ANY (pay_layout, gnd_layout) pair. Aggregate the rejection-reason
        # counters (added specifically for this situation) so you can tell WHICH
        # check is doing this, instead of a bare crash with no diagnosis.
        n_wcc = int(df["n_wcc_fail"].sum()) if "n_wcc_fail" in df.columns else None
        n_wfc = int(df["n_wfc_fail"].sum()) if "n_wfc_fail" in df.columns else None
        n_ifc = int(df["n_ifc_fail"].sum()) if "n_ifc_fail" in df.columns else None
        print("\n" + "=" * 80)
        print("[NO FEASIBLE ARCHITECTURES] Every (pay_layout, gnd_layout) pair had")
        print("zero routings survive. Rejection counts across ALL architectures checked:")
        print(f"    failed WCC (rank-deficient / degenerate) : {n_wcc}")
        print(f"    failed WFC (tension outside [tau_min, tau_max] for wfc_wrench) : {n_wfc}")
        print(f"    failed IFC (cable-cable clearance < d_safe, or exit angle < min_exit_angle_deg) : {n_ifc}")
        print("If IFC dominates: check whether it's d_safe (cable-cable, currently "
              f"{args.d_safe} m) or --min-exit-angle-deg (currently {args.min_exit_angle_deg} deg) "
              "that's doing it - try --disable-ifc to confirm IFC is the cause, then relax "
              "whichever threshold is too strict for your geometry.")
        print("If WFC dominates: --wfc-wrench is still a zero-vector placeholder by "
              "default - an all-zero target wrench with tau_min>0 may be infeasible for "
              "some routing geometries (this can be a legitimate result, not just a "
              "placeholder problem); supply your real target wrench if you have one, or "
              "use --disable-wfc while you work out what that should be.")
        print("=" * 80 + "\n")
        return

    df = df.sort_values(by=["conditioning_index", "manipulability"], ascending=[False, False]).reset_index(drop=True)

    base_dir = "mujoco"
    search_pattern = os.path.join(base_dir, "mujoco_outputs*")
    existing_folders = glob.glob(search_pattern)

    match_found = False
    referring_folder = None

    print("\nChecking existing results directories for identical data...")
    for folder in existing_folders:
        csv_path = os.path.join(folder, "ground_screening_results.csv")
        if os.path.exists(csv_path):
            try:
                existing_df = pd.read_csv(csv_path)
                # Ensure dataframes match structurally and element-by-element
                if df.shape == existing_df.shape and df.equals(existing_df):
                    match_found = True
                    referring_folder = folder
                    break
            except Exception:
                continue

    if match_found:
        print("\n" + "=" * 80)
        print(f"[SKIPPED] Identical screening results already exist.")
        print(f"--> Please refer to this existing folder: '{referring_folder}'")
        print("=" * 80 + "\n")
        return 

    else:
        counter = 1
        while os.path.exists(os.path.join(base_dir, f"mujoco_outputs_{counter}")):
            counter += 1

        target_output_dir = os.path.join(base_dir, f"mujoco_outputs_{counter}")
        os.makedirs(target_output_dir, exist_ok=True)
        print(f"--> No identical results found. Creating new directory: '{target_output_dir}'")

    csv_out_path = args.out if args.out else os.path.join(target_output_dir, "ground_screening_results.csv")
    os.makedirs(os.path.dirname(csv_out_path), exist_ok=True)

    df.to_csv(csv_out_path, index=False)
    print(f"\nWrote {len(df)} architecture results to {csv_out_path}")

    print("\nTop 5 architectures by worst-case conditioning index (each already using its best routing):")
    print(df.head(5).to_string(index=False))

    uav_geo_db = _load_json_db(UAV_DB_PATH)
    ugv_geo_db = _load_json_db(UGV_DB_PATH)
    chosen_uav_layout = "triangle"

    print("\nGenerating MuJoCo XML configurations for the top architectures...")
    top_feasible_df = df[df["feasible"] == True].head(5)

    for idx, row in top_feasible_df.iterrows():
        routing_string = row["best_routing"]
        routing_pairs = [pair.split("-") for pair in routing_string.split("|")]

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
            scale_mode="Normal"
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


if __name__ == "__main__":
    main()
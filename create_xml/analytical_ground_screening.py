"""
USAGE: single pose via --px --py --pz --quat W X Y Z (defaults match your
ctrl_params target: 0.5 -0.5 2.0 / 1 0 0 0), or several poses at once via
--poses-csv pointing at a CSV with columns px,py,pz,quat_w,quat_x,quat_y,quat_z

To turn on the new checks:
    --enable-wfc --tau-min 5 --tau-max 100 --wfc-wrench 0 0 0 0 0 0
    --enable-ifc --d-safe 0.05 --payload-half-extents 0.3 0.3 0.05
"""

import argparse
import itertools
import json
import random

import numpy as np
import pandas as pd
from scipy.optimize import linprog, minimize_scalar

from config_params import UGV_DB_PATH, GRID_MAPPING_UGV
from xml_config_builder import build_xml, save_xml, RoutingValidationError, UGVUAVConfig
from config_params import UAV_DB_PATH, _load_json_db, TAU_MIN, TAU_MAX, D_SAFE, PAYLOAD_HALF_EXTENTS
import json
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


# ---------------------------------------------------------------------------
# DB helpers
# ---------------------------------------------------------------------------

def node_letters(db, layout):
    return [k for k in db[layout].keys() if k != "symmetry_metadata"]

def max_cables(db, layout, letter):
    return db[layout][letter].get("max_cables", 999)


# ---------------------------------------------------------------------------
# Exhaustive routing enumeration (capacity-aware, dedup built in by construction)
# ---------------------------------------------------------------------------

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
        "conditioning_index": sigma_min / sigma_max,
        "manipulability": float(np.prod(svals)),  # sqrt(det(Jp Jp^T)) == product of singular values
        "sigma_min": sigma_min,
        "sigma_max": sigma_max,
        "rank_ok": bool(sigma_min > EPS_RANK),
    }


def check_wfc(Jp, tau_min, tau_max, W_target):
    """
    Tests whether there EXISTS a tension vector tau, with
        tau_min <= tau_i <= tau_max   for every cable   (cables only pull),
    such that
        J_p^T tau = W_target
    (Eq. 1.9, restated as the equality-constrained feasibility problem of
    Eq. 2.35 with the LS-relaxation term set to zero). Solved as an LP
    feasibility problem via scipy.optimize.linprog (zero objective - we only
    care whether the feasible set is non-empty, not which point in it is
    "best"; if you want the specific solution your tension planner would
    pick, replace the objective with your actual H matrix from Eq. 2.35/2.36).

    Jp: (m, 6) ground Jacobian (m = number of ground cables, 6 here).
    tau_min, tau_max: scalars or length-m arrays, your actuator/cable limits.
    W_target: (6,) desired ground-cable wrench at this pose.

    Returns dict with:
        wfc_ok   : bool, feasible or not
        tau      : the feasible tension vector if found, else None
    """
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


def cable_box_distance(seg_start, seg_end, box_center, box_R, box_half_extents):
    Rt = box_R.T
    p1_l = Rt @ (seg_start - box_center)
    p2_l = Rt @ (seg_end - box_center)

    def sqdist_at(t):
        pt = p1_l + t * (p2_l - p1_l)
        d = np.maximum(np.abs(pt) - box_half_extents, 0.0)
        return float(np.dot(d, d))

    res = minimize_scalar(sqdist_at, bounds=(0.0, 1.0), method="bounded")
    return float(np.sqrt(max(res.fun, 0.0)))


def check_ifc(routing, ugv_db, pay_layout, gnd_layout, payload_pos, payload_R,
              d_safe, payload_half_extents=None):
    """
    Full IFC check (Eq. 2.20) for one routing at one pose:
      (a) every pair of ground cables must clear each other by >= d_safe
      (b) every ground cable must clear the payload body by >= d_safe
          (skipped, with a one-time warning upstream, if
          payload_half_extents is None - you haven't told this script your
          payload's dimensions yet)

    Returns dict:
        ifc_ok                    : bool
        min_cable_cable_dist      : float or None (None if <2 cables)
        min_cable_payload_dist    : float or None (None if box not given)
    """
    segments = []  # (r_i, a_i) per cable, world frame
    for p_node, g_node in routing:
        bi = np.array(ugv_db[pay_layout][p_node]["coords"], dtype=float)
        ai = np.array(ugv_db[gnd_layout][g_node]["coords"], dtype=float)
        ai = ai.copy()
        ai[2] = 0.0
        ri = payload_pos + payload_R @ bi
        segments.append((ri, ai))

    # (a) cable-cable clearance
    min_cc = None
    for (r1, a1), (r2, a2) in itertools.combinations(segments, 2):
        d = segment_segment_distance(r1, a1, r2, a2)
        min_cc = d if min_cc is None else min(min_cc, d)

    # (b) cable-payload clearance
    min_cp = None
    if payload_half_extents is not None:
        for ri, ai in segments:
            d = cable_box_distance(ri, ai, payload_pos, payload_R, payload_half_extents)
            min_cp = d if min_cp is None else min(min_cp, d)

    ok = True
    if min_cc is not None and min_cc < d_safe:
        ok = False
    if min_cp is not None and min_cp < d_safe:
        ok = False

    return {
        "ifc_ok": ok,
        "min_cable_cable_dist": min_cc,
        "min_cable_payload_dist": min_cp,
    }


def score_routing_across_poses(routing, ugv_db, pay_layout, gnd_layout, poses,
                                enable_wfc=False, tau_min=5.0, tau_max=100.0,
                                wfc_wrench=None,
                                enable_ifc=False, d_safe=0.05, payload_half_extents=None):

    if wfc_wrench is None:
        wfc_wrench = np.zeros(6)

    per_pose_scores = []
    per_pose_wfc = []
    per_pose_ifc = []

    for payload_pos, payload_R in poses:
        Jp = build_Jp_ground(routing, ugv_db, pay_layout, gnd_layout, payload_pos, payload_R)
        score = score_Jp(Jp)
        if score is None or not score["rank_ok"]:
            return None  # fails WCC

        if enable_wfc:
            wfc = check_wfc(Jp, tau_min, tau_max, wfc_wrench)
            if not wfc["wfc_ok"]:
                return None  # fails WFC
            per_pose_wfc.append(wfc)

        if enable_ifc:
            ifc = check_ifc(routing, ugv_db, pay_layout, gnd_layout, payload_pos, payload_R,
                             d_safe, payload_half_extents)
            if not ifc["ifc_ok"]:
                return None  # fails IFC
            per_pose_ifc.append(ifc)

        per_pose_scores.append(score)

    result = {
        "conditioning_index": min(s["conditioning_index"] for s in per_pose_scores),
        "manipulability": min(s["manipulability"] for s in per_pose_scores),
        "sigma_min": min(s["sigma_min"] for s in per_pose_scores),
        "sigma_max": max(s["sigma_max"] for s in per_pose_scores),
        "rank_ok": True,
        "n_poses_checked": len(per_pose_scores),
    }

    if enable_wfc:
        result["wfc_ok"] = True  # only reaches here if every pose passed
    if enable_ifc:
        cc_vals = [d["min_cable_cable_dist"] for d in per_pose_ifc if d["min_cable_cable_dist"] is not None]
        cp_vals = [d["min_cable_payload_dist"] for d in per_pose_ifc if d["min_cable_payload_dist"] is not None]
        result["ifc_ok"] = True
        result["min_cable_cable_dist"] = min(cc_vals) if cc_vals else None
        result["min_cable_payload_dist"] = min(cp_vals) if cp_vals else None

    return result



def screen_architecture(ugv_db, pay_layout, gnd_layout, poses, max_enumerate=20000, seed=0,
                         enable_wfc=False, tau_min=5.0, tau_max=100.0, wfc_wrench=None,
                         enable_ifc=False, d_safe=0.05, payload_half_extents=None):
    upper_bound = count_routings_upper_bound(ugv_db, pay_layout, gnd_layout)
    if upper_bound == 0:
        return None

    if upper_bound <= max_enumerate:
        routings = enumerate_routings(ugv_db, pay_layout, gnd_layout)
    else:
        routings = enumerate_routings(ugv_db, pay_layout, gnd_layout, max_results=max_enumerate)

    best = None
    for routing in routings:
        gnd_nodes_used = [g for (p, g) in routing]
        from collections import Counter
        gnd_counts = Counter(gnd_nodes_used)

        score = score_routing_across_poses(
            routing, ugv_db, pay_layout, gnd_layout, poses,
            enable_wfc=enable_wfc, tau_min=tau_min, tau_max=tau_max, wfc_wrench=wfc_wrench,
            enable_ifc=enable_ifc, d_safe=d_safe, payload_half_extents=payload_half_extents,
        )
        if score is None:
            continue
        if best is None or score["manipulability"] > best["manipulability"]:
            best = {**score, "routing": routing}

    if best is None:
        return {
            "pay_layout": pay_layout, "gnd_layout": gnd_layout,
            "n_routings_checked": len(routings), "n_poses_checked": len(poses), "feasible": False,
        }

    return {
        "pay_layout": pay_layout, "gnd_layout": gnd_layout,
        "n_routings_checked": len(routings), "feasible": True,
        **{k: v for k, v in best.items() if k != "routing"},
        "best_routing": "|".join(f"{p}-{g}" for p, g in best["routing"]),
    }


def screen_all(poses, pay_layouts=None, gnd_layouts=None, max_enumerate=20000, seed=0,
                enable_wfc=False, tau_min=5.0, tau_max=100.0, wfc_wrench=None,
                enable_ifc=False, d_safe=0.05, payload_half_extents=None):
    ugv_db = _load_json_db(UGV_DB_PATH)
    ugv_db.pop("rectangle-same", None)

    pay_layouts = pay_layouts or list(GRID_MAPPING_UGV.keys())
    gnd_layouts = gnd_layouts or list(GRID_MAPPING_UGV.keys())

    if "rectangle-same" in pay_layouts:
        pay_layouts.remove("rectangle-same")
    if "rectangle-same" in gnd_layouts:
        gnd_layouts.remove("rectangle-same")

    if enable_ifc and payload_half_extents is None:
        print("--enable-ifc was set but no --payload-half-extents given: "
              "only cable-cable clearance will be checked, NOT cable-payload clearance.")

    rows = []
    total = len(pay_layouts) * len(gnd_layouts)
    for i, (pay_layout, gnd_layout) in enumerate(itertools.product(pay_layouts, gnd_layouts), 1):
        print(f"[{i}/{total}] {pay_layout} / {gnd_layout} ...", end=" ")
        result = screen_architecture(
            ugv_db, pay_layout, gnd_layout, poses, max_enumerate=max_enumerate, seed=seed,
            enable_wfc=enable_wfc, tau_min=tau_min, tau_max=tau_max, wfc_wrench=wfc_wrench,
            enable_ifc=enable_ifc, d_safe=d_safe, payload_half_extents=payload_half_extents,
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
                     help="Enable Wrench Feasible Condition check (adds one LP solve per routing per pose)")
    ap.add_argument("--tau-min", type=float, default=TAU_MIN,
                     help=f"minimum admissible cable tension [N], from config_params.TAU_MIN (default {TAU_MIN})")
    ap.add_argument("--tau-max", type=float, default=TAU_MAX,
                     help=f"maximum admissible cable tension [N], from config_params.TAU_MAX (default {TAU_MAX} - "
                          f"still a placeholder, see config_params.py)")
    ap.add_argument("--wfc-wrench", type=float, nargs=6, default=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                     metavar=("FX", "FY", "FZ", "MX", "MY", "MZ"),
                     help="target ground-cable wrench W_target for the WFC check - "
                          "REPLACE with what your tension planner actually asks the ground cables for (Eq. 2.35)")

    # --- IFC (Eq. 2.20) ---
    ap.add_argument("--enable-ifc", action="store_true",
                     help="Enable Interference-Free Condition check (cable-cable always; "
                          "cable-payload only if --payload-half-extents is given)")
    ap.add_argument("--d-safe", type=float, default=D_SAFE,
                     help=f"minimum clearance [m] between cables and between cables/payload, "
                          f"from config_params.D_SAFE (default {D_SAFE})")
    ap.add_argument("--payload-half-extents", type=float, nargs=3, default=PAYLOAD_HALF_EXTENTS,
                     metavar=("HX", "HY", "HZ"),
                     help="payload modeled as an oriented box with these half-extents [m], "
                          "centered/oriented at each pose. Defaults to config_params.PAYLOAD_HALF_EXTENTS "
                          "(currently None - fill that in once known)")

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

    payload_half_extents = (np.array(args.payload_half_extents, dtype=float)
                             if args.payload_half_extents is not None else None)
    wfc_wrench = np.array(args.wfc_wrench, dtype=float)

    df = screen_all(
        poses, max_enumerate=args.max_enumerate, seed=args.seed,
        enable_wfc=args.enable_wfc, tau_min=args.tau_min, tau_max=args.tau_max, wfc_wrench=wfc_wrench,
        enable_ifc=args.enable_ifc, d_safe=args.d_safe, payload_half_extents=payload_half_extents,
    )

    if df.empty:
        print("\nNo architectures found during screening.")
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
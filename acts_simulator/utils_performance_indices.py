import numpy as np
from itertools import combinations
from scipy.spatial import ConvexHull
from utils_optimization import compute_payload_jacobian
import os
import pandas as pd


def compute_aws_vertices(Jp_t: np.ndarray, tau_min: np.ndarray, tau_max: np.ndarray) -> np.ndarray:
    m = Jp_t.shape[0]
    corners = np.array(np.meshgrid(*[[tau_min[i], tau_max[i]] for i in range(m)],
                                    indexing='ij')).reshape(m, -1).T
    return corners @ Jp_t 


def capacity_margin(Jp_t: np.ndarray, tau_min: np.ndarray, tau_max: np.ndarray,
                     task_wrenches: np.ndarray) -> float:
    vertices = compute_aws_vertices(Jp_t, tau_min, tau_max)
    try:
        hull = ConvexHull(vertices)
    except Exception as e:
        raise RuntimeError(f"ConvexHull failed (degenerate AWS -- check rank(Jp)): {e}")

    a = hull.equations[:, :6]
    b = -hull.equations[:, 6]

    gamma = np.inf
    for w_e in task_wrenches:
        for l in range(len(b)):
            al, bl = a[l], b[l]
            gamma = min(gamma, (bl - w_e @ al) / (np.linalg.norm(al) ** 2))
    return gamma


def worst_case_capacity_margin(tau_optimal: np.ndarray, tau_max: np.ndarray) -> float:
    return float(np.min(1.0 - tau_optimal / tau_max))


def conditioning_index(Jp_t: np.ndarray) -> float:
    sv = np.linalg.svd(Jp_t, compute_uv=False)
    return float(sv.min() / sv.max()) if sv.max() > 1e-12 else 0.0


def manipulability(Jp_t: np.ndarray) -> float:
    return float(np.sqrt(max(np.linalg.det(Jp_t.T @ Jp_t), 0.0)))


def radius_available_wrench(cable_unit_vecs: np.ndarray, tau_min: np.ndarray, tau_max: np.ndarray,
                             m_payload: float, g: float = 9.81) -> float:
    m = cable_unit_vecs.shape[0]
    g_vec = np.array([0.0, 0.0, g])
    rAW = np.inf

    for idx in combinations(range(m), 3):
        idx = list(idx)
        M = cable_unit_vecs[idx].T
        if abs(np.linalg.det(M)) < 1e-10:
            continue

        _, s, Vt = np.linalg.svd(M.T)
        n_null = Vt[-1]

        remaining = [j for j in range(m) if j not in idx]
        all_idx = idx + remaining

        I_plus = [j for j in all_idx if n_null @ cable_unit_vecs[j] >= 0]
        I_minus = [j for j in all_idx if n_null @ cable_unit_vecs[j] < 0]

        dp = (sum(tau_max[j] * (n_null @ cable_unit_vecs[j]) for j in I_plus)
              + sum(tau_min[j] * (n_null @ cable_unit_vecs[j]) for j in I_minus)
              - m_payload * abs(n_null @ g_vec))

        dq = (-sum(tau_max[j] * (n_null @ cable_unit_vecs[j]) for j in I_minus)
              - sum(tau_min[j] * (n_null @ cable_unit_vecs[j]) for j in I_plus)
              - m_payload * abs(n_null @ g_vec))

        norm_null = np.linalg.norm(n_null)
        if norm_null < 1e-12:
            continue

        rAW = min(rAW, min(abs(dp), abs(dq)) / norm_null)

    return rAW


def composite_score(gamma, rAW, zeta, w1=1/3, w2=1/3, w3=1/3,
                     gamma_ref=1.0, rAW_ref=1.0, zeta_ref=1.0) -> float:
    return w1 * (gamma / gamma_ref) + w2 * (rAW / rAW_ref) + w3 * (zeta / zeta_ref)

def pose_error(p_payload: np.ndarray, R_mat_payload: np.ndarray,
               p_star: np.ndarray, R_star: np.ndarray) -> tuple[float, float]:
    """
    Returns (position_error [m], orientation_error [rad]).
    Orientation error is the geodesic angle on SO(3): the same rotation
    magnitude used inside the geometric controller's e_R, just converted
    to a scalar angle instead of a body-frame vector.
    """
    pos_err = np.linalg.norm(p_payload - p_star)

    R_rel = R_star.T @ R_mat_payload
    cos_angle = (np.trace(R_rel) - 1.0) / 2.0
    cos_angle = np.clip(cos_angle, -1.0, 1.0)  # guard against numerical drift
    ang_err = np.arccos(cos_angle)

    return float(pos_err), float(ang_err)


def pose_reached(p_payload: np.ndarray, R_mat_payload: np.ndarray,
                  p_star: np.ndarray, R_star: np.ndarray,
                  pos_tol: float = 1e-1, rot_tol: float = np.deg2rad(3.0)
                  ) -> dict:
    """
    pos_tol: meters, rot_tol: radians.
    Returns a dict with individual and combined acceptability flags.
    """
    pos_err, ang_err = pose_error(p_payload, R_mat_payload, p_star, R_star)
    pos_ok = pos_err <= pos_tol
    orient_ok = ang_err <= rot_tol

    return {
        "position_error": pos_err,
        "orientation_error": ang_err,
        "position_reached": pos_ok,
        "orientation_reached": orient_ok,
        "pose_reached": pos_ok and orient_ok,
    }

def compute_rig_performance_indices(p_payload, R_mat_payload,
                                     p_drone_targets, P_GROUND_ANCHORS,
                                     HOOK_OFFSETS_DRONE, HOOK_OFFSETS_GROUND,
                                     optimal_tensions, tau_ground, W_p_star, PAYLOAD_MASS,
                                     tau_min_drone=5.5, tau_max_drone=40.0,
                                     w_min_ground=5.0, w_max_ground=40.0,
                                     g=9.81):
    
    n_a = len(p_drone_targets)
    n_g = len(P_GROUND_ANCHORS)

    all_anchors = list(p_drone_targets) + list(P_GROUND_ANCHORS)
    all_offsets = list(HOOK_OFFSETS_DRONE) + list(HOOK_OFFSETS_GROUND)

    Jp = compute_payload_jacobian(p_payload, R_mat_payload, all_anchors, all_offsets)
    Jp_t = Jp.T 

    cable_unit_vecs = np.zeros((n_a + n_g, 3))
    for i in range(n_a + n_g):
        b_i_global = R_mat_payload @ all_offsets[i]
        r_i = p_payload + b_i_global
        l_vec = all_anchors[i] - r_i
        cable_unit_vecs[i] = l_vec / np.linalg.norm(l_vec)

    tau_min = np.concatenate([np.full(n_a, tau_min_drone), np.full(n_g, w_min_ground)])
    tau_max = np.concatenate([np.full(n_a, tau_max_drone), np.full(n_g, w_max_ground)])

    tau_optimal = np.concatenate([np.asarray(optimal_tensions), tau_ground])

    nu = conditioning_index(Jp_t)
    w = manipulability(Jp_t)
    rAW = radius_available_wrench(cable_unit_vecs, tau_min, tau_max, PAYLOAD_MASS, g=g)
    zeta = worst_case_capacity_margin(tau_optimal, tau_max)

    try:
        gamma = capacity_margin(Jp_t, tau_min, tau_max, task_wrenches=np.array([W_p_star]))
    except RuntimeError:
        gamma = np.nan

    N = composite_score(gamma, rAW, zeta) if np.isfinite(gamma) else np.nan


    return {
        "conditioning_index": nu,
        "manipulability": w,
        "radius_available_wrench": rAW,
        "capacity_margin": gamma,
        "worst_case_capacity_margin": zeta,
        "composite_score": N,
    }

def append_robot_data(filename, config, pos, quat, indices, pose_params):

    print(pose_params)
    print(pose_params["position_error"])

    new_row = {
        "config" : config,
        "pos_x": pos[0],
        "pos_y": pos[1],
        "pos_z": pos[2],
        "quat_w": quat[0],
        "quat_x": quat[1],
        "quat_y": quat[2],
        "quat_z": quat[3],
        "conditioning_index": indices["conditioning_index"],
        "manipulability": indices["manipulability"],
        "radius_available_wrench": indices["radius_available_wrench"],
        "capacity_margin": indices["capacity_margin"],
        "worst_case_capacity_margin": indices["worst_case_capacity_margin"],
        "composite_score": indices["composite_score"],
        "position_error" : pose_params["position_error"],
        "orientation_error": pose_params["orientation_error"],
        "position_reached": pose_params["position_reached"],
        "orientation_reached": pose_params["orientation_reached"],
        "pose_reached": pose_params["pose_reached"],
    }

    df_new = pd.DataFrame([new_row])

    if not os.path.exists(filename):
        df_new.to_excel(filename, index=False)
    else:
        with pd.ExcelWriter(
            filename, mode="a", engine="openpyxl", if_sheet_exists="overlay"
        ) as writer:
            # Find the next available row in the active sheet
            start_row = writer.sheets["Sheet1"].max_row
            # Write data without headers
            df_new.to_excel(
                writer, index=False, header=False, startrow=start_row
            )

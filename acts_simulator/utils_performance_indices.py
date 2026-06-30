import numpy as np
from itertools import combinations
from scipy.spatial import ConvexHull
from utils_optimization import compute_payload_jacobian


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


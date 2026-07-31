import numpy as np
from scipy.optimize import minimize, lsq_linear
import itertools
import os
import sys
_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PROJECT_ROOT not in sys.path:
    sys.path.append(_PROJECT_ROOT)
from acts_simulator import D_SAFE_DRONE, D_SAFE_CABLE, TAU_MIN, TAU_MAX, MODE, THRUST_MAX, THRUST_MIN

# -----------------------------------------------------------------------
# Geometric helpers for the Interference-Free Condition (Eq. 2.19)
# -----------------------------------------------------------------------
def segment_segment_distance(p1, p2, p3, p4):
    """
    Minimum distance between segment [p1, p2] and segment [p3, p4] in 3D.
    Standard closest-point-between-two-segments algorithm
    (see e.g. Ericson, "Real-Time Collision Detection", Ch. 5.1.9).
    This is what d_ij = dist(cable_i, cable_j) actually means in Eq. 2.19 -
    NOT the distance between the segment endpoints.
    """
    d1 = p2 - p1
    d2 = p4 - p3
    r = p1 - p3

    a = np.dot(d1, d1)  # |d1|^2
    e = np.dot(d2, d2)  # |d2|^2
    f = np.dot(d2, r)

    eps = 1e-12

    if a <= eps and e <= eps:
        # Both segments degenerate to points
        return np.linalg.norm(p1 - p3)

    if a <= eps:
        s = 0.0
        t = np.clip(f / e, 0.0, 1.0)
    else:
        c = np.dot(d1, r)
        if e <= eps:
            t = 0.0
            s = np.clip(-c / a, 0.0, 1.0)
        else:
            b = np.dot(d1, d2)
            denom = a * e - b * b
            if denom > eps:
                s = np.clip((b * f - c * e) / denom, 0.0, 1.0)
            else:
                s = 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t = 0.0
                s = np.clip(-c / a, 0.0, 1.0)
            elif t > 1.0:
                t = 1.0
                s = np.clip((b - c) / a, 0.0, 1.0)

    c1 = p1 + s * d1
    c2 = p3 + t * d2
    return np.linalg.norm(c1 - c2)


def point_segment_distance(p, a, b):
    """Minimum distance between point p and segment [a, b]."""
    d = b - a
    denom = np.dot(d, d)
    if denom <= 1e-12:
        return np.linalg.norm(p - a)
    t = np.clip(np.dot(p - a, d) / denom, 0.0, 1.0)
    closest = a + t * d
    return np.linalg.norm(p - closest)


def compute_payload_jacobian_transpose(p_payload, R_mat_payload, p_anchors, hook_offsets):
    m = len(p_anchors)
    J_p = np.zeros((6, m))

    for i in range(m):
        b_i_global = R_mat_payload @ hook_offsets[i]
        r_i = p_payload + b_i_global

        l_vector = p_anchors[i] - r_i
        norm = np.linalg.norm(l_vector)

        if norm < 1e-9:
            u_i = np.array([0.0, 0.0, 1.0])
        else:
            u_i = l_vector / norm

        J_p[0:3, i] = u_i
        J_p[3:6, i] = np.cross(b_i_global, u_i)

    return J_p


def optimize_drone_positions(p_payload, R_mat_payload, p_ground_anchors,
                              drone_masses, l_cables_drone, hook_offsets_drone,
                              hook_offsets_ground, W_p_star,
                              tau_min=TAU_MIN, tau_max=TAU_MAX, d_safe=D_SAFE_DRONE, g=9.81,
                              x0_warm=None):

    n_a = len(drone_masses)
    n_g = len(p_ground_anchors)

    p_hooks_drone = [p_payload + R_mat_payload @ hook_offsets_drone[i] for i in range(n_a)]

    if x0_warm is not None:
        x0 = np.asarray(x0_warm).flatten()
    else:
        x0 = []
        for i in range(n_a):
            init_pos = p_hooks_drone[i] + np.array([0.0, 0.0, l_cables_drone[i]])
            x0.extend(init_pos)
        x0 = np.array(x0)

    # ---------------------------------------------------------------
    # [A] / [C] Cache: x_flat.tobytes() -> (tau, p_a, J_p)
    # Populated once per distinct point, reused by objective() and by
    # every constraint function called at that same point.
    # ---------------------------------------------------------------
    _cache = {}

    def evaluate_tensions_for_layout(p_a_flat):
        key = p_a_flat.tobytes()
        hit = _cache.get(key)
        if hit is not None:
            return hit[0], hit[1]

        p_a = p_a_flat.reshape((n_a, 3))

        # Combine drone positions and ground anchors to get full anchor set
        all_anchors = list(p_a) + list(p_ground_anchors)
        all_offsets = list(hook_offsets_drone) + list(hook_offsets_ground)

        # Compute the system's geometric allocation matrix (once, cached)
        J_p = compute_payload_jacobian_transpose(p_payload, R_mat_payload, all_anchors, all_offsets)

        if MODE == "tau_min":
            tau = np.full(n_a + n_g, tau_min)
        else:
            # Solve for tensions: J_p @ tau = W_p_star -> bounded least squares
            bounds = (
                [THRUST_MIN] * n_a + [tau_min] * n_g,
                [THRUST_MAX] * n_a + [tau_max] * n_g
            )
            res_tau = lsq_linear(J_p, W_p_star, bounds=bounds, method='bvls')
            tau = res_tau.x

        _cache[key] = (tau, p_a, J_p)
        return tau, p_a

    def get_cached_Jp(p_a_flat):
        # populates the cache if needed, then returns J_p from it
        evaluate_tensions_for_layout(p_a_flat)
        return _cache[p_a_flat.tobytes()][2]

    # Objective Function: Minimize sum(||F_prop||^2) + wrench tracking penalty
    def objective(x_flat):
        tau, p_a = evaluate_tensions_for_layout(x_flat)
        J_p = get_cached_Jp(x_flat)

        # Calculate Drone Propulsion Cost
        drone_cost = 0.0
        for i in range(n_a):
            r_i = p_payload + R_mat_payload @ hook_offsets_drone[i]
            u_i = (p_a[i] - r_i) / l_cables_drone[i]
            F_prop = drone_masses[i] * np.array([0.0, 0.0, g]) + tau[i] * u_i
            drone_cost += np.sum(F_prop ** 2)

        # Payload Wrench generated with the current tensions
        W_p_generated = J_p @ tau

        # Penalize the difference between generated wrench and desired tracking wrench
        wrench_mismatch_cost = 100.0 * np.sum((W_p_generated - W_p_star) ** 2)

        return drone_cost + wrench_mismatch_cost

    constraints = []

    # -----------------------------------------------------------------
    # Constraint 1 [D]: Cable Length Equalities for Drones (vectorized)
    # dist_sq_i - l_i^2 == 0 for every drone i.
    # [B] Analytic jacobian: purely geometric in p_a, no dependence on tau.
    # -----------------------------------------------------------------
    def cable_length_constraint(x_flat):
        _, p_a = evaluate_tensions_for_layout(x_flat)
        out = np.empty(n_a)
        for i in range(n_a):
            r_i = p_payload + R_mat_payload @ hook_offsets_drone[i]
            out[i] = np.sum((p_a[i] - r_i) ** 2) - l_cables_drone[i] ** 2
        return out

    def cable_length_jac(x_flat):
        _, p_a = evaluate_tensions_for_layout(x_flat)
        J = np.zeros((n_a, 3 * n_a))
        for i in range(n_a):
            r_i = p_payload + R_mat_payload @ hook_offsets_drone[i]
            J[i, 3 * i:3 * i + 3] = 2.0 * (p_a[i] - r_i)
        return J

    constraints.append({'type': 'eq', 'fun': cable_length_constraint, 'jac': cable_length_jac})

    # -----------------------------------------------------------------
    # Constraint 2: tau_min^2 - w^2 <= 0  -> Ground Tensions >= tau_min
    # (vectorized; left on numerical differencing since it depends on tau,
    #  which comes out of the bvls solve and has no convenient closed form)
    # -----------------------------------------------------------------
    if MODE == "tau_optimal":
        def ground_tension_constraint(x_flat):
            tau, _ = evaluate_tensions_for_layout(x_flat)
            return tau[n_a:n_a + n_g] - tau_min  # each must be >= 0

        constraints.append({'type': 'ineq', 'fun': ground_tension_constraint})

    # -----------------------------------------------------------------
    # Constraint 3 [D]: Inter-drone Collision Avoidance (d_ij >= d_safe), vectorized
    # [B] Analytic jacobian: purely geometric in p_a.
    # -----------------------------------------------------------------
    drone_pairs = list(itertools.combinations(range(n_a), 2))

    def drone_collision_constraint(x_flat):
        _, p_a = evaluate_tensions_for_layout(x_flat)
        out = np.empty(len(drone_pairs))
        for k, (i, j) in enumerate(drone_pairs):
            out[k] = np.linalg.norm(p_a[i] - p_a[j]) - d_safe
        return out

    def drone_collision_jac(x_flat):
        _, p_a = evaluate_tensions_for_layout(x_flat)
        J = np.zeros((len(drone_pairs), 3 * n_a))
        for k, (i, j) in enumerate(drone_pairs):
            diff = p_a[i] - p_a[j]
            dist = np.linalg.norm(diff)
            if dist <= 1e-12:
                continue  # gradient undefined at coincident points; leave as zero row
            grad = diff / dist
            J[k, 3 * i:3 * i + 3] = grad
            J[k, 3 * j:3 * j + 3] = -grad
        return J

    if drone_pairs:
        constraints.append({'type': 'ineq', 'fun': drone_collision_constraint, 'jac': drone_collision_jac})

    # -----------------------------------------------------------------
    # Constraint 4 [D]: Payload Collision Avoidance (d_i >= d_safe), vectorized
    # [B] Analytic jacobian: purely geometric in p_a.
    # -----------------------------------------------------------------
    def payload_collision_constraint(x_flat):
        _, p_a = evaluate_tensions_for_layout(x_flat)
        out = np.empty(n_a)
        for i in range(n_a):
            out[i] = np.linalg.norm(p_a[i] - p_payload) - d_safe
        return out

    def payload_collision_jac(x_flat):
        _, p_a = evaluate_tensions_for_layout(x_flat)
        J = np.zeros((n_a, 3 * n_a))
        for i in range(n_a):
            diff = p_a[i] - p_payload
            dist = np.linalg.norm(diff)
            if dist <= 1e-12:
                continue
            J[i, 3 * i:3 * i + 3] = diff / dist
        return J

    constraints.append({'type': 'ineq', 'fun': payload_collision_constraint, 'jac': payload_collision_jac})

    res = minimize(objective, x0, method='SLSQP', constraints=constraints,
                    options={'maxiter': 500})  # default is often 100
    print(f"SLSQP success={res.success}, message={res.message}")

    optimized_drones = res.x.reshape((n_a, 3))
    opt_tau, _ = evaluate_tensions_for_layout(res.x)
    return optimized_drones, opt_tau[:n_a]


def check_ground_cable_rubbing(p_payload, R_mat_payload, p_ground_anchors,
                                hook_offsets_ground, d_safe=D_SAFE_CABLE):
    """
    Interference-Free Condition (Eq. 2.19) restricted to the n_g ground
    cables against each other: d_ij = dist(cable_i, cable_j) >= d_safe.

    Both endpoints of every ground cable are fixed given the current
    payload pose (anchor is fixed in the world; hook only depends on
    p_payload / R_mat_payload) - this does NOT depend on the drone
    positions being optimized in optimize_drone_positions, so it is kept
    as a separate, independent check rather than an SLSQP constraint.

    Call once per control iteration (or offline per routing candidate)
    right after you compute p_payload / R_mat_payload.

    Returns
    -------
    min_distance : float
        Smallest pairwise segment-to-segment distance found.
    ok : bool
        True if all pairs respect the clearance (min_distance >= d_safe).
    pair_distances : dict[(i, j) -> float]
        Distance for every one of the n_g*(n_g-1)/2 ground-cable pairs,
        indexed by ground-cable index (0-based, matching the order of
        p_ground_anchors / hook_offsets_ground).
    """
    n_g = len(p_ground_anchors)
    p_hooks_ground = [p_payload + R_mat_payload @ hook_offsets_ground[k] for k in range(n_g)]

    pair_distances = {}
    min_distance = np.inf

    for i, j in itertools.combinations(range(n_g), 2):
        if (p_hooks_ground[i] == p_hooks_ground[j]).all():
            continue
        if (p_ground_anchors[i] == p_ground_anchors[j]).all():
            continue
        d = segment_segment_distance(
            p_hooks_ground[i], p_ground_anchors[i],
            p_hooks_ground[j], p_ground_anchors[j],
        )
        pair_distances[(i, j)] = d
        if d < min_distance:
            min_distance = d

    ok = min_distance >= d_safe
    return min_distance, ok, pair_distances
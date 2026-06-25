import numpy as np
from scipy.optimize import minimize
from utils_control import kt, kd, MAX_ROTOR_VELOCITY

def compute_payload_jacobian(p_payload, R_mat_payload, p_anchors, hook_offsets):

    m = len(p_anchors)
    J_p = np.zeros((6, m))
    
    for i in range(m):
        b_i_global = R_mat_payload @ hook_offsets[i]
        r_i = p_payload + b_i_global
        
        l_vector = p_anchors[i] - r_i
        u_i = l_vector / np.linalg.norm(l_vector)

        J_p[0:3, i] = u_i
        J_p[3:6, i] = np.cross(b_i_global, u_i)
        
    return J_p

def optimize_drone_positions(p_payload, R_mat_payload, p_ground_anchors, 
                             drone_masses, l_cables_drone, hook_offsets_drone, 
                             hook_offsets_ground, W_p_star, w_min=5.0, d_safe=0.5, g=9.81):
    """
    Optimizes drone positions p_a to minimize propulsion force squaring while
    ensuring ground cable tensions satisfy the minimum constraint w_min.
    """
    n_a = len(drone_masses)
    n_g = len(p_ground_anchors)
    
    p_hooks_drone = [p_payload + R_mat_payload @ hook_offsets_drone[i] for i in range(n_a)]
    
    x0 = []
    for i in range(n_a):
        init_pos = p_hooks_drone[i] + np.array([0.0, 0.0, l_cables_drone[i]])
        x0.extend(init_pos)
    x0 = np.array(x0)

    def evaluate_tensions_for_layout(p_a_flat):
        p_a = p_a_flat.reshape((n_a, 3))
        
        # Combine drone positions and ground anchors to get full anchor set
        all_anchors = list(p_a) + list(p_ground_anchors)
        all_offsets = list(hook_offsets_drone) + list(hook_offsets_ground)
        
        # Compute the system's geometric allocation matrix
        J_p = compute_payload_jacobian(p_payload, R_mat_payload, all_anchors, all_offsets)
        
        # Solve for tensions: J_p @ tau = W_p_star -> Least squares or quadratic programming
        # Using a bounded least-squares approach to handle the physics mapping
        from scipy.optimize import lsq_linear
        # Set bounds: Drones have thrust limits, Ground cables must be taut (> 0)
        max_thrust = 4 * kt * MAX_ROTOR_VELOCITY**2  # ≈ 44 N
        bounds = (
            [1.0] * n_a + [w_min] * n_g,
            [max_thrust] * n_a + [150.0] * n_g
        )
        
        res_tau = lsq_linear(J_p, W_p_star, bounds=bounds, method='bvls')
        return res_tau.x, p_a

    # Objective Function: Minimize sum(||F_prop||^2)
    def objective(x_flat):
        tau, p_a = evaluate_tensions_for_layout(x_flat)
        total_cost = 0.0
        for i in range(n_a):
            r_i = p_payload + R_mat_payload @ hook_offsets_drone[i]
            u_i = (p_a[i] - r_i) / l_cables_drone[i]
            # F_prop = m*g*e3 + tau_drone * u_i
            F_prop = drone_masses[i] * np.array([0.0, 0.0, g]) + tau[i] * u_i
            total_cost += np.sum(F_prop**2)
        return total_cost

    constraints = []

    # Constraint 1: Cable Length Equalities for Drones
    for i in range(n_a):
        def cable_length_constraint(x_flat, idx=i):
            tau, p_a = evaluate_tensions_for_layout(x_flat)
            r_idx = p_payload + R_mat_payload @ hook_offsets_drone[idx]
            dist_sq = np.sum((p_a[idx] - r_idx)**2)
            return dist_sq - l_cables_drone[idx]**2
        constraints.append({'type': 'eq', 'fun': cable_length_constraint})

    # Constraint 2: w_min^2 - w^2 <= 0  -> Ground Tensions >= w_min
    for j in range(n_g):
        def ground_tension_constraint(x_flat, idx=j):
            tau, _ = evaluate_tensions_for_layout(x_flat)
            w_ground = tau[n_a + idx]
            return w_ground - w_min  # must be >= 0
        constraints.append({'type': 'ineq', 'fun': ground_tension_constraint})

    # Constraint 3: Inter-drone Collision Avoidance (d_ij >= d_safe)
    import itertools
    for i, j in itertools.combinations(range(n_a), 2):
        def drone_collision_constraint(x_flat, idx1=i, idx2=j):
            _, p_a = evaluate_tensions_for_layout(x_flat)
            d_ij = np.linalg.norm(p_a[idx1] - p_a[idx2])
            return d_ij - d_safe
        constraints.append({'type': 'ineq', 'fun': drone_collision_constraint})

    # Constraint 4: Payload Collision Avoidance (d_i >= d_safe)
    for i in range(n_a):
        def payload_collision_constraint(x_flat, idx=i):
            _, p_a = evaluate_tensions_for_layout(x_flat)
            d_i = np.linalg.norm(p_a[idx] - p_payload)
            return d_i - d_safe
        constraints.append({'type': 'ineq', 'fun': payload_collision_constraint})

    res = minimize(objective, x0, method='SLSQP', constraints=constraints)
    
    if res.success:
        optimized_drones = res.x.reshape((n_a, 3))
        opt_tau, _ = evaluate_tensions_for_layout(res.x)
        return optimized_drones, opt_tau[:n_a]
    else:
        return x0.reshape((n_a, 3)), [15.0] * n_a
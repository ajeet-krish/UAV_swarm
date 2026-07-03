import numpy as np


def compute_graph_laplacian(agents, r_comm):
    n = len(agents)
    A_mat = np.zeros((n, n), dtype=np.float64)
    for i in range(n):
        for j in range(n):
            if i != j:
                dist = np.linalg.norm(agents[i].pos - agents[j].pos)
                if dist <= r_comm:
                    A_mat[i, j] = 1.0
    D = np.diag(A_mat.sum(axis=1))
    L = D - A_mat
    evals = np.linalg.eigvalsh(L)
    lambda_2 = evals[1] if n > 1 else 0.0
    return L, lambda_2


def compute_guidance(agent, agents, target, env, params):
    v_cruise = params.get("v_cruise", 4.0)
    k_follow = params.get("k_follow", 2.0)
    k_avoid = params.get("k_avoid", 8.0)
    k_consensus = params.get("k_consensus", 3.0)
    k_damp = params.get("k_damp", 1.0)
    rho0 = params.get("rho0", 3.0)
    follow_dist = params.get("follow_dist", 5.0)

    # Desired velocity
    if agent.is_leader:
        to_target = target - agent.pos
        dist_to_target = np.linalg.norm(to_target)
        if dist_to_target > 1.0:
            desired_vel = (to_target / dist_to_target) * v_cruise
        else:
            desired_vel = np.zeros(3)
    else:
        target_pos = agents[0].pos + agent.d_ij
        pos_error = target_pos - agent.pos
        desired_vel = agents[0].vel + k_follow * pos_error

    # Velocity tracking force
    F_target = k_follow * (desired_vel - agent.vel)

    # Consensus force (formation keeping with neighbors)
    F_consensus = np.zeros(3, dtype=np.float64)
    for other in agents:
        if other.id == agent.id:
            continue
        dist = np.linalg.norm(agent.pos - other.pos)
        if dist < follow_dist:
            offset_error = (agent.pos - other.pos) - (agent.d_ij - other.d_ij)
            F_consensus -= k_consensus * offset_error

    # Obstacle repulsion
    F_obstacle = env.compute_obstacle_forces(agent.pos, k_avoid, rho0)

    # Damping
    F_damping = -k_damp * agent.vel

    return F_target + F_consensus + F_obstacle + F_damping


def compute_formation_error(agents):
    leader_pos = agents[0].pos
    total = 0.0
    for agent in agents:
        if agent.id == 0:
            continue
        expected = leader_pos + agent.d_ij
        error = np.linalg.norm(agent.pos - expected)
        total += error
    return total


def compute_max_control_effort(agents):
    return max(np.linalg.norm(a.recorded_u) for a in agents)

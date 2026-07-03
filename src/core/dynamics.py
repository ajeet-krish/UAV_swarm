import numpy as np
from scipy.linalg import solve_continuous_are


def compute_lqr_gain(m=1.2, b=0.15):
    A = np.array([
        [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, 0.0, -b/m, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, -b/m, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, -b/m],
    ], dtype=np.float64)

    B = np.array([
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [1.0/m, 0.0, 0.0],
        [0.0, 1.0/m, 0.0],
        [0.0, 0.0, 1.0/m],
    ], dtype=np.float64)

    Q = np.diag([10.0, 10.0, 10.0, 1.0, 1.0, 1.0])
    R = 0.1 * np.eye(3)

    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.solve(R, B.T @ P)
    return K


def compute_control(agent, target_state, K, guidance_force):
    x_error = agent.state - target_state
    u_lqr = -K @ x_error
    u_guidance = guidance_force / agent.m
    u_total = u_lqr + u_guidance
    return agent.saturate_u(u_total)

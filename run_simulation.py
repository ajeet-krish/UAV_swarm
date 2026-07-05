#!/usr/bin/env python3
"""
UAV Swarm Simulation - Distributed Consensus with 3D DCM and Spherical Debris Avoidance
Generates frame data for the web viewer.
"""

import numpy as np
from src.core.models import QuadcopterAgent
from src.core.obstacles import Environment, SphereObstacle
from src.core.wind import DrydenGustModel
from src.core.dynamics import compute_lqr_gain, compute_control
from src.core.consensus import (
    compute_graph_laplacian,
    compute_guidance,
    compute_formation_error,
    compute_max_control_effort,
)
from src.viz.export import FrameExporter, sample_vector_field

# Simulation parameters
DT = 0.02
TOTAL_TIME = 90.0
NUM_STEPS = int(TOTAL_TIME / DT)

# Formation definitions - Wedge offsets relative to leader
WEDGE_OFFSETS = [
    [0.0, 0.0, 0.0],       # D0 leader
    [-1.5, 0.0, 1.5],      # D1
    [-1.5, 0.0, -1.5],     # D2
    [-3.0, 0.0, 3.0],      # D3
    [-3.0, 0.0, 1.0],      # D4
    [-3.0, 0.0, -1.0],     # D5
    [-3.0, 0.0, -3.0],     # D6
]

GUIDANCE_PARAMS = {
    "v_cruise": 3.0,
    "k_follow": 2.5,
    "k_avoid": 30.0,       # High avoidance gain to dodge cleanly
    "k_consensus": 4.0,
    "k_damp": 1.2,
    "rho0": 3.5,          # Large activation radius for early avoidance
    "follow_dist": 6.0,
}

R_COMM = 8.0
LEADER_START = np.array([-8.0, 2.0, -8.0], dtype=np.float64)

# 8 Waypoints: figure-8 path with vertical variation, low passes, and sharp turns
WAYPOINTS = [
    np.array([8.0, 9.0, -8.0], dtype=np.float64),    # WP1: High right
    np.array([8.0, 2.0, 8.0], dtype=np.float64),     # WP2: Low opposite corner
    np.array([-2.0, 8.0, 6.0], dtype=np.float64),    # WP3: High center-right gap
    np.array([-8.0, 9.0, 8.0], dtype=np.float64),    # WP4: High left
    np.array([-3.0, 3.0, -3.0], dtype=np.float64),   # WP5: Low center-left through obstacles
    np.array([0.0, 5.0, 0.0], dtype=np.float64),     # WP6: Center pass
    np.array([5.0, 7.0, -5.0], dtype=np.float64),    # WP7: Mid right figure-8 crossover
    np.array([-8.0, 2.0, -8.0], dtype=np.float64),   # WP8: Start (loop back)
]


def build_environment():
    env = Environment()

    # Procedural density field: 9 additional obstacles for 15 total
    # Seeded for reproducibility, then hand-tuned for cinematic narrow passages
    rng = np.random.RandomState(42)
    extra_centers = []
    for _ in range(9):
        for attempt in range(50):
            cx = rng.uniform(-9.0, 9.0)
            cy = rng.uniform(2.0, 10.0)
            cz = rng.uniform(-9.0, 9.0)
            # Keep start zone clear: sphere around LEADER_START
            start_clear = np.linalg.norm(np.array([cx, cy, cz]) - LEADER_START) > 4.0
            # Keep separation between obstacles
            sep_ok = all(
                np.linalg.norm(np.array([cx, cy, cz]) - np.array(c))
                > 3.0 for c in extra_centers
            )
            if start_clear and sep_ok:
                extra_centers.append([cx, cy, cz])
                break

    # Hand-tuned adjustments for narrow passages and cinematic lines of sight
    extra_centers[0] = [3.0, 3.5, 2.0]      # Creates a low gap near WP2 path
    extra_centers[1] = [-6.0, 7.0, -2.5]    # Flanking obstacle for WP1→WP8 corridor
    extra_centers[2] = [0.0, 4.5, -6.0]     # Narrow passage on WP5→WP6 route
    extra_centers[3] = [6.0, 8.5, -2.0]     # WP7 approach obstacle
    extra_centers[4] = [-7.0, 5.0, 5.0]     # WP3→WP4 channel constraint
    extra_centers[5] = [2.0, 9.0, 7.0]      # WP4 return path
    extra_centers[6] = [-2.0, 2.5, 8.0]     # Low obstacle near WP2
    extra_centers[7] = [5.0, 6.0, -7.0]     # WP7 low approach
    extra_centers[8] = [-6.5, 9.5, -7.0]    # WP1 approach obstacle

    radii = [1.5, 1.8, 1.2, 1.6, 1.4, 1.3, 1.0, 1.5, 1.7, 1.1, 1.4, 1.6, 1.2, 1.5, 1.3]

    # 6 original strategic obstacles
    originals = [
        ([4.0, 5.0, 3.0], 1.5, "Asteroid 1"),
        ([-4.0, 4.0, -3.0], 1.8, "Asteroid 2"),
        ([2.0, 8.0, -4.0], 1.2, "Asteroid 3"),
        ([-2.0, 7.0, 4.0], 1.4, "Asteroid 4"),
        ([5.0, 2.0, -5.0], 1.6, "Asteroid 5"),
        ([-5.0, 9.0, 2.0], 1.3, "Asteroid 6"),
    ]

    for i, (center, radius, label) in enumerate(originals):
        env.add_obstacle(SphereObstacle(center=center, radius=radius, label=label))

    # 9 procedural + hand-tuned additions
    for i, center in enumerate(extra_centers):
        r = radii[6 + i] if (6 + i) < len(radii) else 1.2
        env.add_obstacle(SphereObstacle(center=center, radius=r, label=f"Asteroid {7 + i}"))

    return env


def create_agents():
    agents = []
    for i, offset in enumerate(WEDGE_OFFSETS):
        pos = LEADER_START + np.array(offset, dtype=np.float64)
        agent = QuadcopterAgent(
            agent_id=i,
            initial_pos=pos,
            formation_offset=offset,
            is_leader=(i == 0),
        )
        agents.append(agent)
    return agents


def compute_3d_dcm(v_filt):
    """
    Computes the orthogonal Direction Cosine Matrix (DCM) aligned with the velocity vector.
    """
    speed = np.linalg.norm(v_filt)
    if speed < 1e-3:
        return np.eye(3)

    ux = v_filt / speed

    # Reference up vector
    k_ref = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    if abs(np.dot(ux, k_ref)) > 0.99:
        # If climbing/diving almost vertically, switch reference to unit-X
        k_ref = np.array([1.0, 0.0, 0.0], dtype=np.float64)

    # Lateral axis
    uz = np.cross(ux, k_ref)
    uz = uz / (np.linalg.norm(uz) + 1e-12)

    # Vertical axis
    uy = np.cross(uz, ux)
    uy = uy / (np.linalg.norm(uy) + 1e-12)

    # DCM matrix
    R = np.column_stack([ux, uy, uz])
    return R


def build_vector_field_grid():
    grid = []
    # 3D fixed cube zone sampling
    for x in np.arange(-10.0, 10.0, 4.0):
        for y in np.arange(1.0, 11.0, 2.5):
            for z in np.arange(-10.0, 10.0, 4.0):
                grid.append([float(x), float(y), float(z)])
    return grid


def main():
    print("Initializing orbital deep space simulation...")

    env = build_environment()
    agents = create_agents()
    wind_model = DrydenGustModel(sigma=2.0, tau=5.0, seed=42)
    K_lqr = compute_lqr_gain()
    exporter = FrameExporter()
    vector_grid = build_vector_field_grid()

    # Track active waypoint
    current_wp_idx = 0
    target = WAYPOINTS[current_wp_idx]

    # Filtered velocity vector for smooth 3D DCM rotation
    v_filt = np.array([1.0, 0.5, 1.0], dtype=np.float64)

    # Metadata
    exporter.add_meta("dt", DT)
    exporter.add_meta("total_time", TOTAL_TIME)
    exporter.add_meta("num_agents", len(agents))
    exporter.add_meta("obstacles", env.get_obstacle_data())
    exporter.add_meta("wedge_offsets", WEDGE_OFFSETS)
    exporter.add_meta("guidance_params", GUIDANCE_PARAMS)
    exporter.add_meta("r_comm", R_COMM)
    exporter.add_meta("target", target.tolist())
    exporter.add_meta("waypoints", [wp.tolist() for wp in WAYPOINTS])

    print(f"Running {NUM_STEPS} steps (dt={DT}s, {TOTAL_TIME}s total)...")

    for step in range(NUM_STEPS):
        t = step * DT

        # Sample wind
        wind = wind_model.sample(DT)

        # Check waypoint proximity to switch targets
        leader_to_target = target - agents[0].pos
        dist_to_target = np.linalg.norm(leader_to_target)
        if dist_to_target < 2.0:
            current_wp_idx = (current_wp_idx + 1) % len(WAYPOINTS)
            target = WAYPOINTS[current_wp_idx]

        # Smoothly low-pass filter the leader velocity to align 3D DCM
        v_filt = v_filt + 0.05 * (agents[0].vel - v_filt)
        R_dcm = compute_3d_dcm(v_filt)

        # Rotate wedge offsets using the 3D DCM
        for agent in agents:
            # Nominal offset
            d_nom = np.array(WEDGE_OFFSETS[agent.id], dtype=np.float64)
            # Rotate via DCM
            agent.d_ij = R_dcm @ d_nom
            agent.d_ij_target = agent.d_ij

        # Compute controls and step
        for agent in agents:
            guidance = compute_guidance(agent, agents, target, env, GUIDANCE_PARAMS)

            if agent.is_leader:
                # Leader only tracks velocity in LQR, no position error
                target_state = np.concatenate([agent.pos, agent.vel + 0.5 * (target - agent.pos)])
                # Cap the target velocity tracking to v_cruise
                to_t = target - agent.pos
                to_t_norm = np.linalg.norm(to_t)
                if to_t_norm > 0.1:
                    desired_vel = (to_t / to_t_norm) * GUIDANCE_PARAMS["v_cruise"]
                else:
                    desired_vel = np.zeros(3)
                target_state = np.concatenate([agent.pos, desired_vel])
            else:
                target_pos = agents[0].pos + agent.d_ij
                target_state = np.concatenate([target_pos, agents[0].vel])

            u_cmd = compute_control(agent, target_state, K_lqr, guidance)
            agent.wind_local = wind
            agent.recorded_u = u_cmd
            agent.step_rk4(DT, u_cmd)

        # Metrics
        formation_error = compute_formation_error(agents)
        _, lambda_2 = compute_graph_laplacian(agents, R_COMM)
        max_effort = compute_max_control_effort(agents)
        wind_mag = np.linalg.norm(wind)

        metrics = {
            "formation_error": formation_error,
            "lambda_2": lambda_2,
            "max_control_effort": max_effort,
        }

        # Vector field every 10 frames
        vf = None
        if step % 10 == 0:
            vf = sample_vector_field(vector_grid, agents, env.obstacles, env, GUIDANCE_PARAMS)

        exporter.record_frame(t, agents, metrics, wind, agents[0].pos, vector_field=vf)

        if step % 200 == 0:
            print(f"  t={t:.1f}s  WP={current_wp_idx}  error={formation_error:.3f}  lambda2={lambda_2:.3f}  wind={wind_mag:.2f}")

    # Export
    output_path = "docs/assets/data/swarm_simulation.json"
    exporter.save(output_path)
    print(f"\nDone. Exported {len(exporter.frames)} frames to {output_path}")


if __name__ == "__main__":
    main()

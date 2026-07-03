#!/usr/bin/env python3
"""
UAV Swarm Simulation -- Distributed Consensus with APF + LQR
Generates frame data for the Three.js web viewer.
"""

import numpy as np
from src.core.models import QuadcopterAgent
from src.core.obstacles import Environment, BoxObstacle, CylinderObstacle
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
TOTAL_TIME = 18.0
NUM_STEPS = int(TOTAL_TIME / DT)
MORPH_TIME = 8.0
MORPH_DURATION = 2.0

# Formation definitions
WEDGE_OFFSETS = [
    [0.0, 0.0, 0.0],
    [-1.5, 0.0, 1.5],
    [-1.5, 0.0, -1.5],
    [-3.0, 0.0, 3.0],
    [-3.0, 0.0, 1.0],
    [-3.0, 0.0, -1.0],
    [-3.0, 0.0, -3.0],
]

DIAMOND_OFFSETS = [
    [0.0, 1.5, 0.0],
    [-1.5, 0.5, 1.5],
    [-1.5, 0.5, -1.5],
    [-3.0, -0.5, 2.0],
    [-3.0, -0.5, 0.5],
    [-3.0, -0.5, -0.5],
    [-3.0, -0.5, -2.0],
]

GUIDANCE_PARAMS = {
    "v_cruise": 3.0,
    "k_follow": 2.0,
    "k_avoid": 8.0,
    "k_consensus": 3.0,
    "k_damp": 1.0,
    "rho0": 3.0,
    "follow_dist": 6.0,
}

R_COMM = 8.0
LEADER_START = np.array([-6.0, 2.5, 0.0], dtype=np.float64)
TARGET = np.array([20.0, 3.0, 0.0], dtype=np.float64)


def build_environment():
    env = Environment()

    # Left canyon wall
    env.add_obstacle(BoxObstacle(
        bounds=[[-8.0, -1.0, -8.0], [25.0, 10.0, -3.5]],
        label="Left Building",
    ))
    # Right canyon wall
    env.add_obstacle(BoxObstacle(
        bounds=[[-8.0, -1.0, 3.5], [25.0, 10.0, 8.0]],
        label="Right Building",
    ))

    # Pillar obstacles inside corridor
    env.add_obstacle(CylinderObstacle(
        center=[3.0, 2.0, -1.5],
        radius=0.4,
        height=6.0,
        label="Pillar 1",
    ))
    env.add_obstacle(CylinderObstacle(
        center=[7.0, 2.0, 1.5],
        radius=0.4,
        height=6.0,
        label="Pillar 2",
    ))
    env.add_obstacle(CylinderObstacle(
        center=[11.0, 2.0, -0.5],
        radius=0.4,
        height=6.0,
        label="Pillar 3",
    ))

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


def lerp_formation(agents, t):
    if t < MORPH_TIME:
        return
    progress = (t - MORPH_TIME) / MORPH_DURATION
    alpha = min(progress, 1.0)
    for i, agent in enumerate(agents):
        wedge = np.array(WEDGE_OFFSETS[i], dtype=np.float64)
        diamond = np.array(DIAMOND_OFFSETS[i], dtype=np.float64)
        agent.d_ij = (1.0 - alpha) * wedge + alpha * diamond
        agent.d_ij_target = agent.d_ij


def build_vector_field_grid():
    grid = []
    for x in np.arange(-5.0, 18.0, 3.0):
        for y in np.arange(0.5, 6.0, 1.5):
            for z in np.arange(-4.0, 4.0, 2.0):
                grid.append([float(x), float(y), float(z)])
    return grid


def main():
    print("Initializing swarm simulation...")

    env = build_environment()
    agents = create_agents()
    wind_model = DrydenGustModel(sigma=2.0, tau=5.0, seed=42)
    K_lqr = compute_lqr_gain()
    exporter = FrameExporter()
    vector_grid = build_vector_field_grid()

    # Metadata
    exporter.add_meta("dt", DT)
    exporter.add_meta("total_time", TOTAL_TIME)
    exporter.add_meta("num_agents", len(agents))
    exporter.add_meta("morph_time", MORPH_TIME)
    exporter.add_meta("morph_duration", MORPH_DURATION)
    exporter.add_meta("obstacles", env.get_obstacle_data())
    exporter.add_meta("wedge_offsets", WEDGE_OFFSETS)
    exporter.add_meta("diamond_offsets", DIAMOND_OFFSETS)
    exporter.add_meta("guidance_params", GUIDANCE_PARAMS)
    exporter.add_meta("r_comm", R_COMM)
    exporter.add_meta("target", TARGET.tolist())

    print(f"Running {NUM_STEPS} steps (dt={DT}s, {TOTAL_TIME}s total)...")

    for step in range(NUM_STEPS):
        t = step * DT

        # Sample wind
        wind = wind_model.sample(DT)

        # Morph formation
        lerp_formation(agents, t)

        # Compute controls and step
        for agent in agents:
            guidance = compute_guidance(agent, agents, TARGET, env, GUIDANCE_PARAMS)

            if agent.is_leader:
                target_state = np.concatenate([TARGET, np.zeros(3)])
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
            print(f"  t={t:.1f}s  error={formation_error:.3f}  lambda2={lambda_2:.3f}  wind={wind_mag:.2f}")

    # Export
    output_path = "docs/assets/data/swarm_simulation.json"
    exporter.save(output_path)
    print(f"\nDone. Exported {len(exporter.frames)} frames to {output_path}")


if __name__ == "__main__":
    main()

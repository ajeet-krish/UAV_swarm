#!/usr/bin/env python3
"""
2D APF Path Planning Simulation — Corridor navigation with obstacle avoidance.
Generates: optimal run (MP4-ready), gain sweep, Monte Carlo, sensitivity grid.
Velocity-command guidance directly follows APF steering, no LQR.
"""

import json
import numpy as np
import os

# 2D corridor bounds
X_MIN, X_MAX = -15.0, 15.0
Y_MIN, Y_MAX = -5.0, 5.0
START = np.array([-12.0, 0.0], dtype=np.float64)
GOAL_FORWARD = np.array([12.0, 0.0], dtype=np.float64)
GOAL_BACKWARD = np.array([-12.0, 0.0], dtype=np.float64)
WAYPOINTS = [GOAL_FORWARD, GOAL_BACKWARD]

DT = 0.02
V_CRUISE = 3.0
K_ATT = V_CRUISE / 8.0  # attractive velocity gain


def generate_obstacle_layout(seed=42, num=15, density="heavy"):
    rng = np.random.default_rng(seed)
    obstacles = []
    max_attempts = 4000
    if density == "heavy":
        min_clearance = 0.3
        wall_clearance = 0.5
        radius_range = (0.3, 2.2)
    else:
        min_clearance = 1.5
        wall_clearance = 2.0
        radius_range = (0.4, 1.8)
    spawn_zone = 3.5
    attempts = 0
    while len(obstacles) < num and attempts < max_attempts:
        attempts += 1
        r = rng.uniform(*radius_range)
        x = rng.uniform(X_MIN + r + wall_clearance, X_MAX - r - wall_clearance)
        y = rng.uniform(Y_MIN + r + wall_clearance, Y_MAX - r - wall_clearance)
        # Avoid start zone
        if abs(x - START[0]) < spawn_zone and abs(y - START[1]) < spawn_zone:
            continue
        # Avoid goal zone
        if abs(x - GOAL_FORWARD[0]) < spawn_zone and abs(y - GOAL_FORWARD[1]) < spawn_zone:
            continue
        # Check overlap with existing
        ok = True
        for o in obstacles:
            dist = np.linalg.norm(np.array([x, y]) - np.array(o["center"]))
            if dist < r + o["radius"] + min_clearance:
                ok = False
                break
        if ok:
            obstacles.append({"center": [float(x), float(y)], "radius": float(r)})
    return obstacles


def generate_slalom_layout():
    """Hand-designed slalom corridor forcing close-quarters zigzag navigation."""
    return [
        # Gate 1 at x=-7: centered gap
        {"center": [-7.0,  1.75], "radius": 1.0},
        {"center": [-7.0, -1.75], "radius": 1.0},
        # Gate 2 at x=-2: shifted UP
        {"center": [-2.0,  2.75], "radius": 0.8},
        {"center": [-2.0, -0.55], "radius": 1.2},
        # Gate 3 at x=3: shifted DOWN
        {"center": [ 3.0,  0.55], "radius": 1.2},
        {"center": [ 3.0, -2.75], "radius": 0.8},
        # Gate 4 at x=8: centered gap
        {"center": [ 8.0,  1.75], "radius": 1.0},
        {"center": [ 8.0, -1.75], "radius": 1.0},
        # Centerline blockers between gates
        {"center": [-4.5,  0.0], "radius": 0.5},
        {"center": [ 0.5,  0.0], "radius": 0.5},
        {"center": [ 5.5,  0.0], "radius": 0.5},
        # Scattered fillers (visual density)
        {"center": [-3.0,  4.0], "radius": 0.5},
        {"center": [ 1.5,  4.0], "radius": 0.5},
        {"center": [-0.5, -4.0], "radius": 0.5},
        {"center": [ 6.0, -4.0], "radius": 0.5},
    ]


def compute_apf_repulsion(pos, obstacles, k_avoid, rho0, max_perturb=8.0):
    """APF repulsive velocity perturbation (m/s) from obstacles."""
    v = np.zeros(2, dtype=np.float64)
    for obs in obstacles:
        center = np.array(obs["center"])
        r = obs["radius"]
        d = pos - center
        dist = np.linalg.norm(d)
        rho = max(dist - r, 0.05)
        if rho < rho0:
            # Standard APF repulsive gradient (velocity units)
            mag = k_avoid * (1.0 / rho - 1.0 / rho0) / (rho * rho)
            mag = min(mag, max_perturb)
            if dist > 1e-6:
                v += mag * (d / dist)
    return v


def compute_wall_repulsion(pos, k_wall=3.0, wall_margin=2.0, max_perturb=5.0):
    """Wall repulsive velocity perturbation (m/s). Pushes away from each wall."""
    v = np.zeros(2, dtype=np.float64)
    boundaries = [(0, X_MIN, X_MAX), (1, Y_MIN, Y_MAX)]
    for idx, low, high in boundaries:
        p = pos[idx]
        # Low wall: push positive (away from low, toward center)
        dist_to_low = p - low
        if 0 < dist_to_low < wall_margin:
            d = max(dist_to_low, 0.05)
            v[idx] += k_wall * (1.0 / d - 1.0 / wall_margin) / (d * d)
        # High wall: push negative (away from high, toward center)
        dist_to_high = high - p
        if 0 < dist_to_high < wall_margin:
            d = max(dist_to_high, 0.05)
            v[idx] -= k_wall * (1.0 / d - 1.0 / wall_margin) / (d * d)
    v_norm = np.linalg.norm(v)
    if v_norm > max_perturb:
        v = v * (max_perturb / v_norm)
    return v


def simulate_2d(obstacles, k_avoid, rho0, max_steps=1500):
    pos = START.copy()
    wp_idx = 0
    target = WAYPOINTS[wp_idx]

    frames = []
    collision = False
    collision_time = None

    for step in range(max_steps):
        t = step * DT

        to_target = target - pos
        dist_to_target = np.linalg.norm(to_target)

        # Waypoint switching
        if dist_to_target < 0.5:
            wp_idx = (wp_idx + 1) % len(WAYPOINTS)
            target = WAYPOINTS[wp_idx]
            to_target = target - pos
            dist_to_target = np.linalg.norm(to_target)

        # Attractive velocity toward target (proportional to distance)
        if dist_to_target > 0.5:
            v_att = to_target * K_ATT
        else:
            v_att = np.zeros(2)

        # Repulsive velocity perturbations
        v_rep = compute_apf_repulsion(pos, obstacles, k_avoid, rho0)
        v_wall = compute_wall_repulsion(pos)

        # Combine: target attraction + repulsions (loose clamp)
        vel = v_att + v_rep + v_wall
        speed = np.linalg.norm(vel)
        if speed > V_CRUISE * 3:
            vel = vel * (V_CRUISE * 3 / speed)

        # Integrate position and enforce bounds
        pos = pos + vel * DT
        pos = np.clip(pos, [X_MIN + 0.1, Y_MIN + 0.1], [X_MAX - 0.1, Y_MAX - 0.1])

        # Collision check
        for obs in obstacles:
            dist = np.linalg.norm(pos - np.array(obs["center"]))
            if dist < obs["radius"]:
                collision = True
                collision_time = t
                break
        if collision:
            break

        frames.append({
            "t": round(t, 3),
            "pos": [float(pos[0]), float(pos[1])],
            "vel": [float(vel[0]), float(vel[1])],
            "speed": float(speed),
            "F_att": float(np.linalg.norm(v_att)),
            "F_rep": float(np.linalg.norm(v_rep)),
            "F_wall": float(np.linalg.norm(v_wall)),
            "dist_to_goal": float(dist_to_target),
        })

    return frames, collision, collision_time


def run_monte_carlo(num_layouts=20, k_avoid=4, rho0=3.5):
    results = []
    for seed in range(num_layouts):
        obs = generate_obstacle_layout(seed=seed, density="heavy")
        frames, collided, coll_time = simulate_2d(obs, k_avoid, rho0)
        failure_traj = None
        if collided:
            # Store every 10th frame for compression
            failure_traj = [f for i, f in enumerate(frames) if i % 10 == 0]
        results.append({
            "seed": seed,
            "obstacles": obs,
            "success": not collided,
            "collision_time": coll_time,
            "failure_trajectory": failure_traj,
        })
    return results


def run_sensitivity_grid(k_avoid_vals, rho0_vals, obstacles):
    grid = np.zeros((len(k_avoid_vals), len(rho0_vals)), dtype=np.float64)
    for ki, ka in enumerate(k_avoid_vals):
        for ri, r0 in enumerate(rho0_vals):
            frames, collided, _ = simulate_2d(obstacles, ka, r0)
            grid[ki, ri] = 0.0 if collided else 1.0
    return grid.tolist()


def run_mc_sensitivity_grid(num_layouts=20, k_avoid_vals=None, rho0_vals=None):
    """Aggregate failure rate across many layouts for the full sensitivity grid."""
    if k_avoid_vals is None:
        k_avoid_vals = [0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 3.0, 5.0, 10.0]
    if rho0_vals is None:
        rho0_vals = [0.3, 0.5, 0.8, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0]
    nk, nr = len(k_avoid_vals), len(rho0_vals)
    failure_counts = np.zeros((nk, nr), dtype=np.int64)
    layout_difficulty = []
    total_cells = nk * nr

    for seed in range(num_layouts):
        obs = generate_obstacle_layout(seed=seed, density="heavy")
        failures = 0
        for ki, ka in enumerate(k_avoid_vals):
            for ri, r0 in enumerate(rho0_vals):
                _, collided, _ = simulate_2d(obs, ka, r0)
                if collided:
                    failure_counts[ki, ri] += 1
                    failures += 1
        layout_difficulty.append({
            "seed": seed,
            "failure_rate": failures / total_cells,
            "num_obstacles": len(obs),
        })
        print(f"    MC stress: seed={seed}  failures={failures}/{total_cells}  rate={failures/total_cells:.2f}")

    failure_rate_grid = (failure_counts / num_layouts).tolist()
    return {
        "k_avoid_values": k_avoid_vals,
        "rho0_values": rho0_vals,
        "failure_rate_grid": failure_rate_grid,
        "layout_difficulty": layout_difficulty,
    }


def main():
    print("Initializing 2D APF path planning simulation...")
    os.makedirs("docs/assets/data", exist_ok=True)

    # Fixed obstacle layout (hand-designed slalom corridor)
    obstacles = generate_slalom_layout()
    print(f"  Generated {len(obstacles)} obstacles")

    # 1. Optimal run for MP4 (k_avoid=4, rho0=3.5)
    print("  Running optimal trajectory (k_avoid=4, rho0=3.5)...")
    opt_frames, _, _ = simulate_2d(obstacles, k_avoid=4, rho0=3.5)

    # 2. Gain sweep (wider range to show failure at low end)
    print("  Running gain sweep...")
    gain_vals = [0.05, 0.1, 0.2, 0.5, 1, 2, 3, 4, 6, 10]
    gain_sweep = {}
    for ka in gain_vals:
        frames, collided, coll_time = simulate_2d(obstacles, ka, rho0=3.5)
        gain_sweep[str(ka)] = {
            "frames": frames,
            "success": not collided,
            "collision_time": coll_time,
        }

    # 3. Monte Carlo
    print("  Running Monte Carlo (20 layouts)...")
    mc_results = run_monte_carlo(num_layouts=20)

    # 4. Sensitivity grid
    print("  Running sensitivity grid (9x9)...")
    k_av = [0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 3.0, 5.0, 10.0]
    r0_v = [0.3, 0.5, 0.8, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0]
    # Note: these ranges are shared with run_mc_sensitivity_grid defaults
    sens_grid = run_sensitivity_grid(k_av, r0_v, obstacles)

    # 5. MC stress sensitivity (across all layouts)
    print("  Running MC stress sensitivity (20 layouts × 9×9 grid)...")
    mc_sensitivity = run_mc_sensitivity_grid(num_layouts=20)

    # Assemble output
    output = {
        "meta": {
            "dt": DT,
            "total_time": len(opt_frames) * DT,
            "bounds": [X_MIN, X_MAX, Y_MIN, Y_MAX],
            "start": list(START),
            "goal": list(GOAL_FORWARD),
            "obstacles": obstacles,
            "k_avoid_values": gain_vals,
            "num_frames": len(opt_frames),
        },
        "optimal_run": {
            "k_avoid": 4,
            "rho0": 3.5,
            "frames": opt_frames,
        },
        "gain_sweep": gain_sweep,
        "monte_carlo": mc_results,
        "sensitivity": {
            "k_avoid_values": k_av,
            "rho0_values": r0_v,
            "grid": sens_grid,
        },
        "mc_sensitivity": mc_sensitivity,
    }

    output_path = "docs/assets/data/simulation_2d.json"
    with open(output_path, "w") as f:
        json.dump(output, f, indent=2)
    print(f"Done. Exported to {output_path}")
    print(f"  Optimal: {len(opt_frames)} frames")
    print(f"  Gain sweep: {len(gain_sweep)} runs")
    print(f"  Monte Carlo: {len(mc_results)} layouts")
    print(f"  Sensitivity: {len(k_av)}x{len(r0_v)} = {len(k_av)*len(r0_v)} cells")
    mc_k = len(mc_sensitivity["k_avoid_values"])
    mc_r = len(mc_sensitivity["rho0_values"])
    print(f"  MC stress: {len(mc_sensitivity['layout_difficulty'])} layouts x {mc_k}x{mc_r} = {mc_k*mc_r} cells/layout")


if __name__ == "__main__":
    main()

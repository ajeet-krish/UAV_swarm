"""
Generate 2D APF path planning MP4 animation from simulation_2d.json.
Shows corridor, obstacles, APF potential contour, drone trail, HUD.
"""

import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from matplotlib.patches import Circle
from matplotlib.colors import LinearSegmentedColormap
import os

DATA_PATH = "docs/assets/data/simulation_2d.json"
MP4_OUT = "docs/assets/videos/apf_path_2d.mp4"

X_MIN, X_MAX = -15.0, 15.0
Y_MIN, Y_MAX = -5.0, 5.0

# Military green color scheme
GREEN = "#3fb950"
AMBER = "#d29922"
BG = "#0a0e14"
GRID_COLOR = "#21262d"
TEXT_COLOR = "#8b949e"
WHITE = "#c9d1d9"

# Custom colormap for APF potential (transparent amber -> red)
apf_colors = [(0, 0, 0, 0), (0.82, 0.60, 0.13, 0.15), (0.89, 0.28, 0.20, 0.30)]
APF_CMAP = LinearSegmentedColormap.from_list("apf", apf_colors, N=256)


def load_data(path):
    with open(path) as f:
        return json.load(f)


def compute_apf_potential_grid(obstacles, k_avoid, rho0, nx=80, ny=40):
    xs = np.linspace(X_MIN, X_MAX, nx)
    ys = np.linspace(Y_MIN, Y_MAX, ny)
    Xg, Yg = np.meshgrid(xs, ys)
    P = np.zeros_like(Xg)
    for obs in obstacles:
        cx, cy = obs["center"]
        r = obs["radius"]
        dx = Xg - cx
        dy = Yg - cy
        dist = np.sqrt(dx ** 2 + dy ** 2)
        rho = dist - r
        mask = (rho > 0) & (rho < rho0)
        U = np.zeros_like(rho)
        U[mask] = 0.5 * k_avoid * (1.0 / rho[mask] - 1.0 / rho0) ** 2
        P += U
    # Normalize for visual scaling
    pmax = P.max()
    if pmax > 0:
        P = np.clip(P / pmax, 0, 1)
    return Xg, Yg, P


def build_plot(ax, obstacles, Xg, Yg, Pot):
    ax.set_facecolor(BG)
    fig = ax.figure
    fig.patch.set_facecolor("#0d1117")

    # Contour fill
    ax.contourf(Xg, Yg, Pot, levels=20, cmap=APF_CMAP, alpha=0.6, zorder=1)

    # Corridor walls
    ax.axhline(Y_MIN, color=AMBER, linestyle="--", linewidth=1.0, alpha=0.5)
    ax.axhline(Y_MAX, color=AMBER, linestyle="--", linewidth=1.0, alpha=0.5)
    ax.axvline(X_MIN, color=AMBER, linestyle="--", linewidth=1.0, alpha=0.5)
    ax.axvline(X_MAX, color=AMBER, linestyle="--", linewidth=1.0, alpha=0.5)

    # Obstacles
    for obs in obstacles:
        c = obs["center"]
        r = obs["radius"]
        circle = Circle(c, r, linewidth=1.2,
                        edgecolor=WHITE, facecolor=WHITE, alpha=0.2)
        ax.add_patch(circle)

    ax.set_xlim(X_MIN, X_MAX)
    ax.set_ylim(Y_MIN, Y_MAX)
    ax.set_xlabel("X (m)", color=GREEN, fontsize=10)
    ax.set_ylabel("Y (m)", color=GREEN, fontsize=10)
    ax.tick_params(colors=TEXT_COLOR, labelsize=8)
    ax.grid(True, color=GRID_COLOR, alpha=0.3)
    ax.set_aspect("equal")


def main():
    print(f"Loading data from {DATA_PATH}...")
    data = load_data(DATA_PATH)
    meta = data["meta"]
    opt = data["optimal_run"]
    frames = opt["frames"]
    k_avoid = opt.get("k_avoid", 4)
    rho0 = opt.get("rho0", 3.5)
    obstacles = meta["obstacles"]
    start = meta["start"]
    goal = meta["goal"]
    dt = meta["dt"]

    # Frame selection: skip=2 -> 25fps from 50fps base (~15s for 30s sim)
    skip = 2
    selected = frames[::skip]
    dt_frame = dt * skip
    fps = 1.0 / dt_frame

    print(f"  {len(frames)} source frames, {len(selected)} animation frames")
    print(f"  k_avoid={k_avoid}, rho0={rho0}, {len(obstacles)} obstacles")

    # Precompute APF potential grid
    print("  Computing APF potential field...")
    Xg, Yg, Pot = compute_apf_potential_grid(obstacles, k_avoid, rho0)

    print(f"  Rendering MP4 ({fps:.0f}fps, {len(selected)*dt_frame:.1f}s)...")
    fig, ax = plt.subplots(1, 1, figsize=(12, 6), facecolor="#0d1117")
    build_plot(ax, obstacles, Xg, Yg, Pot)

    # Start/goal markers
    ax.scatter(*start, s=80, marker="o", color=GREEN, edgecolors=WHITE,
               linewidths=0.5, zorder=5, label="Start")
    ax.scatter(*goal, s=80, marker="*", color=AMBER, edgecolors=WHITE,
               linewidths=0.5, zorder=5, label="Goal")
    ax.legend(loc="upper right", fontsize=8, facecolor=BG, edgecolor=GRID_COLOR,
              labelcolor=TEXT_COLOR)

    # Drone dot
    drone_dot = ax.scatter([], [], s=60, color=AMBER, edgecolors=WHITE,
                           linewidths=0.8, zorder=10)

    # Trail
    trail_line, = ax.plot([], [], color=AMBER, alpha=0.3, linewidth=1.5, zorder=4)

    # Waypoint marker
    wp_dot = ax.scatter([], [], s=40, marker="s", color="#58a6ff",
                        edgecolors=WHITE, linewidths=0.5, zorder=6)

    # HUD text
    hud = ax.text(0.02, 0.97, "", transform=ax.transAxes, fontsize=9,
                  fontfamily="monospace", color=GREEN, va="top",
                  bbox=dict(facecolor=BG, edgecolor=GRID_COLOR, alpha=0.8, pad=4))

    trail_x, trail_y = [], []
    prev_wp = np.array(goal)

    def update(i):
        nonlocal prev_wp
        f = selected[i]
        t = f["t"]
        pos = np.array(f["pos"])
        vel = np.array(f["vel"])
        dist = f["dist_to_goal"]
        F_apf = np.array(f["F_apf"])
        speed = np.linalg.norm(vel)

        drone_dot.set_offsets([pos])

        # Trail
        trail_x.append(pos[0])
        trail_y.append(pos[1])
        if len(trail_x) > 150:
            trail_x.pop(0)
            trail_y.pop(0)
        trail_line.set_data(trail_x, trail_y)

        # Waypoint detection: if dist was large and now small, waypoint switched
        # SWITCH_THRESHOLD not needed - just show the current target direction
        # Draw waypoint target: the actual goal minus pos direction
        target_dir = np.array(goal) - pos
        if np.linalg.norm(target_dir) > 0.5:
            wp_dot.set_offsets([goal])
        else:
            target_dir = np.array(start) - pos
            wp_dot.set_offsets([start])

        # HUD
        hud.set_text(
            f"t={t:5.1f}s | V={speed:5.2f} m/s | "
            f"d_goal={dist:5.1f}m | "
            f"|F_APF|={np.linalg.norm(F_apf):5.2f} N"
        )

        return drone_dot, trail_line, hud, wp_dot

    anim = matplotlib.animation.FuncAnimation(
        fig, update, frames=len(selected), interval=dt_frame * 1000, blit=False
    )

    os.makedirs("docs/assets/videos", exist_ok=True)
    anim.save(MP4_OUT, writer=FFMpegWriter(fps=fps, bitrate=4000))
    print(f"  Saved {MP4_OUT} ({len(selected)} frames)")
    plt.close(fig)
    print("Done.")


if __name__ == "__main__":
    main()

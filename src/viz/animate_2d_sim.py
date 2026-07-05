"""
Generate 2D APF path planning MP4 animation from simulation_2d.json.
Shows corridor, obstacles, APF potential contour, square drone with heading, HUD.
Matches 3D sim visual style: square drone marker, white obstacle circles.
"""

import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from matplotlib.patches import Circle, Polygon
from matplotlib.colors import LinearSegmentedColormap
import os

DATA_PATH = "docs/assets/data/simulation_2d.json"
MP4_OUT = "docs/assets/videos/apf_path_2d.mp4"

X_MIN, X_MAX = -15.0, 15.0
Y_MIN, Y_MAX = -5.0, 5.0

# Tron-meets-military color scheme
GREEN = "#3fb950"
DRONE_COLOR = "#d29922"
DRONE_EDGE = "#c9d1d9"
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
    for wall_y in [Y_MIN, Y_MAX]:
        ax.axhline(wall_y, color=GREEN, linestyle="--", linewidth=1.0, alpha=0.4)
    for wall_x in [X_MIN, X_MAX]:
        ax.axvline(wall_x, color=GREEN, linestyle="--", linewidth=1.0, alpha=0.4)

    # Obstacles - white filled with visible edge + O1-O15 numbering
    for i, obs in enumerate(obstacles):
        c = obs["center"]
        r = obs["radius"]
        circle = Circle(c, r, linewidth=1.0,
                        edgecolor=WHITE, facecolor=WHITE, alpha=0.12)
        ax.add_patch(circle)
        ax.text(c[0], c[1], f"O{i+1}", fontsize=7, color=WHITE,
                alpha=0.7, ha="center", va="center", zorder=12,
                fontfamily="monospace",
                bbox=dict(facecolor=BG, edgecolor="none", alpha=0.4,
                          pad=1))

    ax.set_xlim(X_MIN, X_MAX)
    ax.set_ylim(Y_MIN, Y_MAX)
    ax.tick_params(colors=TEXT_COLOR, labelsize=8, bottom=False, left=False,
                   labelbottom=False, labelleft=False)
    ax.grid(True, color=GRID_COLOR, alpha=0.2)
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

    # Start/goal markers (minimal)
    ax.scatter(*start, s=60, marker="o", color=GREEN, edgecolors=WHITE,
               linewidths=0.5, zorder=5, label="Start")
    ax.scatter(*goal, s=60, marker="*", color=GREEN, edgecolors=WHITE,
               linewidths=0.5, zorder=5, label="Goal")
    ax.legend(loc="upper right", fontsize=8, facecolor=BG, edgecolor=GRID_COLOR,
              labelcolor=TEXT_COLOR)

    # Drone square (Polygon — rotated to heading)
    half = 0.30
    drone_square = Polygon(np.zeros((4, 2)), closed=True,
                           facecolor=DRONE_COLOR, edgecolor=DRONE_EDGE,
                           linewidth=0.8, zorder=10)
    ax.add_patch(drone_square)

    # Heading line (extends from square center in velocity direction)
    heading_line, = ax.plot([], [], color=WHITE, linewidth=1.5, alpha=0.7,
                            solid_capstyle="round", zorder=11)

    # Trail
    trail_line, = ax.plot([], [], color=DRONE_COLOR, alpha=0.25, linewidth=1.2,
                          zorder=4)

    # HUD text
    hud = ax.text(0.02, 0.97, "", transform=ax.transAxes, fontsize=9,
                  fontfamily="monospace", color=GREEN, va="top",
                  bbox=dict(facecolor=BG, edgecolor=GRID_COLOR, alpha=0.8, pad=4))

    trail_x, trail_y = [], []
    sq_half = 0.30
    heading_len = 0.6
    corners_local = np.array([[-sq_half, -sq_half],
                              [ sq_half, -sq_half],
                              [ sq_half,  sq_half],
                              [-sq_half,  sq_half]])

    def update(i):
        f = selected[i]
        t = f["t"]
        pos = np.array(f["pos"])
        vel = np.array(f["vel"])
        dist = f["dist_to_goal"]
        speed = np.linalg.norm(vel)

        # Rotated square
        theta = np.arctan2(vel[1], vel[0])
        rot = np.array([[np.cos(theta), -np.sin(theta)],
                        [np.sin(theta),  np.cos(theta)]])
        drone_square.set_xy(corners_local @ rot.T + pos)

        # Heading line
        hx = pos[0] + heading_len * np.cos(theta)
        hy = pos[1] + heading_len * np.sin(theta)
        heading_line.set_data([pos[0], hx], [pos[1], hy])

        # Trail
        trail_x.append(pos[0])
        trail_y.append(pos[1])
        if len(trail_x) > 150:
            trail_x.pop(0)
            trail_y.pop(0)
        trail_line.set_data(trail_x, trail_y)

        # HUD
        hud.set_text(
            f"t={t:5.1f}s | V={speed:5.2f} m/s | "
            f"d_goal={dist:5.1f}m | "
            f"v=({vel[0]:5.2f},{vel[1]:5.2f})"
        )

        return drone_square, heading_line, trail_line, hud

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

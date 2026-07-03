"""
Generate 2D animated GIFs (top-down and side-view) from saved JSON data.
"""

import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import PillowWriter
from matplotlib.patches import Rectangle, Circle
import os

DATA_PATH = "docs/assets/data/swarm_simulation.json"
TOP_OUT = "docs/assets/images/top_down.gif"
SIDE_OUT = "docs/assets/images/side_view.gif"

DRONE_COLORS = {0: "#d29922", 1: "#3fb950", 2: "#3fb950",
                3: "#3fb950", 4: "#3fb950", 5: "#3fb950", 6: "#3fb950"}
MORPH_TIME = 8.0


def load_data(path):
    with open(path) as f:
        return json.load(f)


def build_top_down_plot(ax, meta):
    ax.set_facecolor("#0a0e14")
    ax.set_xlim(-8, 22)
    ax.set_ylim(-6, 6)
    ax.set_xlabel("X (m)", color="#3fb950", fontsize=9)
    ax.set_ylabel("Z (m)", color="#3fb950", fontsize=9)
    ax.tick_params(colors="#8b949e", labelsize=7)
    ax.grid(True, color="#21262d", alpha=0.3)
    ax.set_aspect("equal")

    for obs in meta["obstacles"]:
        if obs["type"] == "box":
            mc = np.array(obs["min_corner"])
            Mx = np.array(obs["max_corner"])
            w = Mx[0] - mc[0]
            h = Mx[2] - mc[2]
            rect = Rectangle((mc[0], mc[2]), w, h, linewidth=0.8,
                             edgecolor="#30363d", facecolor="#161b22", alpha=0.5)
            ax.add_patch(rect)
        elif obs["type"] == "cylinder":
            c = obs["center"]
            r = obs["radius"]
            circle = Circle((c[0], c[2]), r, linewidth=0.8,
                            edgecolor="#6b21a8", facecolor="#4a0e78", alpha=0.3)
            ax.add_patch(circle)

    # Target waypoint
    t = meta["target"]
    ax.plot(t[0], t[2], marker="*", markersize=10, color="#d29922", alpha=0.7,
            markeredgecolor="none")


def build_side_view_plot(ax, meta):
    ax.set_facecolor("#0a0e14")
    ax.set_xlim(-8, 22)
    ax.set_ylim(-1, 7)
    ax.set_xlabel("X (m)", color="#3fb950", fontsize=9)
    ax.set_ylabel("Y (m)", color="#3fb950", fontsize=9)
    ax.tick_params(colors="#8b949e", labelsize=7)
    ax.grid(True, color="#21262d", alpha=0.3)
    ax.set_aspect("equal")

    for obs in meta["obstacles"]:
        if obs["type"] == "box":
            mc = np.array(obs["min_corner"])
            Mx = np.array(obs["max_corner"])
            w = Mx[0] - mc[0]
            h = Mx[1] - mc[1]
            rect = Rectangle((mc[0], mc[1]), w, h, linewidth=0.8,
                             edgecolor="#30363d", facecolor="#161b22", alpha=0.5)
            ax.add_patch(rect)
        elif obs["type"] == "cylinder":
            c = obs["center"]
            r = obs["radius"]
            h = obs["height"]
            rect = Rectangle((c[0] - r, c[1] - h / 2), 2 * r, h,
                             linewidth=0.8, edgecolor="#6b21a8",
                             facecolor="#4a0e78", alpha=0.3)
            ax.add_patch(rect)

    t = meta["target"]
    ax.plot(t[0], t[1], marker="*", markersize=10, color="#d29922", alpha=0.7,
            markeredgecolor="none")


def main():
    print(f"Loading data from {DATA_PATH}...")
    data = load_data(DATA_PATH)
    frames = data["frames"]
    meta = data["meta"]

    # Skip frames for GIF (every 3rd frame -> ~16fps for 50fps source)
    skip = 3
    selected_frames = frames[::skip]
    dt = meta["dt"] * skip

    print(f"Generating top-down GIF ({len(selected_frames)} frames)...")

    fig_top, ax_top = plt.subplots(1, 1, figsize=(10, 6), facecolor="#0d1117")
    build_top_down_plot(ax_top, meta)

    top_scat = ax_top.scatter([], [], s=30, zorder=5)
    top_ghost = ax_top.scatter([], [], s=10, zorder=3, marker="o",
                               edgecolors="#484f58", facecolors="none", linewidths=0.5, alpha=0.4)
    top_title = ax_top.set_title("", color="#39d353", fontsize=10, fontfamily="monospace",
                                 fontweight="bold")

    trail_x = [[] for _ in range(meta["num_agents"])]
    trail_z = [[] for _ in range(meta["num_agents"])]

    def update_top(i):
        frame = selected_frames[i]
        t = frame["t"]
        drones = frame["drones"]
        xs = [d["pos"][0] for d in drones]
        zs = [d["pos"][2] for d in drones]
        colors = [DRONE_COLORS.get(d["id"], "#3fb950") for d in drones]

        top_scat.set_offsets(np.column_stack([xs, zs]))
        top_scat.set_color(colors)
        top_scat.set_edgecolors(colors)

        leader_pos = np.array(drones[0]["pos"])
        gx, gz = [], []
        for d in drones[1:]:
            off = np.array(d["d_ij"])
            tp = leader_pos + off
            gx.append(tp[0])
            gz.append(tp[2])
        top_ghost.set_offsets(np.column_stack([gx, gz]) if gx else np.empty((0, 2)))

        # Trails
        for j, d in enumerate(drones):
            trail_x[j].append(d["pos"][0])
            trail_z[j].append(d["pos"][2])
            if len(trail_x[j]) > 20:
                trail_x[j].pop(0)
                trail_z[j].pop(0)

        for j in range(meta["num_agents"]):
            if len(trail_x[j]) < 2:
                continue
            alphas = np.linspace(0.1, 0.6, len(trail_x[j]))
            c = DRONE_COLORS.get(j, "#3fb950")
            ax_top.plot(trail_x[j], trail_z[j], color=c, alpha=0.3, linewidth=1)

        form_state = "DIAMOND" if t >= MORPH_TIME else "WEDGE"
        err = frame["metrics"]["formation_error"]
        top_title.set_text(f"t = {t:.1f}s  |  {form_state}  |  E_fmt = {err:.3f}m")

        return top_scat, top_ghost, top_title

    anim_top = matplotlib.animation.FuncAnimation(fig_top, update_top,
                                                   frames=len(selected_frames),
                                                   interval=dt * 1000, blit=False)
    anim_top.save(TOP_OUT, writer=PillowWriter(fps=15))
    print(f"  Saved {TOP_OUT}")

    plt.close(fig_top)

    print(f"Generating side-view GIF ({len(selected_frames)} frames)...")

    fig_side, ax_side = plt.subplots(1, 1, figsize=(10, 6), facecolor="#0d1117")
    build_side_view_plot(ax_side, meta)

    side_scat = ax_side.scatter([], [], s=30, zorder=5)
    side_ghost = ax_side.scatter([], [], s=10, zorder=3, marker="o",
                                 edgecolors="#484f58", facecolors="none", linewidths=0.5, alpha=0.4)
    side_title = ax_side.set_title("", color="#39d353", fontsize=10, fontfamily="monospace",
                                   fontweight="bold")

    trail_x2 = [[] for _ in range(meta["num_agents"])]
    trail_y2 = [[] for _ in range(meta["num_agents"])]

    def update_side(i):
        frame = selected_frames[i]
        t = frame["t"]
        drones = frame["drones"]
        xs = [d["pos"][0] for d in drones]
        ys = [d["pos"][1] for d in drones]
        colors = [DRONE_COLORS.get(d["id"], "#3fb950") for d in drones]

        side_scat.set_offsets(np.column_stack([xs, ys]))
        side_scat.set_color(colors)
        side_scat.set_edgecolors(colors)

        leader_pos = np.array(drones[0]["pos"])
        gx, gy = [], []
        for d in drones[1:]:
            off = np.array(d["d_ij"])
            tp = leader_pos + off
            gx.append(tp[0])
            gy.append(tp[1])
        side_ghost.set_offsets(np.column_stack([gx, gy]) if gx else np.empty((0, 2)))

        for j, d in enumerate(drones):
            trail_x2[j].append(d["pos"][0])
            trail_y2[j].append(d["pos"][1])
            if len(trail_x2[j]) > 20:
                trail_x2[j].pop(0)
                trail_y2[j].pop(0)

        for j in range(meta["num_agents"]):
            if len(trail_x2[j]) < 2:
                continue
            c = DRONE_COLORS.get(j, "#3fb950")
            ax_side.plot(trail_x2[j], trail_y2[j], color=c, alpha=0.3, linewidth=1)

        form_state = "DIAMOND" if t >= MORPH_TIME else "WEDGE"
        err = frame["metrics"]["formation_error"]
        side_title.set_text(f"t = {t:.1f}s  |  {form_state}  |  E_fmt = {err:.3f}m")

        return side_scat, side_ghost, side_title

    anim_side = matplotlib.animation.FuncAnimation(fig_side, update_side,
                                                    frames=len(selected_frames),
                                                    interval=dt * 1000, blit=False)
    anim_side.save(SIDE_OUT, writer=PillowWriter(fps=15))
    print(f"  Saved {SIDE_OUT}")

    plt.close(fig_side)
    print("Done.")


if __name__ == "__main__":
    os.makedirs("docs/assets/images", exist_ok=True)
    os.makedirs("docs/assets/videos", exist_ok=True)
    main()

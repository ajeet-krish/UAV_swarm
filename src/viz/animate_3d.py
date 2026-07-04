"""
Generate a 3D MP4 animation of the UAV swarm navigating the orbital sphere field.
"""

import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import os

DATA_PATH = "docs/assets/data/swarm_simulation.json"
OUT_PATH = "docs/assets/videos/swarm_3d.mp4"

DRONE_COLORS = {
    0: "#d29922",  # leader
    1: "#3fb950", 2: "#3fb950",
    3: "#3fb950", 4: "#3fb950",
    5: "#3fb950", 6: "#3fb950",
}

TRAIL_LEN = 30


def load_data(path):
    with open(path) as f:
        return json.load(f)


def draw_sphere_wire(ax, center, radius, color="#ffffff", alpha=0.5):
    cx, cy, cz = center
    theta = np.linspace(0, 2 * np.pi, 32)

    # XY circle
    ax.plot(cx + radius * np.cos(theta), cy + radius * np.sin(theta), [cz] * len(theta),
            color=color, alpha=alpha, linewidth=1.2)
    # XZ circle
    ax.plot(cx + radius * np.cos(theta), [cy] * len(theta), cz + radius * np.sin(theta),
            color=color, alpha=alpha, linewidth=1.2)
    # YZ circle
    ax.plot([cx] * len(theta), cy + radius * np.cos(theta), cz + radius * np.sin(theta),
            color=color, alpha=alpha, linewidth=1.2)


def build_static_scene(ax, meta):
    for obs in meta["obstacles"]:
        if obs["type"] == "sphere":
            draw_sphere_wire(ax, obs["center"], obs["radius"])


def main():
    print(f"Loading data from {DATA_PATH}...")
    data = load_data(DATA_PATH)
    frames = data["frames"]
    meta = data["meta"]
    num_drones = meta["num_agents"]
    total_time = meta["total_time"]

    print(f"Rendering {len(frames)} frames for {num_drones} drones...")

    # Figure setup
    fig = plt.figure(figsize=(16, 9), facecolor="#0d1117")
    ax = fig.add_subplot(111, projection="3d", facecolor="#0a0e14")

    # Fixed coordinate bounds for deep space zone
    ax.set_xlim(-10, 10)
    ax.set_ylim(0, 12)
    ax.set_zlim(-10, 10)

    ax.set_xlabel("X (m)", color="#3fb950", fontsize=10)
    ax.set_ylabel("Y (m)", color="#3fb950", fontsize=10)
    ax.set_zlabel("Z (m)", color="#3fb950", fontsize=10)

    ax.xaxis.label.set_color("#3fb950")
    ax.yaxis.label.set_color("#3fb950")
    ax.zaxis.label.set_color("#3fb950")

    ax.tick_params(colors="#8b949e", labelsize=8)
    ax.xaxis.set_major_formatter(plt.FuncFormatter(lambda v, _: f"{v:.0f}"))
    ax.yaxis.set_major_formatter(plt.FuncFormatter(lambda v, _: f"{v:.0f}"))
    ax.zaxis.set_major_formatter(plt.FuncFormatter(lambda v, _: f"{v:.0f}"))
    ax.grid(True, color="#21262d", alpha=0.3)

    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.xaxis.pane.set_edgecolor("#21262d")
    ax.yaxis.pane.set_edgecolor("#21262d")
    ax.zaxis.pane.set_edgecolor("#21262d")

    # Static scene elements (asteroids)
    build_static_scene(ax, meta)

    # Static camera angle for fixed coordinate reference
    ax.view_init(elev=20, azim=-55)

    # Trail containers (per drone)
    trail_artists = []
    trail_x = [[] for _ in range(num_drones)]
    trail_y = [[] for _ in range(num_drones)]
    trail_z = [[] for _ in range(num_drones)]

    # Pre-allocate drone scatter
    drone_scat = ax.scatter([], [], [], s=45, zorder=5)
    ghost_scat = ax.scatter([], [], [], s=12, zorder=2, marker="o",
                            edgecolors="#484f58", facecolors="none", linewidths=0.5, alpha=0.3)

    # HUD text
    hud_time = ax.text2D(0.02, 0.95, "", transform=ax.transAxes,
                         color="#39d353", fontsize=11, fontfamily="monospace",
                         verticalalignment="top", fontweight="bold")
    hud_wp = ax.text2D(0.02, 0.88, "", transform=ax.transAxes,
                        color="#d29922", fontsize=9, fontfamily="monospace",
                        verticalalignment="top")
    hud_error = ax.text2D(0.02, 0.82, "", transform=ax.transAxes,
                          color="#8b949e", fontsize=8, fontfamily="monospace",
                          verticalalignment="top")
    hud_wind = ax.text2D(0.02, 0.76, "", transform=ax.transAxes,
                         color="#8b949e", fontsize=8, fontfamily="monospace",
                         verticalalignment="top")

    total_frames = len(frames)

    # Update function
    def animate(frame_idx):
        frame = frames[frame_idx]
        t = frame["t"]
        drones = frame["drones"]

        # Orbit camera 360 degrees over the full simulation
        azim = -55 + 360 * (frame_idx / max(total_frames - 1, 1))
        ax.view_init(elev=20, azim=azim)

        # Drone positions
        xs = [d["pos"][0] for d in drones]
        ys = [d["pos"][1] for d in drones]
        zs = [d["pos"][2] for d in drones]
        colors = [DRONE_COLORS.get(d["id"], "#3fb950") for d in drones]

        drone_scat._offsets3d = (xs, ys, zs)
        drone_scat.set_color(colors)
        drone_scat.set_edgecolors(colors)
        drone_scat.set_linewidths(0.5)

        # Ghost targets (formation offsets)
        leader_pos = np.array(drones[0]["pos"])
        gx, gy, gz = [], [], []
        for d in drones[1:]:
            off = np.array(d["d_ij"])
            target_pt = leader_pos + off
            gx.append(target_pt[0])
            gy.append(target_pt[1])
            gz.append(target_pt[2])
        ghost_scat._offsets3d = (gx, gy, gz)

        # Trails
        for i in range(num_drones):
            pos = np.array(drones[i]["pos"])
            trail_x[i].append(pos[0])
            trail_y[i].append(pos[1])
            trail_z[i].append(pos[2])
            if len(trail_x[i]) > TRAIL_LEN:
                trail_x[i].pop(0)
                trail_y[i].pop(0)
                trail_z[i].pop(0)

        # Remove old trail lines and redraw
        for art in trail_artists:
            art.remove()
        trail_artists.clear()

        for i in range(num_drones):
            n = len(trail_x[i])
            if n < 2:
                continue
            segments = []
            seg_colors = []
            for j in range(n - 1):
                segments.append([
                    [trail_x[i][j], trail_y[i][j], trail_z[i][j]],
                    [trail_x[i][j + 1], trail_y[i][j + 1], trail_z[i][j + 1]],
                ])
                alpha = 0.05 + (j / n) * 0.4
                c = DRONE_COLORS.get(i, "#3fb950")
                from matplotlib.colors import to_rgba
                seg_colors.append(to_rgba(c, alpha))

            if segments:
                lc = Line3DCollection(segments, colors=seg_colors, linewidths=1.2)
                ax.add_collection(lc)
                trail_artists.append(lc)

        # HUD update
        err = frame["metrics"]["formation_error"]
        wind_mag = np.linalg.norm(frame["wind"])

        # Find active target from meta and current position
        target_coords = np.array(meta["target"])
        hud_time.set_text(f"> t = {t:.1f}s  /  {total_time:.0f}s")
        hud_wp.set_text(f"> SYSTEM: ROTATING WEDGE ENGAGED")
        hud_error.set_text(f"> E_fmt = {err:.3f} m")
        hud_wind.set_text(f"> wind  = {wind_mag:.2f} m/s")

        return (drone_scat, ghost_scat, hud_time, hud_wp, hud_error, hud_wind) + tuple(trail_artists)

    # Pre-render first frame to set up
    animate(0)

    # Write video
    print(f"Writing {OUT_PATH}...")
    writer = FFMpegWriter(fps=50, bitrate=4000, codec="libx264")
    with writer.saving(fig, OUT_PATH, dpi=100):
        for i in range(len(frames)):
            animate(i)
            writer.grab_frame()
            if i % 200 == 0:
                print(f"  Frame {i}/{len(frames)}")

    print(f"Done. Saved to {OUT_PATH}")


if __name__ == "__main__":
    os.makedirs("docs/assets/videos", exist_ok=True)
    main()

"""
Generate a 2x3 faceted trajectory comparison image from simulation_2d.json.
Each panel shows one gain value with obstacle numbering, corridor walls, and
start/goal markers. Output saved to docs/assets/images/trajectory_facets.png.
"""

import json
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

DATA_PATH = "docs/assets/data/simulation_2d.json"
OUTPUT = "docs/assets/images/trajectory_facets.png"

X_MIN, X_MAX = -15.0, 15.0
Y_MIN, Y_MAX = -5.0, 5.0

BG = "#0a0e14"
FIG_BG = "#0d1117"
GRID_COLOR = "#21262d"
TEXT_COLOR = "#8b949e"
WHITE = "#c9d1d9"
GREEN = "#3fb950"
AMBER = "#d29922"

FACET_GAINS = ["0.05", "0.2", "0.5", "2", "4", "10"]
FACET_COLORS = ["#ff00ff", "#ff8800", "#88ff00", "#0088ff", "#3fb950", "#bc8cff"]


def main():
    with open(DATA_PATH) as f:
        data = json.load(f)
    meta = data["meta"]
    obstacles = meta["obstacles"]
    gs = data["gain_sweep"]

    fig, axes = plt.subplots(3, 2, figsize=(9, 11))
    fig.patch.set_facecolor(FIG_BG)

    for idx, (ax, ka) in enumerate(zip(axes.flat, FACET_GAINS)):
        ax.set_facecolor(BG)
        frames = gs[ka]["frames"]
        xs = [f["pos"][0] for f in frames]
        ys = [f["pos"][1] for f in frames]

        # Trajectory line
        ax.plot(xs, ys, color=FACET_COLORS[idx], linewidth=1.8, zorder=4)

        # Obstacle circles
        for oi, obs in enumerate(obstacles):
            c = obs["center"]
            r = obs["radius"]
            circle = Circle(c, r, linewidth=0.8, edgecolor=WHITE,
                            facecolor=WHITE, alpha=0.12, zorder=3)
            ax.add_patch(circle)
            ax.text(c[0], c[1], f"O{oi+1}", fontsize=6, color=WHITE,
                    alpha=0.65, ha="center", va="center", zorder=5,
                    fontfamily="monospace",
                    bbox=dict(facecolor=BG, edgecolor="none", alpha=0.5,
                              pad=1))

        # Corridor walls
        ax.axhline(Y_MIN, color=AMBER, linestyle="--", linewidth=0.8,
                   alpha=0.3, zorder=1)
        ax.axhline(Y_MAX, color=AMBER, linestyle="--", linewidth=0.8,
                   alpha=0.3, zorder=1)
        ax.axvline(X_MIN, color=AMBER, linestyle="--", linewidth=0.8,
                   alpha=0.3, zorder=1)
        ax.axvline(X_MAX, color=AMBER, linestyle="--", linewidth=0.8,
                   alpha=0.3, zorder=1)

        # Start / Goal squares
        ax.scatter(-12, 0, s=40, marker="s", color=GREEN, edgecolors=WHITE,
                   linewidths=0.4, zorder=6)
        ax.scatter(12, 0, s=40, marker="s", color=GREEN, edgecolors=WHITE,
                   linewidths=0.4, zorder=6)

        # k_a label at top
        ax.text(0.5, 1.02, f"k_a = {ka}", transform=ax.transAxes,
                fontsize=10, color=FACET_COLORS[idx], ha="center",
                va="bottom", fontfamily="monospace", fontweight="bold")

        # Axis styling
        ax.set_xlim(X_MIN, X_MAX)
        ax.set_ylim(Y_MIN, Y_MAX)
        ax.set_xticks([])
        ax.set_yticks([])
        ax.grid(True, color=GRID_COLOR, alpha=0.2, linewidth=0.4)
        for spine in ax.spines.values():
            spine.set_edgecolor(GRID_COLOR)
            spine.set_linewidth(0.5)



    plt.tight_layout(pad=1.5)
    os.makedirs("docs/assets/images", exist_ok=True)
    plt.savefig(OUTPUT, dpi=150, facecolor=FIG_BG, edgecolor="none")
    plt.close(fig)
    print(f"Saved {OUTPUT}")


if __name__ == "__main__":
    main()

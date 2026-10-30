"""Figures from the benchmark runs.

    python3 scripts/plot_benchmark.py results/sim_2026-10-30 [docs/figures] [repeat]

paths_<scenario>.png: top view of every controller's true path, obstacles and the pedestrian
track; clearance_<scenario>.png: distance to the closest obstacle over the lap.
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from metrics import CONTROLLERS, LABEL, load  # noqa: E402

from tb_nmpc_sim import scenarios  # noqa: E402

COLOR = {"nmpc": "#1f5fbf", "nmpc_nopred": "#8a63c9", "mppi": "#d9672b", "rpp": "#3d9a50"}


def clearance(d, scen, path):
    xy = np.stack([d["gt_x"], d["gt_y"]], axis=1)
    c = np.full(len(xy), np.inf)
    for _, ox, oy, r in scenarios.static_obstacles(scen, path):
        c = np.minimum(c, np.hypot(xy[:, 0] - ox, xy[:, 1] - oy) - scenarios.ROBOT_RADIUS - r)
    w = d["ped_state"] == 1
    cp = np.hypot(xy[:, 0] - d["ped_x"], xy[:, 1] - d["ped_y"]) - scenarios.ROBOT_RADIUS - scenarios.RADIUS["PED"]
    return np.where(w, np.minimum(c, cp), c)


def main():
    root = Path(sys.argv[1])
    figdir = Path(sys.argv[2]) if len(sys.argv) > 2 else root
    rep = sys.argv[3] if len(sys.argv) > 3 else "1"  # which repeat to draw
    figdir.mkdir(parents=True, exist_ok=True)
    path = scenarios.path()
    for scen in scenarios.SCENARIOS:
        runs = {c: load(root / f"{scen}_{c}_{rep}") for c in CONTROLLERS if (root / f"{scen}_{c}_{rep}" / "run.csv").exists()}
        if not runs:
            continue
        fig, ax = plt.subplots(figsize=(8, 4.6))
        ax.plot(path.xy[:, 0], path.xy[:, 1], color="0.6", lw=1, ls="--", label="reference")
        for _, ox, oy, r in scenarios.static_obstacles(scen, path):
            ax.add_patch(plt.Circle((ox, oy), r, color="0.35"))
        ped = scenarios.pedestrian(scen, path)
        if ped is not None:
            ax.annotate("", xy=ped[1], xytext=ped[0], arrowprops=dict(arrowstyle="->", color="0.3", lw=1.2))
            ax.text(*ped[0], " person", fontsize=9, va="top")
        for c, (d, meta) in runs.items():
            ax.plot(d["gt_x"], d["gt_y"], color=COLOR[c], lw=1.6, label=LABEL[c] + ("" if meta["completed"] else " (not finished)"))
        ax.plot(0, 0, "k>", ms=8)
        ax.set_aspect("equal")
        ax.set_xlim(-3.0, 1.0)
        ax.set_ylim(-2.4, 0.6)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_title(f"Scenario '{scen}': true robot paths in Webots")
        ax.legend(loc="upper left", fontsize=8, framealpha=0.9)
        ax.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(figdir / f"paths_{scen}.png", dpi=150)
        plt.close(fig)

        fig, ax = plt.subplots(figsize=(8, 3))
        for c, (d, _) in runs.items():
            cl = clearance(d, scen, path)
            if np.isfinite(cl).any():
                ax.plot(d["t"] - d["t"][0], np.where(np.isfinite(cl), cl, np.nan) * 100, color=COLOR[c], label=LABEL[c])
        ax.axhline(0, color="k", lw=0.8)
        ax.set_ylim(-30, 150)
        ax.set_xlabel("time [s]")
        ax.set_ylabel("clearance [cm]")
        ax.set_title(f"Scenario '{scen}': edge-to-edge distance to the closest obstacle")
        ax.legend(fontsize=8)
        ax.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(figdir / f"clearance_{scen}.png", dpi=150)
        plt.close(fig)
    print(f"figures in {figdir}")


if __name__ == "__main__":
    main()

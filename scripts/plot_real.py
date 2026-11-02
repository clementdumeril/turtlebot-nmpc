"""Top view of real-robot runs in the odometry frame, against the reference.

    python3 scripts/plot_real.py results/real_2026-11-01 docs/figures/real_circle_box.png \
        circle_clear_nmpc_2 circle_obstacle_nmpc_1 circle_object__mppi_1 --shape circle --size 0.5 --laps 2
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from metrics import LABEL, load  # noqa: E402
from plot_benchmark import COLOR  # noqa: E402

from tb_nmpc.reference import anchored  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("root")
    ap.add_argument("out")
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--shape", default="circle")
    ap.add_argument("--size", type=float, default=0.5)
    ap.add_argument("--laps", type=float, default=2.0)
    args = ap.parse_args()
    fig, ax = plt.subplots(figsize=(6, 5.5))
    ref_drawn = False
    styles = ["-", "-", "--", ":"]
    for i, name in enumerate(args.runs):
        d, meta = load(Path(args.root) / name)
        x0, y0, th0 = d["odom_x"][0], d["odom_y"][0], d["odom_yaw"][0]
        c, s = np.cos(-th0), np.sin(-th0)
        # express everything in the start frame (robot at the origin facing +x)
        dx, dy = d["odom_x"] - x0, d["odom_y"] - y0
        X, Y = c * dx - s * dy, s * dx + c * dy
        if not ref_drawn:
            p = anchored(args.size, 1.0, (0, 0, 0), args.shape)
            ax.plot(p.xy[:, 0], p.xy[:, 1], color="0.6", ls="--", lw=1, label="reference")
            ref_drawn = True
        col = COLOR.get(meta["controller"], "k") if "clear" not in name else "0.35"
        ax.plot(X, Y, color=col, lw=1.6, ls=styles[i % len(styles)], label=f"{LABEL.get(meta['controller'])}: {name}")
    ax.plot(0, 0, "k>", ms=8)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m] (start frame, odometry)")
    ax.set_ylabel("y [m]")
    ax.set_title("Real TurtleBot3 (apple): circle, with and without a box")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7, loc="upper left")
    fig.tight_layout()
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=150)
    print(args.out)


if __name__ == "__main__":
    main()

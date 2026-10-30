"""Side-by-side top view of two controllers in the same scenario, from the logged ground truth.

    python3 scripts/make_gif.py results/sim_2026-10-30 crossing docs/figures/crossing_nmpc_vs_mppi.gif
        [--left nmpc] [--right mppi] [--repeat 1] [--speed 2]

The window starts 4 s before the person starts walking and lasts until they have left.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from metrics import LABEL, load  # noqa: E402
from plot_benchmark import COLOR, clearance  # noqa: E402

from tb_nmpc_sim import scenarios  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("root")
    ap.add_argument("scenario")
    ap.add_argument("out")
    ap.add_argument("--left", default="nmpc")
    ap.add_argument("--right", default="mppi")
    ap.add_argument("--repeat", default="1")
    ap.add_argument("--speed", type=float, default=2.0)
    ap.add_argument("--fps", type=int, default=12)
    args = ap.parse_args()

    path = scenarios.path()
    runs = [(c, *load(Path(args.root) / f"{args.scenario}_{c}_{args.repeat}")) for c in (args.left, args.right)]
    # Common time window, relative to each run's own pedestrian start.
    windows = []
    for _, d, _ in runs:
        t = d["t"] - d["t"][0]
        walk = np.nonzero(d["ped_state"] >= 1)[0]
        gone = np.nonzero(d["ped_state"] >= 2)[0]
        t0 = t[walk[0]] - 4.0 if len(walk) else 0.0
        t1 = (t[gone[0]] + 3.0) if len(gone) else t[-1]
        windows.append((t, t0, t1))
    dur = min(w[2] - w[1] for w in windows)
    n_frames = int(dur / args.speed * args.fps)

    fig, axes = plt.subplots(1, 2, figsize=(10, 4.2))
    artists = []
    for ax, (c, d, _), (t, t0, _) in zip(axes, runs, windows):
        ax.plot(path.xy[:, 0], path.xy[:, 1], color="0.7", lw=1, ls="--")
        for _, ox, oy, r in scenarios.static_obstacles(args.scenario, path):
            ax.add_patch(plt.Circle((ox, oy), r, color="0.35"))
        trail, = ax.plot([], [], color=COLOR[c], lw=2)
        robot = plt.Circle((0, 0), scenarios.ROBOT_RADIUS, color=COLOR[c], alpha=0.9)
        ped = plt.Circle((40, 40), scenarios.RADIUS["PED"], color="0.15")
        ax.add_patch(robot)
        ax.add_patch(ped)
        txt = ax.text(0.02, 0.03, "", transform=ax.transAxes, fontsize=9)
        ax.set_xlim(-2.9, 0.9)
        ax.set_ylim(-2.4, 0.6)
        ax.set_aspect("equal")
        ax.set_xticks([])
        ax.set_yticks([])
        cl = clearance(d, args.scenario, path)
        ax.set_title(f"{LABEL[c]}   (min clearance {np.nanmin(np.where(np.isfinite(cl), cl, np.nan)) * 100:.0f} cm)",
                     fontsize=10)
        artists.append((d, t, t0, cl, trail, robot, ped, txt))
    fig.suptitle(f"Webots, person crossing at {scenarios.SCENARIOS[args.scenario]['ped']['speed']:.1f} m/s, "
                 f"{args.speed:.0f}x speed", fontsize=11)
    fig.tight_layout()

    def update(f):
        out = []
        for d, t, t0, cl, trail, robot, ped, txt in artists:
            tt = t0 + f * args.speed / args.fps
            i = min(int(np.searchsorted(t, tt)), len(t) - 1)
            j0 = int(np.searchsorted(t, t0 - 10.0))
            trail.set_data(d["gt_x"][j0:i + 1], d["gt_y"][j0:i + 1])
            robot.center = (d["gt_x"][i], d["gt_y"][i])
            ped.center = (d["ped_x"][i], d["ped_y"][i]) if d["ped_state"][i] == 1 else (40, 40)
            c = cl[i]
            txt.set_text(f"t = {tt:4.1f} s   clearance {c * 100:5.0f} cm" if np.isfinite(c) else f"t = {tt:4.1f} s")
            out += [trail, robot, ped, txt]
        return out

    anim = FuncAnimation(fig, update, frames=n_frames, blit=False)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    anim.save(args.out, writer=PillowWriter(fps=args.fps), dpi=80)
    print(f"{args.out}: {n_frames} frames")


if __name__ == "__main__":
    main()

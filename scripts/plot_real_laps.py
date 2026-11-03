"""Real NMPC circle lap in the odometry frame, coloured by speed, and its tracking error over time.

    PYTHONPATH=src/tb_nmpc python3 scripts/plot_real_laps.py results/real_2026-11-01 docs/figures/real_laps.png

Run circle_clear_nmpc_2 (r = 0.5 m, 2 laps). Tracking error is the logged cross-track error.
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from metrics import load  # noqa: E402

from tb_nmpc.reference import anchored  # noqa: E402

RUN, TITLE = "circle_clear_nmpc_2", "Circle, r = 0.5 m, 2 laps"


def smooth(v, n=15):
    return np.convolve(np.nan_to_num(v), np.ones(n) / n, mode="same")


def main():
    root, out = Path(sys.argv[1]), sys.argv[2]
    fig, (ax, axt) = plt.subplots(1, 2, figsize=(12, 5.2), gridspec_kw={"width_ratios": [1, 1.25]})
    d, meta = load(root / RUN)
    x0, y0, th0 = d["odom_x"][0], d["odom_y"][0], d["odom_yaw"][0]
    c, s = np.cos(-th0), np.sin(-th0)
    dx, dy = d["odom_x"] - x0, d["odom_y"] - y0
    X, Y = c * dx - s * dy, s * dx + c * dy
    v = smooth(d["odom_v"])
    p = anchored(0.5, 1.0, (0, 0, 0), "circle")
    ax.plot(p.xy[:, 0], p.xy[:, 1], color="0.55", ls="--", lw=1.2, label="reference")
    seg = np.stack([np.c_[X[:-1], Y[:-1]], np.c_[X[1:], Y[1:]]], axis=1)
    lc = LineCollection(seg, cmap="viridis", norm=plt.Normalize(0, 0.12), lw=2.6)
    lc.set_array(v[:-1])
    ax.add_collection(lc)
    ax.plot(0, 0, "k>", ms=9)
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_xlim(-0.62, 0.62)
    ax.set_ylim(-0.1, 1.1)
    ax.set_xlabel("x [m], start frame (odometry)")
    ax.set_ylabel("y [m]")
    ax.legend(loc="center", fontsize=9, frameon=False)
    cb = fig.colorbar(lc, ax=ax, shrink=0.8, pad=0.02)
    cb.set_label("speed [m/s]")
    cte = d["cte"]
    ok = np.isfinite(cte)
    t = d["t"] - d["t"][0]
    rmse = 100 * np.sqrt(np.mean(cte[ok] ** 2))
    ax.set_title(TITLE, fontsize=11)
    axt.plot(t[ok], 100 * cte[ok], color="#1d3f66", lw=1.2, label="cross-track error [cm]")
    axt.axhline(0, color="0.6", lw=0.8)
    axt.set_ylabel("cross-track error [cm]")
    axt.set_xlabel("time [s]")
    axt.grid(alpha=0.3)
    axv = axt.twinx()
    axv.plot(t, v, color="#d97b29", lw=1.2)
    axv.set_ylabel("speed [m/s]", color="#d97b29")
    axv.set_ylim(0, 0.15)
    axt.set_title(f"RMS {rmse:.1f} cm, max {100 * np.abs(cte[ok]).max():.1f} cm over {t[-1]:.0f} s", fontsize=11)
    fig.suptitle("Real TurtleBot3 (apple) following a circle with the NMPC, 1 November 2026", fontsize=13)
    fig.savefig(out, dpi=130, bbox_inches="tight")
    print(out, f"rmse {rmse:.2f} cm")


if __name__ == "__main__":
    main()

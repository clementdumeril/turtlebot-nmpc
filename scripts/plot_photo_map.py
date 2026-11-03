"""Final map of an exploration run with the robot's path, where each photo was taken, and some of
the photos pinned to their spot.

    python3 scripts/plot_photo_map.py <map.npz> <run dir with poses.csv> <out.png> --photos 0 3 6 ...

map.npz comes from scripts/bag_map.py.
"""
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import ConnectionPatch  # noqa: E402
from PIL import Image  # noqa: E402

SPIN, DRIVE, PATH = "#d97b29", "#2f7fc1", "#1d3f66"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("map")
    ap.add_argument("run")
    ap.add_argument("out")
    ap.add_argument("--photos", type=int, nargs="+", required=True)
    a = ap.parse_args()
    m = np.load(a.map)
    g, res, (ox, oy), trail = m["grid"], float(m["res"]), m["origin"], m["trail"]
    rows = list(csv.DictReader(open(Path(a.run) / "poses.csv")))
    P = np.array([[float(r["x"]), float(r["y"]), math.radians(float(r["yaw_deg"]))] for r in rows])
    d = np.hypot(*np.diff(P[:, :2], axis=0).T)
    spin = np.zeros(len(P), bool)
    spin[1:] |= d < 0.08
    spin[:-1] |= d < 0.08

    known = np.argwhere(g >= 0)
    (i0, j0), (i1, j1) = known.min(0), known.max(0)
    ext = [ox + j0 * res, ox + (j1 + 1) * res, oy + i0 * res, oy + (i1 + 1) * res]
    img = np.full(g.shape + (3,), 1.0)
    img[g < 0] = (0.93, 0.94, 0.95)
    img[(g >= 0) & (g < 65)] = (1.0, 1.0, 1.0)
    img[g >= 65] = (0.16, 0.18, 0.20)
    img = img[i0:i1 + 1, j0:j1 + 1]

    sel = sorted(a.photos, key=lambda k: P[k, 0])
    top, bottom = sel[::2], sel[1::2]
    cols = max(len(top), len(bottom))
    fig = plt.figure(figsize=(15, 10.5))
    gs = fig.add_gridspec(3, cols, height_ratios=[1, 3.2, 1], hspace=0.08, wspace=0.06,
                          left=0.02, right=0.98, top=0.93, bottom=0.03)
    ax = fig.add_subplot(gs[1, :])
    ax.imshow(img, extent=ext, origin="lower", interpolation="nearest")
    ax.plot(trail[:, 0], trail[:, 1], color=PATH, lw=1.6, alpha=0.8, label="path (cartographer)")
    ax.scatter(P[spin, 0], P[spin, 1], s=14, color=SPIN, zorder=3, label="photo during a spin in place")
    ax.scatter(P[~spin, 0], P[~spin, 1], s=14, color=DRIVE, zorder=3, label="photo while driving, every 30 cm")
    ax.plot(*trail[0, :2], "o", ms=9, mfc="white", mec=PATH, mew=2, zorder=4)
    ax.annotate("start", trail[0, :2], xytext=(8, 10), textcoords="offset points", fontsize=10, color=PATH)
    ax.set_xlim(ext[0], ext[1])
    ax.set_ylim(ext[2], ext[3])
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)
    ax.plot([ext[1] - 1.3, ext[1] - 0.3], [ext[2] + 0.3] * 2, color="k", lw=3)
    ax.text(ext[1] - 0.8, ext[2] + 0.45, "1 m", ha="center", fontsize=10)
    ax.legend(loc="upper left", fontsize=9, frameon=False)

    for row, ids in ((0, top), (2, bottom)):
        offset = (cols - len(ids)) / 2
        for c, k in enumerate(ids):
            # centre the shorter row
            sub = gs[row, int(c + offset)] if offset == int(offset) else gs[row, c]
            axp = fig.add_subplot(sub)
            axp.imshow(Image.open(Path(a.run) / rows[k]["file"]))
            axp.set_xticks([])
            axp.set_yticks([])
            for s in axp.spines.values():
                s.set_edgecolor(SPIN if spin[k] else DRIVE)
                s.set_linewidth(2.5)
            axp.set_title(f"{rows[k]['file'][4:8]}", fontsize=9, pad=2, color="0.3")
            x, y, th = P[k]
            ax.annotate("", (x + 0.35 * math.cos(th), y + 0.35 * math.sin(th)), (x, y),
                        arrowprops=dict(arrowstyle="-|>", color="k", lw=1.2), zorder=5)
            xy_img = (0.5, 0.0) if row == 0 else (0.5, 1.0)
            fig.add_artist(ConnectionPatch(xyA=xy_img, coordsA=axp.transAxes, xyB=(x, y), coordsB=ax.transData,
                                           color="0.45", lw=0.8, alpha=0.8))
    fig.suptitle("Real TurtleBot3 exploring the lab on its own: map, path and the photos it took (run 3, 2 November 2026)",
                 fontsize=13)
    fig.savefig(a.out, dpi=110)
    print(a.out, f"{len(P)} photos, {spin.sum()} in spins, {(~spin).sum()} while driving")


if __name__ == "__main__":
    main()

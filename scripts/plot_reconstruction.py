"""Top and side view of a SIFT + bundle-adjustment reconstruction with the photo spots.

    python3 scripts/plot_reconstruction.py <reconstruction dir> <run name> <out.png>
(<reconstruction dir> holds reconstruction/output/<run>/ba.ply and captures/<run>/poses.csv)
"""
import csv
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
base, run, out = sys.argv[1], sys.argv[2], sys.argv[3]
pts, col = [], []
lines = open(f"{base}/reconstruction/output/{run}/ba.ply").read().splitlines()
start = lines.index("end_header") + 1
for l in lines[start:]:
    v = l.split()
    pts.append([float(x) for x in v[:3]]); col.append([int(c) / 255 for c in v[3:6]])
P, C = np.array(pts), np.array(col)
rows = list(csv.DictReader(open(f"{base}/captures/{run}/poses.csv")))
cam = np.array([[float(r["x"]), float(r["y"])] for r in rows])
yaw = np.radians([float(r["yaw_deg"]) for r in rows])
fig, ax = plt.subplots(1, 2, figsize=(11, 5))
ax[0].scatter(P[:, 0], P[:, 1], c=C, s=14, edgecolors="k", linewidths=0.2)
ax[0].quiver(cam[:, 0], cam[:, 1], np.cos(yaw), np.sin(yaw), scale=12, width=0.004, color="#1f5fbf")
ax[0].plot(cam[:, 0], cam[:, 1], "o", color="#1f5fbf", ms=4, label="photo spots (arrows: camera direction)")
ax[0].set_aspect("equal"); ax[0].set_xlabel("x [m] (map)"); ax[0].set_ylabel("y [m]"); ax[0].set_title(f"Top view: {len(P)} points after bundle adjustment")
ax[0].legend(fontsize=8, loc="lower left"); ax[0].grid(alpha=0.3)
ax[1].scatter(P[:, 1], P[:, 2], c=C, s=14, edgecolors="k", linewidths=0.2)
ax[1].set_xlabel("y [m]"); ax[1].set_ylabel("z [m] (height)"); ax[1].set_title("Side view"); ax[1].grid(alpha=0.3)
fig.tight_layout(); fig.savefig(out, dpi=120)
print(P.min(0).round(2), P.max(0).round(2))

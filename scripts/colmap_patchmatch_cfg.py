"""COLMAP patch-match.cfg with source images picked from the robot's camera poses.

    python3 scripts/colmap_patchmatch_cfg.py <poses.csv> <out.cfg> [--n 10]

COLMAP's default picks sources by shared sparse points; with few SIFT tracks most images get none.
Here a source must look the same way (optical axes < 50 deg apart) from 4 cm to 2 m away; closer
to a 30 cm baseline and a smaller angle scores higher.
"""
import argparse
import csv

import numpy as np
from scipy.spatial.transform import Rotation

level = Rotation.from_euler("xyz", [-np.pi / 2, 0.0, -np.pi / 2]).as_matrix()
tilt = Rotation.from_euler("xyz", [8.12, 0.0, -0.17], degrees=True).as_matrix()
R_BC, t_BC = level @ tilt, np.array([0.115, 0.0, 0.0])

ap = argparse.ArgumentParser()
ap.add_argument("poses")
ap.add_argument("out")
ap.add_argument("--n", type=int, default=10)
a = ap.parse_args()
rows = list(csv.DictReader(open(a.poses)))
C, Z = [], []
for r in rows:
    Rb = Rotation.from_quat([float(r[k]) for k in ("qx", "qy", "qz", "qw")]).as_matrix()
    C.append(np.array([float(r[k]) for k in ("x", "y", "z")]) + Rb @ t_BC)
    Z.append(Rb @ R_BC[:, 2])
C, Z = np.array(C), np.array(Z)
lines, counts = [], []
for i, r in enumerate(rows):
    d = np.linalg.norm(C - C[i], axis=1)
    cos = Z @ Z[i]
    ok = (d > 0.04) & (d < 2.0) & (cos > np.cos(np.radians(50)))
    score = np.where(ok, np.minimum(d, 0.3) / 0.3 * cos, -1)
    src = [rows[j]["file"] for j in np.argsort(-score)[:a.n] if score[j] > 0]
    counts.append(len(src))
    if src:
        lines += [r["file"], ", ".join(src)]
open(a.out, "w").write("\n".join(lines) + "\n")
print(f"{len(lines) // 2}/{len(rows)} reference images, sources per image: min {min(counts)}, median {int(np.median(counts))}")

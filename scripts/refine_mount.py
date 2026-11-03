"""Refine the camera mount (base_link -> camera) from COLMAP feature tracks, robot poses fixed.

    python3 scripts/refine_mount.py <colmap TXT model> <poses.csv> <calibration.json> [start pitch, deg]

Unknowns: mount rotation (3) and mount x, y (2), plus every 3D point. The mount height cannot be
seen this way (it only shifts all points), so it stays at the given value. Huber loss on the
reprojection error. Prints the refined mount and the error before and after.
"""
import csv
import json
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation

level = Rotation.from_euler("xyz", [-np.pi / 2, 0.0, -np.pi / 2]).as_matrix()
MOUNT_RPY0 = np.array([float(sys.argv[4]) if len(sys.argv) > 4 else 8.12, 0.0, -0.17])  # Lab 6: 8.12 deg up
T0 = np.array([0.115, 0.0, 0.0])

model, poses_csv, calib = sys.argv[1:4]
cal = json.load(open(calib))
K, dist = np.array(cal["K"]), np.array(cal["dist"])

rows = {r["file"]: r for r in csv.DictReader(open(poses_csv))}
pts = {}
for l in open(model + "/points3D.txt"):
    if l.strip() and not l.startswith("#"):
        f = l.split()
        pts[int(f[0])] = np.array(list(map(float, f[1:4])))
pid = {p: k for k, p in enumerate(pts)}
X0 = np.array([pts[p] for p in pts])

obs_cam, obs_pt, obs_uv, base_R, base_t, names = [], [], [], [], [], []
L = [l for l in open(model + "/images.txt") if not l.startswith("#")]
for k in range(0, len(L) - 1, 2):
    head, p2d = L[k].split(), L[k + 1].split()
    if not head:
        continue
    name = head[-1]
    r = rows[name]
    c = len(names)
    names.append(name)
    base_R.append(Rotation.from_quat([float(r[q]) for q in ("qx", "qy", "qz", "qw")]).as_matrix())
    base_t.append(np.array([float(r[q]) for q in ("x", "y", "z")]))
    for j in range(0, len(p2d), 3):
        p = int(p2d[j + 2])
        if p in pid:
            obs_cam.append(c); obs_pt.append(pid[p]); obs_uv.append((float(p2d[j]), float(p2d[j + 1])))
obs_cam, obs_pt, obs_uv = np.array(obs_cam), np.array(obs_pt), np.array(obs_uv)
base_R, base_t = np.array(base_R), np.array(base_t)
print(f"{len(names)} images, {len(X0)} points, {len(obs_uv)} observations")


def mount(m):
    R = level @ Rotation.from_euler("xyz", MOUNT_RPY0 + m[:3], degrees=True).as_matrix()
    t = np.array([T0[0] + m[3], T0[1] + m[4], T0[2]])
    return R, t


def project(m, X):
    R_bc, t_bc = mount(m)
    Rb, tb = base_R[obs_cam], base_t[obs_cam]
    Rw = Rb @ R_bc                                    # camera -> world
    C = tb + np.einsum("nij,j->ni", Rb, t_bc)
    Xc = np.einsum("nji,nj->ni", Rw, X[obs_pt] - C)  # Rw^T (X - C)
    x, y = Xc[:, 0] / Xc[:, 2], Xc[:, 1] / Xc[:, 2]
    k1, k2, p1, p2, k3 = dist
    r2 = x * x + y * y
    rad = 1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3
    xd = x * rad + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
    yd = y * rad + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
    return np.c_[K[0, 0] * xd + K[0, 2], K[1, 1] * yd + K[1, 2]]


def residual(z):
    return (project(z[:5], z[5:].reshape(-1, 3)) - obs_uv).ravel()


A = lil_matrix((2 * len(obs_uv), 5 + 3 * len(X0)), dtype=int)
for k, p in enumerate(obs_pt):
    A[2 * k:2 * k + 2, :5] = 1
    A[2 * k:2 * k + 2, 5 + 3 * p:8 + 3 * p] = 1


def stats(z, tag):
    e = np.linalg.norm(residual(z).reshape(-1, 2), axis=1)
    print(f"{tag}: reprojection error median {np.median(e):.2f} px, mean {e.mean():.2f}, inliers<2px {np.mean(e < 2):.0%}")


z0 = np.r_[np.zeros(5), X0.ravel()]
stats(z0, "Lab 6 mount")
# points only first (mount fixed), then everything
sol = least_squares(lambda x: residual(np.r_[np.zeros(5), x]), X0.ravel(), loss="huber", f_scale=2.0,
                    jac_sparsity=A[:, 5:], x_scale="jac", max_nfev=50)
z1 = np.r_[np.zeros(5), sol.x]
stats(z1, "Lab 6 mount, points re-fit")
sol = least_squares(residual, z1, loss="huber", f_scale=2.0, jac_sparsity=A, x_scale="jac", max_nfev=200)
stats(sol.x, "refined mount")
m = sol.x[:5]
print("mount pitch-up %.2f deg, yaw %.2f deg, roll %.2f deg, x %.3f m, y %.3f m" %
      (MOUNT_RPY0[0] + m[0], MOUNT_RPY0[1] + m[1], MOUNT_RPY0[2] + m[2], T0[0] + m[3], T0[1] + m[4]))
np.save(model + "/refined_points.npy", sol.x[5:].reshape(-1, 3))
json.dump({"rpy_deg": (MOUNT_RPY0 + m[:3]).tolist(), "t": [T0[0] + m[3], T0[1] + m[4], T0[2]]},
          open(model + "/mount.json", "w"))

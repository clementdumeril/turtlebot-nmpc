"""360 deg panoramas from the robot's in-place spins, stitched with the robot's own yaw (no feature matching).

    python3 scripts/spin_panorama.py <map.npz> <run dir with poses.csv> <calibration.json> <out dir>

A spin is a run of photos taken less than 8 cm apart. Each panorama pixel is a ray from the spin
centre; it is placed at the distance the lidar map gives in that direction (first wall cell), so
nearby chairs line up despite the camera sitting 11.5 cm off the rotation axis. Photos are blended
with weights that fade to their borders. Writes pano_<k>.jpg (cylindrical, rows linear in
tan(elevation), column k at azimuth -2 pi (k + 0.5) / width) and spins.json (centre, heights, distance per column) for the viewer.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import median_filter

import textured_map as T

WP = 3600                       # panorama width, px (10 px per degree)
E_LO, E_HI = -11.0, 24.0        # elevation range, deg (camera sees about -12 to +28)
COLS = 720                      # distance samples per panorama for the viewer mesh


def spins_of(rows, min_photos):
    B = np.array([[float(r["x"]), float(r["y"])] for r in rows])
    d = np.hypot(*np.diff(B, axis=0).T)
    out, cur = [], [0]
    for k in range(1, len(rows)):
        if d[k - 1] < 0.08:
            cur.append(k)
        else:
            out.append(cur)
            cur = [k]
    out.append(cur)
    return [s for s in out if len(s) >= min_photos], B


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("map"); ap.add_argument("run"); ap.add_argument("calib"); ap.add_argument("out")
    ap.add_argument("--min-photos", type=int, default=12)
    a = ap.parse_args()
    run, out = Path(a.run), Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    m = np.load(a.map)
    occ, res, origin = m["grid"] >= 65, float(m["res"]), m["origin"]
    cams = T.Cams(run, Path(a.calib))
    rows = list(csv.DictReader(open(run / "poses.csv")))
    spins, B = spins_of(rows, a.min_photos)

    fc = WP / (2 * np.pi)
    y_hi, y_lo = fc * np.tan(np.radians(E_HI)), fc * np.tan(np.radians(E_LO))
    hp = int(round(y_hi - y_lo))
    xs, ys = np.meshgrid(np.arange(WP), np.arange(hp))
    phi = -2 * np.pi * (xs + 0.5) / WP        # left to right = turning clockwise, as seen from the centre
    tz = (y_hi - ys - 0.5) / fc
    meta = []
    for n, s in enumerate(spins):
        c0, z = B[s].mean(0), float(cams.C[s, 2].mean())
        pd = T.polar_depth(occ, res, origin, np.array([[c0[0], c0[1], z]]))[0]
        pd = median_filter(np.clip(pd, 0.6, 6.0), size=17, mode="wrap")
        dist = pd[(((phi[0] % (2 * np.pi)) / (2 * np.pi)) * T.AZ_BINS).astype(int) % T.AZ_BINS]
        D = np.broadcast_to(dist, phi.shape).ravel()
        P = np.c_[c0[0] + D * np.cos(phi).ravel(), c0[1] + D * np.sin(phi).ravel(), z + D * tz.ravel()]
        acc, ws = np.zeros((len(P), 3)), np.zeros(len(P))
        for c in s:
            uv, zc = cams.project(c, P)
            ok = (zc > 0.1) & (uv[:, 0] > 1) & (uv[:, 0] < cams.w - 2) & (uv[:, 1] > 1) & (uv[:, 1] < cams.h - 2)
            i = np.nonzero(ok)[0]
            w = (np.minimum(np.minimum(uv[i, 0], cams.w - uv[i, 0]) / 80, 1)
                 * np.minimum(np.minimum(uv[i, 1], cams.h - uv[i, 1]) / 40, 1)) ** 2 + 1e-4
            acc[i] += w[:, None] * cams.sample(c, uv[i])
            ws[i] += w
        img = np.where(ws[:, None] > 0, acc / np.maximum(ws, 1e-9)[:, None], 20).reshape(hp, WP, 3)
        Image.fromarray(img.astype(np.uint8)).save(out / f"pano_{n}.jpg", quality=88)
        cols = pd[((1 - (np.arange(COLS) + 0.5) / COLS) * T.AZ_BINS).astype(int) % T.AZ_BINS]  # column k: azimuth -2pi(k+0.5)/COLS
        meta.append({"file": f"pano_{n}.jpg", "centre": np.round(c0, 3).tolist(), "z": round(z, 4),
                     "tan_lo": float(np.tan(np.radians(E_LO))), "tan_hi": float(np.tan(np.radians(E_HI))),
                     "dist": np.round(cols, 3).tolist(), "photos": [rows[k]["file"] for k in s],
                     "coverage": round(float(np.mean(ws > 0)), 3)})
        print(f"pano_{n}: {len(s)} photos at ({c0[0]:.2f}, {c0[1]:.2f}), {np.mean(ws > 0):.0%} covered")
    (out / "spins.json").write_text(json.dumps(meta))


if __name__ == "__main__":
    main()

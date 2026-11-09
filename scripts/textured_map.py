"""2.5D model of an explored room: cartographer walls extruded to a fixed height and the floor,
both textured with the robot's own photos.

    python3 scripts/textured_map.py <map.npz> <run dir with poses.csv> <calibration.json> <out dir>
        [--pitch 8.12] [--camera-height H]

map.npz comes from scripts/bag_map.py. Every texture pixel is projected into the 76 photos (robot
pose x camera mount); a photo counts when the pixel faces it, falls inside it and no wall cell lies
in between (2D ray cast in the map). Visible photos are blended with weights favouring close,
head-on views. Without --camera-height, the floor height is the one that makes the floor look the
same from all photos (weak on a uniform carpet).
Writes model.json (geometry), walls.jpg and floor.jpg (texture atlases) and room.obj/.mtl.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage
from scipy.spatial.transform import Rotation

level = Rotation.from_euler("xyz", [-np.pi / 2, 0.0, -np.pi / 2]).as_matrix()
tilt = Rotation.from_euler("xyz", [8.12, 0.0, -0.17], degrees=True).as_matrix()
R_BC, t_BC = level @ tilt, np.array([0.115, 0.0, 0.0])
WALL_H = 1.0        # m
PPM = 160           # texture pixels per metre
AZ_BINS = 2880      # polar visibility resolution (0.125 deg)
MAX_RANGE = 8.0
POWER = 6           # weight exponent: close to "best view" but without hard seams


class Cams:
    def __init__(self, run: Path, calib: Path):
        cal = json.load(open(calib))
        self.K = np.array(cal["K"])
        self.dist = np.array(cal["dist"] + [0.0] * (5 - len(cal["dist"])))
        self.R, self.C, self.img, self.files = [], [], [], []
        for r in csv.DictReader(open(run / "poses.csv")):
            Rb = Rotation.from_quat([float(r[k]) for k in ("qx", "qy", "qz", "qw")]).as_matrix()
            pb = np.array([float(r[k]) for k in ("x", "y", "z")])
            self.R.append(Rb @ R_BC)
            self.C.append(pb + Rb @ t_BC)
            self.img.append(np.asarray(Image.open(run / r["file"]).convert("RGB"), np.float32))
            self.files.append(r["file"])
        self.C = np.array(self.C)
        self.h, self.w = self.img[0].shape[:2]

    def project(self, c: int, P: np.ndarray):
        """World points (N,3) -> pixel (N,2), depth (N,)."""
        X = (P - self.C[c]) @ self.R[c]
        z = X[:, 2]
        x, y = X[:, 0] / np.maximum(z, 1e-6), X[:, 1] / np.maximum(z, 1e-6)
        k1, k2, p1, p2, k3 = self.dist
        r2 = x * x + y * y
        rad = 1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3
        xd = x * rad + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
        yd = y * rad + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
        u = self.K[0, 0] * xd + self.K[0, 2]
        v = self.K[1, 1] * yd + self.K[1, 2]
        # the distortion polynomial folds back past r = 0.77; the image corners are at r < 0.68
        z = np.where(r2 < 0.70 ** 2, z, -1.0)
        return np.c_[u, v], z

    def sample(self, c: int, uv: np.ndarray) -> np.ndarray:
        im = self.img[c]
        u = np.clip(uv[:, 0] - 0.5, 0, self.w - 1.001)
        v = np.clip(uv[:, 1] - 0.5, 0, self.h - 1.001)
        u0, v0 = u.astype(int), v.astype(int)
        a, b = (u - u0)[:, None], (v - v0)[:, None]
        return ((1 - a) * (1 - b) * im[v0, u0] + a * (1 - b) * im[v0, u0 + 1]
                + (1 - a) * b * im[v0 + 1, u0] + a * b * im[v0 + 1, u0 + 1])


def polar_depth(occ, res, origin, C):
    """Distance to the first occupied cell from each camera, per azimuth bin."""
    az = (np.arange(AZ_BINS) + 0.5) * 2 * np.pi / AZ_BINS
    s = np.arange(0.05, MAX_RANGE, res / 3)
    out = np.empty((len(C), AZ_BINS))
    for c, (x, y, _) in enumerate(C):
        px = x + np.outer(np.cos(az), s)
        py = y + np.outer(np.sin(az), s)
        i = ((py - origin[1]) / res).astype(int)
        j = ((px - origin[0]) / res).astype(int)
        inside = (i >= 0) & (i < occ.shape[0]) & (j >= 0) & (j < occ.shape[1])
        hit = np.zeros_like(inside)
        hit[inside] = occ[i[inside], j[inside]]
        first = np.where(hit.any(1), hit.argmax(1), len(s) - 1)
        out[c] = s[first]
    return out


def blend(cams, depth, P, N, floor: bool, max_dist: float):
    """Weighted colour of points P (normals N) over all photos that see them."""
    acc = np.zeros((len(P), 3), np.float32)
    wsum = np.zeros(len(P), np.float32)
    for c in range(len(cams.C)):
        d = P - cams.C[c]
        dh = np.hypot(d[:, 0], d[:, 1])
        dist = np.linalg.norm(d, axis=1)
        cosi = -(d * N).sum(1) / np.maximum(dist, 1e-6)
        b = ((np.arctan2(d[:, 1], d[:, 0]) % (2 * np.pi)) / (2 * np.pi) * AZ_BINS).astype(int) % AZ_BINS
        # a wall face sits on the near edge of its cell: allow it up to the hit distance;
        # the floor must stop short of the wall
        tol = -0.03 if floor else 0.06
        ok = (cosi > 0.15) & (dh < depth[c, b] + tol) & (dist < max_dist)
        if not ok.any():
            continue
        idx = np.nonzero(ok)[0]
        uv, z = cams.project(c, P[idx])
        m = (z > 0.05) & (uv[:, 0] > 2) & (uv[:, 0] < cams.w - 3) & (uv[:, 1] > 2) & (uv[:, 1] < cams.h - 3)
        idx, uv = idx[m], uv[m]
        w = (cosi[idx] / np.maximum(dist[idx], 0.3)) ** POWER
        acc[idx] += w[:, None] * cams.sample(c, uv)
        wsum[idx] += w
    return acc, wsum


def floor_height(cams, depth, free, res, origin):
    """Floor z that minimises the colour spread of floor points seen from several photos."""
    ij = np.argwhere(free)[::7]
    xy = np.c_[origin[0] + (ij[:, 1] + 0.5) * res, origin[1] + (ij[:, 0] + 0.5) * res]
    best = None
    for h in np.arange(0.04, 0.26, 0.01):
        z0 = cams.C[:, 2].mean() - h
        P = np.c_[xy, np.full(len(xy), z0)]
        N = np.tile([0.0, 0.0, 1.0], (len(P), 1))
        s1 = np.zeros((len(P), 3)); s2 = np.zeros((len(P), 3)); n = np.zeros(len(P))
        for c in range(len(cams.C)):
            d = P - cams.C[c]
            dh = np.hypot(d[:, 0], d[:, 1])
            b = ((np.arctan2(d[:, 1], d[:, 0]) % (2 * np.pi)) / (2 * np.pi) * AZ_BINS).astype(int) % AZ_BINS
            ok = (dh < depth[c, b] - 0.03) & (dh < 2.0) & (dh > 0.3)
            idx = np.nonzero(ok)[0]
            uv, z = cams.project(c, P[idx])
            m = (z > 0.05) & (uv[:, 0] > 2) & (uv[:, 0] < cams.w - 3) & (uv[:, 1] > 2) & (uv[:, 1] < cams.h - 3)
            col = cams.sample(c, uv[m])
            s1[idx[m]] += col; s2[idx[m]] += col ** 2; n[idx[m]] += 1
        k = n >= 3
        var = (s2[k] / n[k, None] - (s1[k] / n[k, None]) ** 2).sum(1).mean()
        print(f"  camera {h:.2f} m above the floor: colour spread {var:7.1f} over {k.sum()} points")
        if best is None or var < best[0]:
            best = (var, h, z0)
    return best[1], best[2]


def atlas(colors, seen, n_tiles, tw, th, fallback):
    cols = max(1, int(math.ceil(math.sqrt(n_tiles * th / tw))))
    cols = min(cols, 4096 // tw)
    rows = int(math.ceil(n_tiles / cols))
    img = np.empty((rows * th, cols * tw, 3), np.uint8)
    img[:] = fallback
    t = colors.reshape(n_tiles, th, tw, 3)
    s = seen.reshape(n_tiles, th, tw)
    for k in range(n_tiles):
        r, c = divmod(k, cols)
        tile = np.where(s[k][..., None], t[k], fallback)
        img[r * th:(r + 1) * th, c * tw:(c + 1) * tw] = tile
    return img, cols, rows


def main():
    global R_BC
    ap = argparse.ArgumentParser()
    ap.add_argument("map"); ap.add_argument("run"); ap.add_argument("calib"); ap.add_argument("out")
    ap.add_argument("--pitch", type=float, default=8.12, help="camera tilt up, deg (calibrated value)")
    ap.add_argument("--camera-height", type=float, help="camera height above the floor, m")
    a = ap.parse_args()
    mp, run, calib, out = Path(a.map), Path(a.run), Path(a.calib), Path(a.out)
    R_BC = level @ Rotation.from_euler("xyz", [a.pitch, 0.0, -0.17], degrees=True).as_matrix()
    out.mkdir(parents=True, exist_ok=True)
    m = np.load(mp)
    grid, res, origin = m["grid"], float(m["res"]), m["origin"]
    occ = grid >= 65
    lab, n = ndimage.label(occ)
    sizes = np.bincount(lab.ravel())
    occ &= sizes[lab] >= 3                      # drop 1-2 cell specks
    free = (grid >= 0) & (grid < 65) & ~occ
    cams = Cams(run, calib)
    depth = polar_depth(occ, res, origin, cams.C)

    if a.camera_height is None:
        print("floor height:")
        h_cam, z0 = floor_height(cams, depth, (grid >= 0) & (grid < 25), res, origin)
    else:
        h_cam, z0 = a.camera_height, cams.C[:, 2].mean() - a.camera_height

    # wall faces: boundary between an occupied cell and a free neighbour, facing the free side
    faces = []
    for (di, dj, nx, ny) in [(0, 1, 1, 0), (0, -1, -1, 0), (1, 0, 0, 1), (-1, 0, 0, -1)]:
        nb = np.zeros_like(occ)
        src = free[max(di, 0):occ.shape[0] + min(di, 0), max(dj, 0):occ.shape[1] + min(dj, 0)]
        nb[max(-di, 0):occ.shape[0] + min(-di, 0), max(-dj, 0):occ.shape[1] + min(-dj, 0)] = src
        for i, j in np.argwhere(occ & nb):
            cx, cy = origin[0] + (j + 0.5) * res, origin[1] + (i + 0.5) * res
            mx, my = cx + nx * res / 2, cy + ny * res / 2
            # endpoints ordered so that (p1 - p0) x z = normal  (counter-clockwise seen from outside)
            tx, ty = -ny, nx
            faces.append((mx - tx * res / 2, my - ty * res / 2, mx + tx * res / 2, my + ty * res / 2, nx, ny))
    faces = np.array(faces)
    tw, th = int(round(res * PPM)), int(round(WALL_H * PPM))
    u = (np.arange(tw) + 0.5) / tw
    v = (np.arange(th) + 0.5) / th
    U, V = np.meshgrid(u, v)                                # (th, tw); row 0 = bottom
    U, V = U.ravel(), V.ravel()
    P = np.empty((len(faces), th * tw, 3))
    P[..., 0] = faces[:, None, 0] + (faces[:, None, 2] - faces[:, None, 0]) * U
    P[..., 1] = faces[:, None, 1] + (faces[:, None, 3] - faces[:, None, 1]) * U
    P[..., 2] = z0 + V * WALL_H
    Nw = np.repeat(np.c_[faces[:, 4:6], np.zeros(len(faces))], th * tw, axis=0)
    Pw = P.reshape(-1, 3) + 0.002 * Nw
    print(f"walls: {len(faces)} faces, {len(Pw)} texture pixels")
    acc, ws = blend(cams, depth, Pw, Nw, floor=False, max_dist=6.0)
    seen_w = ws > 0
    col_w = np.clip(acc / np.maximum(ws, 1e-12)[:, None], 0, 255).astype(np.uint8)
    # store tiles top row first for the image
    col_w = col_w.reshape(len(faces), th, tw, 3)[:, ::-1].reshape(-1, 3)
    seen_w = seen_w.reshape(len(faces), th, tw)[:, ::-1].reshape(-1)
    wall_img, wcols, wrows = atlas(col_w, seen_w, len(faces), tw, th, np.array([150, 150, 150], np.uint8))

    # floor: one tile per known cell
    fcells = np.argwhere(free)
    ft = tw
    fu = (np.arange(ft) + 0.5) / ft
    FU, FV = np.meshgrid(fu, fu)
    FU, FV = FU.ravel(), FV.ravel()
    Pf = np.empty((len(fcells), ft * ft, 3))
    Pf[..., 0] = origin[0] + (fcells[:, None, 1] + FU) * res
    Pf[..., 1] = origin[1] + (fcells[:, None, 0] + FV) * res
    Pf[..., 2] = z0
    Pf = Pf.reshape(-1, 3)
    Nf = np.tile([0.0, 0.0, 1.0], (len(Pf), 1))
    print(f"floor: {len(fcells)} cells, {len(Pf)} texture pixels")
    acc, ws = blend(cams, depth, Pf, Nf, floor=True, max_dist=3.5)
    seen_f = ws > 0
    col_f = np.clip(acc / np.maximum(ws, 1e-12)[:, None], 0, 255).astype(np.uint8)
    col_f = col_f.reshape(len(fcells), ft, ft, 3)[:, ::-1].reshape(-1, 3)
    seen_f = seen_f.reshape(len(fcells), ft, ft)[:, ::-1].reshape(-1)
    floor_img, fcols, frows = atlas(col_f, seen_f, len(fcells), ft, ft, np.array([200, 200, 200], np.uint8))

    Image.fromarray(wall_img).save(out / "walls.jpg", quality=88)
    Image.fromarray(floor_img).save(out / "floor.jpg", quality=85)
    model = {
        "res": res, "origin": [float(origin[0]), float(origin[1])], "z0": round(float(z0), 4),
        "wall_h": WALL_H, "camera_height": round(float(h_cam), 3), "pitch_deg": a.pitch,
        "walls": {"cols": wcols, "rows": wrows, "tw": tw, "th": th,
                  "faces": np.round(faces[:, :4], 4).tolist()},
        "floor": {"cols": fcols, "rows": frows, "t": ft, "cells": fcells.tolist()},
        "cams": [{"file": f, "C": np.round(C, 4).tolist(), "R": np.round(R, 5).tolist()}
                 for f, C, R in zip(cams.files, cams.C, cams.R)],
        "trail": np.round(m["trail"][:, :2], 3).tolist(),
        "occ": np.argwhere(occ).tolist(),
        "coverage": {"walls": round(float(seen_w.mean()), 3), "floor": round(float(seen_f.mean()), 3)},
    }
    (out / "model.json").write_text(json.dumps(model))
    write_obj(out, model)
    print(f"camera {h_cam:.2f} m above the floor; seen by a photo: walls {seen_w.mean():.0%}, floor {seen_f.mean():.0%}")


def write_obj(out: Path, m):
    """Quads with atlas UVs, two materials. Y up is left to the viewer; coordinates are the map frame."""
    v, vt, fw, ff = [], [], [], []
    W, Fl = m["walls"], m["floor"]
    z0, z1 = m["z0"], m["z0"] + m["wall_h"]
    for k, (x0, y0, x1, y1) in enumerate(W["faces"]):
        r, c = divmod(k, W["cols"])
        u0, u1 = c / W["cols"], (c + 1) / W["cols"]
        t0, t1 = 1 - (r + 1) / W["rows"], 1 - r / W["rows"]
        b = len(v)
        v += [(x0, y0, z0), (x1, y1, z0), (x1, y1, z1), (x0, y0, z1)]
        vt += [(u0, t0), (u1, t0), (u1, t1), (u0, t1)]
        fw.append(b)
    res, ox, oy = m["res"], *m["origin"]
    for k, (i, j) in enumerate(Fl["cells"]):
        r, c = divmod(k, Fl["cols"])
        u0, u1 = c / Fl["cols"], (c + 1) / Fl["cols"]
        t0, t1 = 1 - (r + 1) / Fl["rows"], 1 - r / Fl["rows"]
        x0, y0 = ox + j * res, oy + i * res
        b = len(v)
        v += [(x0, y0, z0), (x0 + res, y0, z0), (x0 + res, y0 + res, z0), (x0, y0 + res, z0)]
        vt += [(u0, t0), (u1, t0), (u1, t1), (u0, t1)]
        ff.append(b)
    with open(out / "room.obj", "w", newline="\n") as f:
        f.write("mtllib room.mtl\n")
        f.writelines(f"v {x:.4f} {y:.4f} {z:.4f}\n" for x, y, z in v)
        f.writelines(f"vt {a:.5f} {b:.5f}\n" for a, b in vt)
        for name, quads in (("walls", fw), ("floor", ff)):
            f.write(f"usemtl {name}\n")
            f.writelines(f"f {b+1}/{b+1} {b+2}/{b+2} {b+3}/{b+3} {b+4}/{b+4}\n" for b in quads)
    (out / "room.mtl").write_text("newmtl walls\nmap_Kd walls.jpg\n\nnewmtl floor\nmap_Kd floor.jpg\n", newline="\n")


if __name__ == "__main__":
    main()

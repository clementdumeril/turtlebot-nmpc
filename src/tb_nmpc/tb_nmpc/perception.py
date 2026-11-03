"""Lidar scan to circular obstacles.

Points are clustered by the gap between consecutive beams. Small clusters (a leg pair, a box,
a bin) become one circle each and go to the tracker; long clusters (walls, furniture) are cut
into chunks and passed on as static circles, since tracking a wall segment only produces fake
velocities as the chunk boundaries move.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class PerceptionParams:
    range_min: float = 0.12        # [m] drop returns closer than this (robot body, TB4 bumper)
    range_max: float = 3.0         # [m] ignore what is farther (not relevant within a 3 s horizon)
    gap_base: float = 0.12         # [m] new cluster when consecutive points are farther apart than
    gap_beams: float = 3.0         #     max(gap_base, gap_beams * range * angle_increment)
    min_points: int = 2            # clusters with fewer points are noise
    small_size: float = 0.6        # [m] clusters up to this extent are tracked objects
    r_min: float = 0.08            # [m] smallest circle radius
    chunk: float = 0.3             # [m] length of the pieces a long cluster is cut into
    merge_dist: float = 0.0        # [m] merge small circles whose centres are closer (two legs); 0 = off
    pole_max: float = 0.10         # [m] clusters this thin are poles: chair or table legs ...
    pole_radius: float = 0.0       # [m] ... given this radius (office-chair base below the lidar); 0 = off


def scan_points(ranges, angle_min: float, angle_inc: float, p: PerceptionParams):
    """Valid beams as (range, angle) arrays, in beam order."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_inc * np.arange(len(r))
    ok = np.isfinite(r) & (r >= p.range_min) & (r <= p.range_max)
    return r[ok], a[ok]


def cluster(r: np.ndarray, a: np.ndarray, angle_inc: float, p: PerceptionParams, full_circle: bool):
    """Index groups of consecutive beams that belong to the same surface."""
    if len(r) == 0:
        return []
    xy = np.stack([r * np.cos(a), r * np.sin(a)], axis=1)
    gaps = np.hypot(*np.diff(xy, axis=0).T)
    da = np.diff(a)
    tol = np.maximum(p.gap_base, p.gap_beams * np.minimum(r[:-1], r[1:]) * np.maximum(da, abs(angle_inc)))
    breaks = np.nonzero(gaps > tol)[0]
    groups = np.split(np.arange(len(r)), breaks + 1)
    if full_circle and len(groups) > 1:
        # Merge the last and first groups if they meet across the scan seam.
        g0, g1 = groups[0], groups[-1]
        if np.hypot(*(xy[g0[0]] - xy[g1[-1]])) <= max(p.gap_base, p.gap_beams * r[g0[0]] * abs(angle_inc)):
            groups = [np.r_[g1, g0]] + groups[1:-1]
    return [g for g in groups if len(g) >= p.min_points]


def circles(r, a, groups, sensor_pose, p: PerceptionParams):
    """Circles in the odometry frame.

    sensor_pose = (x, y, yaw) of the lidar in the odometry frame.
    Returns (small, static): arrays (n, 3) of [cx, cy, radius].
    """
    sx, sy, syaw = sensor_pose
    small, static, pole_flags = [], [], []
    for g in groups:
        ang = a[g] + syaw
        pts = np.stack([sx + r[g] * np.cos(ang), sy + r[g] * np.sin(ang)], axis=1)
        extent = float(np.hypot(*(pts[-1] - pts[0])))
        if extent <= p.small_size:
            c = pts.mean(0)
            rad = max(p.r_min, float(np.hypot(*(pts - c).T).max()))
            # The visible arc is the near side: push the centre away from the sensor.
            u = c - (sx, sy)
            u /= max(np.hypot(*u), 1e-6)
            c = c + 0.5 * rad * u
            small.append([c[0], c[1], rad])
            pole_flags.append(extent <= p.pole_max)
        else:
            n = max(1, int(np.ceil(extent / p.chunk)))
            for chunk in np.array_split(pts, n):
                if len(chunk) == 0:
                    continue
                c = chunk.mean(0)
                rad = max(p.r_min, float(np.hypot(*(chunk - c).T).max()))
                static.append([c[0], c[1], rad])
    small = np.array(small).reshape(-1, 3)
    poles = small[np.array(pole_flags, bool)] if len(small) else small
    if p.merge_dist > 0 and len(small) > 1:
        small = merge_close(small, p.merge_dist)
    if p.pole_radius > 0 and len(poles):
        # A TurtleBot3 lidar (17 cm up) sees only the column of an office chair: its five-leg base
        # spreads about 30 cm around, below the scan, and the robot drove into it. A thin cluster
        # that was not merged with a neighbour (two legs of a person) gets the base's radius.
        for k, c in enumerate(small):
            if np.any(np.all(np.abs(poles - c) < 1e-9, axis=1)):
                small[k, 2] = max(c[2], p.pole_radius)
    return small, np.array(static).reshape(-1, 3)


def merge_close(c: np.ndarray, dist: float) -> np.ndarray:
    """Merge circles whose centres are within dist into one enclosing circle (greedy)."""
    c = [row.copy() for row in c]
    merged = True
    while merged and len(c) > 1:
        merged = False
        for i in range(len(c)):
            for j in range(i + 1, len(c)):
                if np.hypot(c[i][0] - c[j][0], c[i][1] - c[j][1]) < dist:
                    ctr = 0.5 * (c[i][:2] + c[j][:2])
                    half = 0.5 * np.hypot(*(c[i][:2] - c[j][:2]))
                    c[i] = np.r_[ctr, half + max(c[i][2], c[j][2])]
                    del c[j]
                    merged = True
                    break
            if merged:
                break
    return np.array(c).reshape(-1, 3)


def detect(ranges, angle_min, angle_inc, sensor_pose, p: PerceptionParams):
    """Whole pipeline: scan to (small, static) circles in the odometry frame."""
    r, a = scan_points(ranges, angle_min, angle_inc, p)
    full = abs(angle_inc) * len(ranges) > 2 * np.pi - 2 * abs(angle_inc)
    groups = cluster(r, a, angle_inc, p, full)
    return circles(r, a, groups, sensor_pose, p)

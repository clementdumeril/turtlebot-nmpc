"""Log-odds occupancy grid from 2D scans at known poses (odometry), no loop closure.

The fallback mapper for simulation and for the offline check. On the robot, cartographer (as
in Lab 4) builds the map and corrects the odometry drift; this one trusts the odometry.
"""
from __future__ import annotations

import numpy as np

from .grid import Grid

L_FREE, L_OCC, L_MIN, L_MAX = -0.4, 0.85, -4.0, 4.0  # log-odds increments per scan and clamp bounds


class LogOddsMap:
    """Square log-odds grid of fixed size; `seen` marks cells any beam has crossed or ended in."""

    def __init__(self, size: float = 12.0, res: float = 0.05, center=(0.0, 0.0)):
        """size [m] side length, res [m] cell size, center (x, y) [m] of the square in the map frame."""
        n = int(round(size / res))
        self.res = res
        self.ox, self.oy = center[0] - size / 2, center[1] - size / 2
        self.l = np.zeros((n, n), np.float32)
        self.seen = np.zeros((n, n), bool)

    def _cells(self, x, y):
        """Flat cell indices of world points (x, y) arrays, dropping points outside the grid."""
        i = np.floor((y - self.oy) / self.res).astype(np.int64)
        j = np.floor((x - self.ox) / self.res).astype(np.int64)
        ok = (i >= 0) & (i < self.l.shape[0]) & (j >= 0) & (j < self.l.shape[1])
        return i[ok] * self.l.shape[1] + j[ok]

    def update(self, sensor_pose, ranges, angle_min: float, angle_inc: float, range_min: float,
               max_range: float = 3.0, inf_free: float = 2.0) -> None:
        """Integrate one scan taken at sensor_pose = (x, y, yaw) in the map frame.

        Returns beyond max_range and beams with no return (inf) clear space up to max_range or
        inf_free, without marking an obstacle.
        """
        sx, sy, syaw = sensor_pose
        r = np.asarray(ranges, float)
        a = angle_min + angle_inc * np.arange(len(r)) + syaw  # beam angles in the map frame
        finite = np.isfinite(r)
        valid = ~finite | (r >= range_min)
        hit = finite & (r >= range_min) & (r <= max_range)
        # free length per beam: up to one cell short of a hit, else max_range, else inf_free for inf
        free_len = np.where(hit, r - self.res, np.where(finite, np.minimum(r, max_range), inf_free))
        free_len = np.where(valid, np.maximum(free_len, 0.0), 0.0)
        # sample every beam at half-cell steps, all beams at once: n[b] samples for beam b,
        # beam = beam index of each sample, k = sample number within its beam
        step = self.res * 0.5
        n = np.ceil(free_len / step).astype(int)
        beam = np.repeat(np.arange(len(r)), n)
        k = np.arange(n.sum()) - np.repeat(np.cumsum(n) - n, n)
        d = k * step
        fx, fy = sx + d * np.cos(a[beam]), sy + d * np.sin(a[beam])
        # np.unique: each cell is updated once per scan, however many samples fall in it
        free_idx = np.unique(self._cells(fx, fy))
        hit_idx = np.unique(self._cells(sx + r[hit] * np.cos(a[hit]), sy + r[hit] * np.sin(a[hit])))
        flat, seen = self.l.ravel(), self.seen.ravel()  # views: writing them updates self.l / self.seen
        flat[free_idx] += L_FREE
        # a cell both crossed and hit counts as a hit only: cancel the free update it just got
        flat[hit_idx] += L_OCC - L_FREE * np.isin(hit_idx, free_idx)
        np.clip(flat, L_MIN, L_MAX, out=flat)
        seen[free_idx] = True
        seen[hit_idx] = True

    def grid(self) -> Grid:
        """Snapshot as a Grid: occupancy probability 0..100 from the log-odds, -1 where never seen."""
        p =1.0 - 1.0 / (1.0 + np.exp(self.l))
        data = np.where(self.seen, np.rint(100 * p), -1).astype(np.int16)
        return Grid(data, self.res, self.ox, self.oy)

"""Reference path for both controllers: a figure eight or a circle, and arc-length progress along it.

The figure eight is a lemniscate of Gerono, x = a sin t, y = a sin t cos t, started at the top
of its right lobe (t = pi/4) where the tangent is +x, and shifted so that the start is the
origin. The circle starts at the origin heading +x and turns left (centre at (0, r)). A robot
that starts at its odometry origin facing +x is on either path.
"""
from __future__ import annotations

import numpy as np


def lemniscate(a: float, laps: float = 1.0, points_per_lap: int = 4000) -> np.ndarray:
    """Dense polyline (n, 2) of the figure eight of half-width a [m], starting at the origin."""
    t0 = np.pi / 4
    t = t0 + np.linspace(0.0, 2 * np.pi * laps, max(2, int(points_per_lap * laps)) + 1)
    xy = np.stack([a * np.sin(t), a * np.sin(t) * np.cos(t)], axis=1)
    return xy - xy[0]


def circle(r: float, laps: float = 1.0, points_per_lap: int = 2000) -> np.ndarray:
    """Dense polyline (n, 2) of a circle of radius r [m], starting at the origin heading +x, turning left."""
    phi = np.linspace(0.0, 2 * np.pi * laps, max(2, int(points_per_lap * laps)) + 1)
    return np.stack([r * np.sin(phi), r * (1.0 - np.cos(phi))], axis=1)


SHAPES = {"eight": lemniscate, "circle": circle}


class Path:
    """Polyline with arc length, heading and curvature, and a local projection."""

    def __init__(self, xy: np.ndarray):
        xy = np.asarray(xy, dtype=float)
        keep = np.r_[True, np.hypot(*np.diff(xy, axis=0).T) > 1e-9]
        self.xy = xy[keep]
        ds = np.hypot(*np.diff(self.xy, axis=0).T)
        self.s = np.r_[0.0, np.cumsum(ds)]
        self.length = float(self.s[-1])
        d = np.gradient(self.xy, self.s, axis=0)
        self.theta = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
        self.kappa = np.gradient(self.theta, self.s)

    def sample(self, s) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """x, y, heading (unwrapped) and curvature at arc lengths s (clipped to the path)."""
        s = np.clip(np.asarray(s, dtype=float), 0.0, self.length)
        return (np.interp(s, self.s, self.xy[:, 0]), np.interp(s, self.s, self.xy[:, 1]),
                np.interp(s, self.s, self.theta), np.interp(s, self.s, self.kappa))

    def project(self, p, s_guess: float, back: float = 0.2, ahead: float = 0.8) -> tuple[float, float]:
        """Arc length of the closest point within [s_guess - back, s_guess + ahead], and the distance.

        The window keeps the progress monotonic where the figure eight crosses itself.
        """
        lo, hi = np.searchsorted(self.s, [s_guess - back, s_guess + ahead])
        lo, hi = max(lo - 1, 0), min(hi + 1, len(self.s))
        seg_a, seg_b = self.xy[lo:hi - 1], self.xy[lo + 1:hi]
        s_a = self.s[lo:hi - 1]
        if len(seg_a) == 0:
            return float(np.clip(s_guess, 0, self.length)), float(np.hypot(*(np.asarray(p) - self.xy[-1])))
        d = seg_b - seg_a
        L2 = np.maximum((d ** 2).sum(1), 1e-12)
        u = np.clip(((np.asarray(p) - seg_a) * d).sum(1) / L2, 0.0, 1.0)
        q = seg_a + u[:, None] * d
        dist = np.hypot(*(np.asarray(p) - q).T)
        i = int(np.argmin(dist))
        return float(s_a[i] + u[i] * np.sqrt(L2[i])), float(dist[i])

    def distance(self, pts: np.ndarray) -> np.ndarray:
        """Distance from each point (m, 2) to the whole polyline (cross-track error for metrics)."""
        pts = np.atleast_2d(pts)
        a, b = self.xy[:-1], self.xy[1:]
        d = b - a
        L2 = np.maximum((d ** 2).sum(1), 1e-12)
        out = np.empty(len(pts))
        for k in range(0, len(pts), 256):
            P = pts[k:k + 256, None, :]
            u = np.clip(((P - a) * d).sum(2) / L2, 0.0, 1.0)
            q = a + u[..., None] * d
            out[k:k + 256] = np.sqrt(((P - q) ** 2).sum(2)).min(1)
        return out

    def horizon_s(self, s0: float, v_ref: float, dt: float, n: int, slow_end: float = 0.4):
        """Arc lengths and speeds of the reference for stages 0..n, advancing at v_ref from s0.

        The speed ramps down over the last slow_end metres so the robot stops at the end.
        """
        s = np.empty(n + 1)
        v = np.empty(n + 1)
        s[0] = s0
        for k in range(n + 1):
            remaining = self.length - s[k]
            v[k] = v_ref * float(np.clip(remaining / slow_end, 0.0, 1.0)) if slow_end > 0 else v_ref
            if k < n:
                s[k + 1] = min(s[k] + max(v[k], 0.25 * v_ref if remaining > 0.02 else 0.0) * dt, self.length)
        return s, v

    def at(self, s: np.ndarray, v: np.ndarray) -> np.ndarray:
        """Rows (x, y, theta, v, omega) at arc lengths s with speeds v."""
        x, y, th, kap = self.sample(s)
        return np.stack([x, y, th, v, v * kap], axis=1)

    def horizon(self, s0: float, v_ref: float, dt: float, n: int, slow_end: float = 0.4) -> np.ndarray:
        """Reference for stages 0..n: rows (x, y, theta, v, omega)."""
        return self.at(*self.horizon_s(s0, v_ref, dt, n, slow_end))

    def skip_blocked(self, s: np.ndarray, circles: np.ndarray, step: float = 0.05) -> np.ndarray:
        """Move reference points that fall inside a circle (x, y, r) forward along the path.

        Without this, a static obstacle sitting on the path holds the whole 0.5 m reference
        inside its inflated radius, and stopping in front of it is the optimum (I saw the NMPC
        deadlock in front of a cylinder at the tip of a lobe). Stage 0 is left alone.
        """
        s = np.array(s, dtype=float)
        if len(circles) == 0:
            return s
        c = np.asarray(circles, dtype=float).reshape(-1, 3)
        for k in range(1, len(s)):
            s[k] = max(s[k], s[k - 1])
            while s[k] < self.length:
                x, y, _, _ = self.sample(s[k])
                if not np.any(np.hypot(c[:, 0] - x, c[:, 1] - y) < c[:, 2]):
                    break
                s[k] = min(s[k] + step, self.length)
        return s

def anchored(a: float, laps: float, pose, shape: str = "eight") -> Path:
    """The path (figure eight of half-width a, or circle of radius a) in the odometry frame,
    starting at pose = (x, y, yaw)."""
    xy = SHAPES[shape](a, laps)
    c, s = np.cos(pose[2]), np.sin(pose[2])
    return Path(xy @ np.array([[c, s], [-s, c]]) + np.asarray(pose[:2]))

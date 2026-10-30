"""Multi-object tracker: constant-velocity Kalman filter per obstacle, nearest-neighbour association.

State per track [x, y, vx, vy] in the odometry frame, measurement [x, y] (circle centre).
The prediction over the MPC horizon is x + v t, with a radius that grows with the velocity
uncertainty, so a track that was just born or is turning gets a wider berth.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from itertools import count

import numpy as np


@dataclass
class TrackerParams:
    sigma_a: float = 1.5        # [m/s^2] white-acceleration process noise (a walking person)
    sigma_z: float = 0.05       # [m] centre measurement noise
    gate: float = 0.5           # [m] max distance between prediction and detection
    max_age: float = 1.0        # [s] drop a track not seen for this long
    min_hits: int = 3           # detections before a track's velocity is trusted
    v_static: float = 0.10      # [m/s] below this a track is treated as static
    v_max: float = 2.5          # [m/s] velocity clamp (association glitches)
    r_alpha: float = 0.3        # radius low-pass gain
    grow_k: float = 1.0         # radius growth: grow_k * sigma_v * t ...
    grow_max: float = 0.25      # [m] ... capped here


@dataclass
class Track:
    id: int
    x: np.ndarray
    P: np.ndarray
    r: float
    t: float
    hits: int = 1
    born: float = 0.0
    history: list = field(default_factory=list)

    @property
    def speed(self) -> float:
        return float(np.hypot(self.x[2], self.x[3]))


class Tracker:
    def __init__(self, params: TrackerParams | None = None):
        self.p = params or TrackerParams()
        self.tracks: list[Track] = []
        self._ids = count()

    def _predict(self, tr: Track, t: float) -> None:
        dt = t - tr.t
        if dt <= 0:
            return
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        G = np.array([[0.5 * dt * dt, 0], [0, 0.5 * dt * dt], [dt, 0], [0, dt]])
        tr.x = F @ tr.x
        tr.P = F @ tr.P @ F.T + (self.p.sigma_a ** 2) * G @ G.T
        tr.t = t

    def _update(self, tr: Track, z: np.ndarray, r: float) -> None:
        H = np.zeros((2, 4))
        H[0, 0] = H[1, 1] = 1.0
        S = H @ tr.P @ H.T + (self.p.sigma_z ** 2) * np.eye(2)
        K = tr.P @ H.T @ np.linalg.inv(S)
        tr.x = tr.x + K @ (z - H @ tr.x)
        tr.P = (np.eye(4) - K @ H) @ tr.P
        v = np.hypot(tr.x[2], tr.x[3])
        if v > self.p.v_max:
            tr.x[2:] *= self.p.v_max / v
        tr.r = (1 - self.p.r_alpha) * tr.r + self.p.r_alpha * r
        tr.hits += 1

    def update(self, t: float, detections: np.ndarray) -> list[Track]:
        """Advance all tracks to time t and fuse the detections (n, 3) = [x, y, r]."""
        detections = np.asarray(detections, dtype=float).reshape(-1, 3)
        for tr in self.tracks:
            self._predict(tr, t)
        pairs = []
        for i, tr in enumerate(self.tracks):
            for j, d in enumerate(detections):
                dist = float(np.hypot(*(tr.x[:2] - d[:2])))
                if dist < self.p.gate:
                    pairs.append((dist, i, j))
        used_t, used_d = set(), set()
        for dist, i, j in sorted(pairs):
            if i in used_t or j in used_d:
                continue
            used_t.add(i)
            used_d.add(j)
            self._update(self.tracks[i], detections[j, :2], detections[j, 2])
            self.tracks[i].history.append(t)
        for j, d in enumerate(detections):
            if j not in used_d:
                P = np.diag([self.p.sigma_z ** 2] * 2 + [1.0, 1.0])
                self.tracks.append(Track(next(self._ids), np.r_[d[:2], 0.0, 0.0], P, float(d[2]), t, born=t, history=[t]))
        self.tracks = [tr for tr in self.tracks if t - tr.history[-1] <= self.p.max_age]
        return self.tracks

    def velocity(self, tr: Track) -> np.ndarray:
        """Velocity used for prediction: zero until the track is confirmed or if it is slow."""
        if tr.hits < self.p.min_hits or tr.speed < self.p.v_static:
            return np.zeros(2)
        return tr.x[2:].copy()

    def predict(self, tr: Track, times: np.ndarray, use_velocity: bool = True) -> np.ndarray:
        """Predicted circles (len(times), 3) = [x, y, r] at times measured from the track's last update."""
        times = np.asarray(times, dtype=float)
        v = self.velocity(tr) if use_velocity else np.zeros(2)
        xy = tr.x[:2] + times[:, None] * v
        r = np.full(len(times), tr.r)
        if use_velocity and np.any(v):
            sigma_v = float(np.sqrt(max(tr.P[2, 2], tr.P[3, 3])))
            r = r + np.minimum(self.p.grow_max, self.p.grow_k * sigma_v * times)
        return np.column_stack([xy, r])

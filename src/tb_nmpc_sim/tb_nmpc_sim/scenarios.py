"""Benchmark scenarios, defined along the figure eight so that every controller meets the same
obstacles at the same place. Used by the Webots supervisor plugin and by the metrics script.

Positions are in the world frame, which equals the odometry frame at launch: the robot starts
at the origin facing +x, at the start of the path.
"""
from __future__ import annotations

import numpy as np

from tb_nmpc.reference import Path, lemniscate

PATH_SIZE = 1.5        # [m] half-width of the figure eight (3 m x 1.5 m, 9.15 m per lap)
V_NOMINAL = 0.18       # [m/s] reference speed, used to time the crossing
ROBOT_RADIUS = 0.105   # TurtleBot3 Burger
RADIUS = {"OBS1": 0.15, "OBS2": 0.21, "PED": 0.15}  # cylinder r 0.15, box 0.3 x 0.3 (circumradius)

# static: (DEF name, arc length s [m], lateral offset [m], + = left of the path)
# ped: crossing point s, walking speed, start distance from the path. The person stands still
# on the robot's left, starts walking when the robot is where it would meet them at the
# crossing point at V_NOMINAL, and leaves the arena 1.8 m past the path.
SCENARIOS = {
    "clear": {"static": [], "ped": None},
    # OBS1 was first exactly on the path at the tip of the left lobe: Nav2 MPPI and my NMPC
    # (before Path.skip_blocked) both stopped in front of it, as a path follower without a
    # global planner will. 18 cm off the path both must swerve, and the path is not blocked.
    "static": {"static": [("OBS1", 5.5, 0.18), ("OBS2", 8.6, -0.12)], "ped": None},
    "crossing": {"static": [], "ped": {"s": 2.9, "speed": 0.5, "dist": 1.8}},
    "crossing_fast": {"static": [], "ped": {"s": 2.9, "speed": 1.0, "dist": 1.8}},
    "mixed": {"static": [("OBS1", 5.5, 0.18), ("OBS2", 8.6, -0.12)], "ped": {"s": 2.9, "speed": 0.5, "dist": 1.8}},
}


def path() -> Path:
    return Path(lemniscate(PATH_SIZE))


def static_obstacles(name: str, p: Path | None = None) -> list[tuple[str, float, float, float]]:
    """[(DEF, x, y, radius)] of the static obstacles of a scenario."""
    p = p or path()
    out = []
    for d, s, off in SCENARIOS[name]["static"]:
        x, y, th, _ = p.sample(s)
        out.append((d, float(x - off * np.sin(th)), float(y + off * np.cos(th)), RADIUS[d]))
    return out


def pedestrian(name: str, p: Path | None = None):
    """(start xy, end xy, velocity xy, trigger arc length) or None."""
    spec = SCENARIOS[name]["ped"]
    if spec is None:
        return None
    p = p or path()
    x, y, th, _ = p.sample(spec["s"])
    left = np.array([-np.sin(th), np.cos(th)])
    cross = np.array([float(x), float(y)])
    start = cross + left * spec["dist"]
    end = cross - left * spec["dist"]
    vel = -left * spec["speed"]
    s_trigger = spec["s"] - V_NOMINAL * spec["dist"] / spec["speed"]
    return start, end, vel, s_trigger

"""Exploration closed loop without ROS: a 4 m x 3 m room with an inner wall and two boxes,
a simulated 360-beam lidar, the log-odds map, the explorer's frontier planner and the NMPC.

    python3 scripts/explore_offline.py [--plot explore_offline.png]

Prints the explored fraction, goals, photo spots, time and the closest approach to a wall.
Takes about a minute.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path as FsPath

import numpy as np

ROOT = FsPath(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "tb_nmpc"))
sys.path.insert(0, str(ROOT / "src" / "tb_explore"))
from tb_explore.grid import plan_to_frontier, plan_to_point  # noqa: E402
from tb_explore.mapping import LogOddsMap  # noqa: E402
from tb_nmpc.ocp import FAR, Nmpc, OcpConfig  # noqa: E402
from tb_nmpc.perception import PerceptionParams, detect  # noqa: E402
from tb_nmpc.reference import Path, resample  # noqa: E402

ROBOT_R = 0.105


def box(cx, cy, hx, hy):
    p = [(cx - hx, cy - hy), (cx + hx, cy - hy), (cx + hx, cy + hy), (cx - hx, cy + hy)]
    return [(p[k], p[(k + 1) % 4]) for k in range(4)]


# Room 4 x 3 m (x in [-0.5, 3.5], y in [-1.5, 1.5]), an inner wall with a 0.8 m door, two boxes.
WALLS = box(1.5, 0.0, 2.0, 1.5) + [((1.6, -1.5), (1.6, -0.4)), ((1.6, 0.4), (1.6, 1.5))] \
    + box(0.6, 0.9, 0.15, 0.15) + box(2.6, -0.7, 0.2, 0.2)
SEG = np.array([[a[0], a[1], b[0], b[1]] for a, b in WALLS])


def raycast(x, y, yaw, n=360, rmax=3.5):
    ang = yaw - np.pi + 2 * np.pi * np.arange(n) / n
    dx, dy = np.cos(ang)[:, None], np.sin(ang)[:, None]
    ax, ay, bx, by = SEG.T
    ex, ey = bx - ax, by - ay
    den = dx * ey - dy * ex
    with np.errstate(divide="ignore", invalid="ignore"):
        t = ((ax - x) * ey - (ay - y) * ex) / den
        u = ((ax - x) * dy - (ay - y) * dx) / den
    t = np.where((den != 0) & (t > 0) & (u >= 0) & (u <= 1), t, np.inf)
    r = t.min(1)
    r[r > rmax] = np.inf
    return r, -np.pi, 2 * np.pi / n


def wall_distance(p):
    ax, ay, bx, by = SEG.T
    ex, ey = bx - ax, by - ay
    u = np.clip(((p[0] - ax) * ex + (p[1] - ay) * ey) / (ex * ex + ey * ey), 0, 1)
    return float(np.min(np.hypot(ax + u * ex - p[0], ay + u * ey - p[1])))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--plot", default="")
    args = ap.parse_args()
    cfg = OcpConfig(v_max=0.12)
    mpc = Nmpc(cfg)
    perc = PerceptionParams()
    m = LogOddsMap(8.0, 0.05, (1.5, 0.0))
    x = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    t, dt = 0.0, cfg.dt
    trail, spots, goals, blacklist, closest = [x[:2].copy()], [], 0, [], np.inf

    def sense():
        r, amin, inc = raycast(x[0], x[1], x[2])
        m.update((x[0], x[1], x[2]), r, amin, inc, 0.12)
        return r, amin, inc

    def spin():
        nonlocal t
        for k in range(12):  # 12 photos 30 degrees apart, about 1.2 s each
            x[2] += 2 * np.pi / 12
            sense()
            t += 1.2
        spots.append(x[:2].copy())

    def follow(xy, timeout=60.0):
        nonlocal t, closest
        path = Path(resample(xy))
        x[2] = np.arctan2(xy[min(1, len(xy) - 1), 1] - x[1], xy[min(1, len(xy) - 1), 0] - x[0])  # align
        x[3:] = 0.0
        mpc.reset()
        s, t0 = 0.0, t
        while s < path.length - 0.03 and t - t0 < timeout:
            r, amin, inc = sense() if round(t / dt) % 2 == 0 else raycast(x[0], x[1], x[2])  # lidar at 5 Hz
            small, static = detect(r, amin, inc, (x[0], x[1], x[2]), perc)
            circles = np.vstack([small, static]) if len(small) or len(static) else np.zeros((0, 3))
            s, _ = path.project(x[:2], s, back=0.1, ahead=0.3)
            sr, vr = path.horizon_s(s, 0.108, dt, cfg.N)
            infl = circles.copy()
            infl[:, 2] += ROBOT_R + 0.03
            sr = path.skip_blocked(sr, infl)
            ref = path.at(sr, vr)
            ref[:, 2] += np.round((x[2] - ref[0, 2]) / (2 * np.pi)) * 2 * np.pi
            obs = np.tile([FAR, FAR, 0.0], (cfg.N + 1, cfg.M, 1))
            if len(infl):
                d = np.hypot(infl[:, 0] - x[0], infl[:, 1] - x[1]) - infl[:, 2]
                for j, k in enumerate(np.argsort(d)[:cfg.M]):
                    obs[:, j, :] = infl[k]
            mpc.solve(x, ref, obs)
            x[:] = mpc.X[1]
            t += dt
            trail.append(x[:2].copy())
            closest = min(closest, wall_distance(x[:2]) - ROBOT_R)
        return s >= path.length - 0.03

    sense()
    spin()
    while t < 900:
        plan, clusters = plan_to_frontier(m.grid(), x[:2], 0.17, 0.40, 8, 0.35, 0.5, blacklist, 0.35)
        if plan is None:
            break
        ok = follow(plan.path_xy)
        blacklist.append(plan.goal_xy)
        goals += ok
        spin()
    home = plan_to_point(m.grid(), x[:2], (0.0, 0.0), 0.17, 0.40)
    if home is not None:
        follow(home)

    g = m.grid()
    # Explored fraction of the reachable floor (inside the room, more than the robot radius from walls).
    ii, jj = np.mgrid[0:g.shape[0], 0:g.shape[1]]
    cx, cy = g.world(ii, jj)
    inside = (cx > -0.45) & (cx < 3.45) & (cy > -1.45) & (cy < 1.45)
    known = g.data >= 0
    frac = (known & inside).sum() / inside.sum()
    print(f"explored {frac*100:.0f}% of the room, {goals} goals reached, {len(spots)} photo spots "
          f"({12*len(spots)} photos), {t:.0f} s, closest approach to a wall {closest*100:.1f} cm, "
          f"ended {np.hypot(*x[:2]):.2f} m from the start")
    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(7, 5.5))
        img = np.where(g.data < 0, 0.5, 1 - g.data / 100.0)
        ax.imshow(img, cmap="gray", origin="lower", extent=[g.ox, g.ox + g.shape[1] * g.res, g.oy, g.oy + g.shape[0] * g.res], vmin=0, vmax=1)
        for (a, b) in WALLS:
            ax.plot([a[0], b[0]], [a[1], b[1]], color="#b85a22", lw=1)
        tr = np.array(trail)
        ax.plot(tr[:, 0], tr[:, 1], color="#1f5fbf", lw=1.4, label="robot")
        sp = np.array(spots)
        ax.plot(sp[:, 0], sp[:, 1], "o", color="#2f7d4f", ms=6, label="photo spots (12 photos each)")
        ax.set_xlim(-0.8, 3.8)
        ax.set_ylim(-1.8, 1.8)
        ax.set_aspect("equal")
        ax.set_title(f"Offline exploration check: {frac*100:.0f}% explored, {len(spots)} photo spots")
        ax.legend(loc="lower right", fontsize=8)
        fig.tight_layout()
        fig.savefig(args.plot, dpi=130)


if __name__ == "__main__":
    main()

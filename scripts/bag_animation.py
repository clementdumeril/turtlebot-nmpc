"""Top-view animation of an exploration run from its rosbag: the cartographer map growing, the
robot's path, the lidar points and the explorer's goals.

    python3 scripts/bag_animation.py <bag dir> <out.gif> [--frames 120] [--fps 12]

The bag must be uncompressed (a bag recorded with --compression-mode file has a .db3.zstd file:
decompress it with zstd or python's zstandard and remove compression_format/mode from
metadata.yaml). Robot pose in the map = TF map -> odom (cartographer) composed with /odom.
"""
from __future__ import annotations

import argparse
import bisect
import math

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TOPICS = ["/map", "/odom", "/tf", "/scan", "/explorer/frontiers", "/explorer/state"]
LASER_X = -0.032


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read(bag: str):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    out = {k: [] for k in TOPICS}
    while r.has_next():
        name, data, t = r.read_next()
        if name not in out:
            continue
        msg = deserialize_message(data, get_message(types[name]))
        t = t * 1e-9
        if name == "/map":
            i = msg.info
            out[name].append((t, np.asarray(msg.data, np.int16).reshape(i.height, i.width), i.resolution,
                              i.origin.position.x, i.origin.position.y))
        elif name == "/odom":
            p = msg.pose.pose
            out[name].append((t, p.position.x, p.position.y, yaw_of(p.orientation)))
        elif name == "/tf":
            for tr in msg.transforms:
                if tr.header.frame_id == "map" and tr.child_frame_id == "odom":
                    a = tr.transform
                    out[name].append((t, a.translation.x, a.translation.y, yaw_of(a.rotation)))
        elif name == "/scan":
            r_ = np.asarray(msg.ranges, float)
            a_ = msg.angle_min + msg.angle_increment * np.arange(len(r_))
            ok = np.isfinite(r_) & (r_ > msg.range_min) & (r_ < 3.5)
            out[name].append((t, r_[ok], a_[ok]))
        elif name == "/explorer/frontiers":
            goal = [(m.pose.position.x, m.pose.position.y) for m in msg.markers if m.ns == "goal" and m.action == 0]
            pts = np.array([[p.x, p.y] for m in msg.markers if m.ns == "frontiers" for p in m.points]).reshape(-1, 2)
            out[name].append((t, goal[0] if goal else None, pts))
        elif name == "/explorer/state":
            out[name].append((t, msg.data))
    return out


def latest(seq, t):
    """Last element of a time-sorted list of (t, ...) at or before t, or None."""
    k = bisect.bisect_right([s[0] for s in seq], t) - 1
    return seq[k] if k >= 0 else None


def compose(a, b):
    x, y, th = a
    c, s = math.cos(th), math.sin(th)
    return x + c * b[0] - s * b[1], y + s * b[0] + c * b[1], th + b[2]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("out")
    ap.add_argument("--frames", type=int, default=120)
    ap.add_argument("--fps", type=int, default=12)
    args = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    d = read(args.bag)
    for k in d:
        d[k].sort(key=lambda s: s[0])
    t0 = max(d["/map"][0][0], d["/odom"][0][0])
    t1 = d["/odom"][-1][0]
    odom_t = [s[0] for s in d["/odom"]]

    def pose(t):
        o = latest(d["/odom"], t)
        m = latest(d["/tf"], t) or (t, 0.0, 0.0, 0.0)
        return compose(m[1:], o[1:]) if o else None

    # trail sampled every 0.2 s in the map frame
    ts = np.arange(t0, t1, 0.2)
    trail = np.array([pose(t)[:2] for t in ts])
    goals = [(s[0], s[1]) for s in d["/explorer/frontiers"] if s[1] is not None]

    _, gfin, res, ox, oy = d["/map"][-1]
    known = np.argwhere(gfin >= 0)
    lo = known.min(0) * res + [oy, ox]
    hi = known.max(0) * res + [oy, ox]
    pad = 0.4
    xlim, ylim = (lo[1] - pad, hi[1] + pad), (lo[0] - pad, hi[0] + pad)

    fig, ax = plt.subplots(figsize=(7.2, 7.2 * (ylim[1] - ylim[0]) / (xlim[1] - xlim[0]) + 0.6))
    fig.subplots_adjust(0.02, 0.02, 0.98, 0.92)
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    img = ax.imshow(np.zeros((2, 2)), cmap="gray", vmin=0, vmax=1, origin="lower", interpolation="nearest")
    path_line, = ax.plot([], [], color="#1f5fbf", lw=2)
    scan_pts, = ax.plot([], [], ".", color="#d1495b", ms=2.5)
    goal_pts, = ax.plot([], [], "*", color="#e08a1e", ms=14, mec="k", mew=0.5)
    robot = plt.Circle((0, 0), 0.105, color="#1f5fbf", zorder=5)
    ax.add_patch(robot)
    heading, = ax.plot([], [], color="white", lw=2, zorder=6)
    title = fig.suptitle("", fontsize=12)

    def frame(k):
        t = t0 + (t1 - t0) * k / max(args.frames - 1, 1)
        m = latest(d["/map"], t)
        if m is not None:
            _, g, r_, mx, my = m
            im = np.where(g < 0, 0.82, 1.0 - np.clip(g, 0, 100) / 100.0)
            img.set_data(im)
            img.set_extent([mx, mx + g.shape[1] * r_, my, my + g.shape[0] * r_])
        n = int(np.searchsorted(ts, t))
        path_line.set_data(trail[:n, 0], trail[:n, 1])
        p = pose(t)
        if p:
            robot.center = p[:2]
            heading.set_data([p[0], p[0] + 0.18 * math.cos(p[2])], [p[1], p[1] + 0.18 * math.sin(p[2])])
            sc = latest(d["/scan"], t)
            if sc is not None:
                ls = compose(p, (LASER_X, 0.0, 0.0))
                scan_pts.set_data(ls[0] + sc[1] * np.cos(sc[2] + ls[2]), ls[1] + sc[1] * np.sin(sc[2] + ls[2]))
        g_ = [g for tg, g in goals if tg <= t]
        goal_pts.set_data([g[0] for g in g_], [g[1] for g in g_])
        st = latest(d["/explorer/state"], t)
        label = {"SPIN": "turning in place", "PLAN": "planning", "ALIGN": "turning to the path",
                 "DRIVE": "driving to the frontier", "HOME": "driving home", "DONE": "done"}.get(st[1] if st else "", "")
        title.set_text(f"TurtleBot3 exploring on its own   t = {t - t0:5.0f} s   {label}")
        return [img, path_line, scan_pts, goal_pts, robot, heading, title]

    anim = FuncAnimation(fig, frame, frames=args.frames, blit=False)
    anim.save(args.out, writer=PillowWriter(fps=args.fps), dpi=90)
    frame(args.frames - 1)
    fig.savefig(args.out.rsplit(".", 1)[0] + "_final.png", dpi=150)
    print(f"{args.out}: {args.frames} frames, {t1 - t0:.0f} s of run, {len(goals)} goals, "
          f"map {xlim[1]-xlim[0]:.1f} x {ylim[1]-ylim[0]:.1f} m, path {np.sum(np.hypot(*np.diff(trail, axis=0).T)):.1f} m")


if __name__ == "__main__":
    main()

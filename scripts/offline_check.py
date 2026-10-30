"""Closed-loop check of the NMPC without ROS or Webots: unicycle simulation, figure eight,
one person crossing the path, perfect detections fed through the Kalman tracker.

    python3 scripts/offline_check.py [--no-predict] [--plot out.png]

Prints tracking RMSE, minimum clearance and solve times. Takes about 10 s.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "tb_nmpc"))
from tb_nmpc.ocp import FAR, Nmpc, OcpConfig  # noqa: E402
from tb_nmpc.reference import Path as RefPath, lemniscate  # noqa: E402
from tb_nmpc.tracker import Tracker  # noqa: E402

ROBOT_R, MARGIN = 0.105, 0.05


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-predict", action="store_true")
    ap.add_argument("--ped-speed", type=float, default=0.5)
    ap.add_argument("--plot", default="")
    args = ap.parse_args()

    cfg = OcpConfig()
    mpc = Nmpc(cfg)
    path = RefPath(lemniscate(1.5))
    tracker = Tracker()
    v_ref = 0.18
    # Person crossing the path perpendicular at s = 3.5 m, timed to meet the robot.
    s_cross = 3.5
    cx, cy, cth, _ = path.sample(s_cross)
    normal = np.array([-np.sin(cth), np.cos(cth)])
    t_meet = s_cross / v_ref
    p0 = np.array([cx, cy]) - normal * args.ped_speed * t_meet
    ped_v = normal * args.ped_speed
    ped_r = 0.15

    x = np.zeros(5)
    s_prog, t, dt = 0.0, 0.0, cfg.dt
    log, times = [], []
    while s_prog < path.length - 0.03 and t < 120:
        s_prog, _ = path.project(x[:2], s_prog)
        ref = path.horizon(s_prog, v_ref, dt, cfg.N)
        ped = p0 + ped_v * t
        if t % 0.2 < 1e-9 or abs(t % 0.2 - 0.2) < 1e-9:  # 5 Hz lidar
            tracker.update(t, np.array([[*(ped + np.random.normal(0, 0.02, 2)), ped_r]]))
        obs = np.tile([FAR, FAR, 0.0], (cfg.N + 1, cfg.M, 1))
        for j, tr in enumerate(tracker.tracks[:cfg.M]):
            ts = (t - tr.t) + dt * np.arange(cfg.N + 1)
            pr = tracker.predict(tr, ts, use_velocity=not args.no_predict)
            pr[:, 2] += ROBOT_R + MARGIN
            obs[:, j, :] = pr
        t0 = time.perf_counter()
        status, ts = mpc.solve(x, ref, obs)
        wall = time.perf_counter() - t0
        times.append((ts, wall))
        # Apply the first input exactly (perfect actuation).
        x = mpc.X[1].copy()
        t += dt
        log.append([t, *x, *ped, status])
    log = np.array(log)
    cte = path.distance(log[:, 1:3])
    clear = np.hypot(log[:, 1] - log[:, 6], log[:, 2] - log[:, 7]) - ROBOT_R - ped_r
    ts = np.array(times) * 1e3
    print(f"lap {t:.1f} s, cross-track RMSE {np.sqrt(np.mean(cte**2))*100:.1f} cm (max {cte.max()*100:.1f}), "
          f"min clearance {clear.min()*100:.1f} cm, solve median {np.median(ts[:,0]):.2f} ms max {ts[:,0].max():.2f} ms, "
          f"wall median {np.median(ts[:,1]):.2f} ms, nonzero status {(log[:,8]!=0).sum()}")
    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(7, 4))
        ax.plot(path.xy[:, 0], path.xy[:, 1], "k--", lw=1)
        ax.plot(log[:, 1], log[:, 2], "b")
        ax.plot(log[:, 6], log[:, 7], "r")
        ax.set_aspect("equal")
        fig.savefig(args.plot, dpi=120)


if __name__ == "__main__":
    main()

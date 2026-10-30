"""Metrics of real-robot runs (benchmark.py --real): no ground truth, so everything comes from
odometry, the commands and the lidar.

    python3 scripts/metrics_real.py results/real_2026-10-31 [--path-size 1.0]

- tracking error: odometry pose against the figure eight anchored where the run started;
- lidar clearance: smallest lidar range during the lap minus the lidar-to-edge distance
  (--edge, 0.075 m for a Burger whose LDS sits about 3 cm behind the centre);
- stops, angular acceleration and solve time as in metrics.py.
Writes summary_real.csv and summary_real.md.
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from metrics import LABEL, episodes, fmt, load  # noqa: E402

from tb_nmpc.reference import anchored  # noqa: E402


def run_metrics(run_dir: Path, path_size: float, edge: float) -> dict:
    d, meta = load(run_dir)
    t = d["t"] - d["t"][0]
    oxy = np.stack([d["odom_x"], d["odom_y"]], axis=1)
    path = anchored(path_size, 1.0, (d["odom_x"][0], d["odom_y"][0], d["odom_yaw"][0]))
    cte = path.distance(oxy)
    stops = episodes((t > 2.0) & (np.abs(d["odom_v"]) < 0.02), t, 0.5) - (1 if meta["completed"] else 0)
    ct, cw = d["cmd_t"], d["cmd_w"]
    ok = np.isfinite(ct) & np.isfinite(cw)
    ct, cw = ct[ok], cw[ok]
    keep = np.r_[True, np.diff(ct) > 1e-6] if len(ct) else np.array([], bool)
    ct, cw = ct[keep], cw[keep]
    alpha = np.diff(cw) / np.maximum(np.diff(ct), 1e-3) if len(ct) > 2 else np.array([np.nan])
    solve = d["solve_ms"][np.isfinite(d["solve_ms"])]
    scan = d.get("scan_min", np.full(len(t), np.nan))
    return {
        "run": run_dir.name, "controller": meta["controller"], "completed": bool(meta["completed"]),
        "lap_time_s": float(t[-1]) if meta["completed"] else float("nan"),
        "cte_odom_rmse_cm": float(np.sqrt(np.mean(cte ** 2)) * 100), "cte_odom_max_cm": float(cte.max() * 100),
        "lidar_clearance_cm": float((np.nanmin(scan) - edge) * 100) if np.isfinite(scan).any() else float("nan"),
        "stops": int(max(stops, 0)), "ang_accel_rms": float(np.sqrt(np.nanmean(alpha ** 2))),
        "solve_med_ms": float(np.median(solve)) if len(solve) else float("nan"),
        "solve_max_ms": float(solve.max()) if len(solve) else float("nan"),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("root")
    ap.add_argument("--path-size", type=float, default=1.0)
    ap.add_argument("--edge", type=float, default=0.075)
    args = ap.parse_args()
    root = Path(args.root)
    out = [run_metrics(r.parent, args.path_size, args.edge) for r in sorted(root.glob("*/meta.json"))]
    if not out:
        raise SystemExit("no runs")
    with open(root / "summary_real.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(out[0]))
        w.writeheader()
        w.writerows(out)
    lines = ["| Run | Controller | Done | Lap [s] | Tracking RMSE / max, odom [cm] | Lidar clearance [cm] | Stops "
             "| RMS ang. accel [rad/s²] | Solve median / max [ms] |", "|---|---|---|---|---|---|---|---|---|"]
    for r in out:
        lines.append(f"| {r['run']} | {LABEL.get(r['controller'], r['controller'])} | {'yes' if r['completed'] else 'no'} "
                     f"| {fmt(r['lap_time_s'])} | {fmt(r['cte_odom_rmse_cm'])} / {fmt(r['cte_odom_max_cm'])} "
                     f"| {fmt(r['lidar_clearance_cm'])} | {r['stops']} | {fmt(r['ang_accel_rms'], 2)} "
                     f"| {fmt(r['solve_med_ms'], 2)} / {fmt(r['solve_max_ms'], 2)} |")
    (root / "summary_real.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()

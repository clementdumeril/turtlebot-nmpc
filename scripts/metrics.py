"""Metrics of the benchmark runs, from the ground truth logged by benchmark.py.

    python3 scripts/metrics.py results/sim_2026-10-30

Reads every <dir>/<scenario>_<controller>_<repeat>/run.csv (or run.csv.gz), writes <dir>/summary.csv and summary.md.
- tracking error: distance from the robot to the figure eight over the whole lap (detours
  around obstacles included), RMSE and max. "odom" uses the odometry pose against the path
  anchored where the run started, which is what the controller sees; "true" uses the Webots
  ground truth and so also contains the odometry drift (about 10 cm per lap here, the same for
  every controller);
- clearance: edge-to-edge distance between the robot (r = 0.105 m) and the closest obstacle,
  static or the pedestrian; a collision is clearance < 0;
- stops: episodes where the robot's true speed stays below 0.02 m/s for 0.5 s or more before
  the end of the lap;
- smoothness: RMS of the commanded angular acceleration d(omega)/dt;
- solve time: acados time_tot per control step (NMPC only; Nav2 does not report it).
"""
from __future__ import annotations

import csv
import gzip
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "tb_nmpc"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "tb_nmpc_sim"))
from tb_nmpc.reference import anchored  # noqa: E402
from tb_nmpc_sim import scenarios  # noqa: E402

CONTROLLERS = ["nmpc", "nmpc_nopred", "mppi", "rpp"]
LABEL = {"nmpc": "NMPC + prediction", "nmpc_nopred": "NMPC, no prediction", "mppi": "Nav2 MPPI",
         "rpp": "Nav2 RPP"}


def load(run_dir: Path):
    f = run_dir / "run.csv"
    rows = list(csv.DictReader(open(f) if f.exists() else gzip.open(run_dir / "run.csv.gz", "rt")))
    data = {k: np.array([float(r[k]) if r[k] not in ("", "nan") else np.nan for r in rows]) for k in rows[0]}
    meta = json.loads((run_dir / "meta.json").read_text())
    return data, meta


def episodes(mask: np.ndarray, t: np.ndarray, min_len: float) -> int:
    n, start = 0, None
    for i, m in enumerate(mask):
        if m and start is None:
            start = t[i]
        if (not m or i == len(mask) - 1) and start is not None:
            if t[i] - start >= min_len:
                n += 1
            start = None
    return n


def run_metrics(run_dir: Path) -> dict:
    d, meta = load(run_dir)
    scen = meta["scenario"]
    path = scenarios.path()
    t = d["t"] - d["t"][0]
    xy = np.stack([d["gt_x"], d["gt_y"]], axis=1)
    cte = path.distance(xy)
    oxy = np.stack([d["odom_x"], d["odom_y"]], axis=1)
    opath = anchored(scenarios.PATH_SIZE, 1.0, (d["odom_x"][0], d["odom_y"][0], d["odom_yaw"][0]))
    cte_odom = opath.distance(oxy)
    clear = scenarios.static_clearance(scen, xy, path)
    walking = d["ped_state"] == 1
    if walking.any():
        cp = np.hypot(xy[:, 0] - d["ped_x"], xy[:, 1] - d["ped_y"]) - scenarios.ROBOT_RADIUS - scenarios.RADIUS["PED"]
        clear = np.where(walking, np.minimum(clear, cp), clear)
    # True speed over 0.2 s windows.
    k = 10
    sp = np.full(len(t), np.nan)
    sp[k:] = np.hypot(*(xy[k:] - xy[:-k]).T) / np.maximum(t[k:] - t[:-k], 1e-6)
    end = len(t) - 1
    if meta["completed"]:
        s = 0.0
        prog = []
        for p in xy:
            s, _ = path.project(p, s)
            prog.append(s)
        prog = np.array(prog)
        end = int(np.argmax(prog >= path.length - 0.05)) if np.any(prog >= path.length - 0.05) else end
    active = (t > 2.0) & (np.arange(len(t)) < end)
    stops = episodes(active & (sp < 0.02), t, 0.5)
    collisions = episodes(clear < 0, t, 0.0) if np.isfinite(clear).any() else 0
    # Commanded angular acceleration, from the command changes.
    ct, cw = d["cmd_t"], d["cmd_w"]
    ok = np.isfinite(ct) & np.isfinite(cw)
    ct, cw = ct[ok], cw[ok]
    keep = np.r_[True, np.diff(ct) > 1e-6]
    ct, cw = ct[keep], cw[keep]
    alpha = np.diff(cw) / np.maximum(np.diff(ct), 1e-3) if len(ct) > 2 else np.array([np.nan])
    solve = d["solve_ms"][np.isfinite(d["solve_ms"])]
    est_err = np.hypot(d["est_x"] - d["ped_x"], d["est_y"] - d["ped_y"])
    est_err = est_err[walking & np.isfinite(est_err)]
    vel_err = np.hypot(d["est_vx"] - d["ped_vx"], d["est_vy"] - d["ped_vy"])
    vel_err = vel_err[walking & np.isfinite(vel_err)]
    return {
        "scenario": scen, "controller": meta["controller"], "completed": bool(meta["completed"]),
        "lap_time_s": float(t[end]) if meta["completed"] else float("nan"),
        "cte_odom_rmse_cm": float(np.sqrt(np.mean(cte_odom[:end + 1] ** 2)) * 100),
        "cte_odom_max_cm": float(cte_odom[:end + 1].max() * 100),
        "cte_rmse_cm": float(np.sqrt(np.mean(cte[:end + 1] ** 2)) * 100),
        "cte_max_cm": float(cte[:end + 1].max() * 100),
        "min_clearance_cm": float(clear.min() * 100) if np.isfinite(clear).any() else float("nan"),
        "collisions": int(collisions),
        "stops": int(stops),
        "ang_accel_rms": float(np.sqrt(np.nanmean(alpha ** 2))),
        "solve_med_ms": float(np.median(solve)) if len(solve) else float("nan"),
        "solve_p99_ms": float(np.percentile(solve, 99)) if len(solve) else float("nan"),
        "solve_max_ms": float(solve.max()) if len(solve) else float("nan"),
        "ped_pos_err_cm": float(np.median(est_err) * 100) if len(est_err) else float("nan"),
        "ped_vel_err": float(np.median(vel_err)) if len(vel_err) else float("nan"),
    }


def fmt(x, nd=1):
    return "-" if x is None or (isinstance(x, float) and not np.isfinite(x)) else f"{x:.{nd}f}"


def main():
    root = Path(sys.argv[1])
    out = []
    for run in sorted(root.glob("*/meta.json")):
        try:
            out.append(run_metrics(run.parent))
        except Exception as e:  # noqa: BLE001
            print(f"{run.parent.name}: {e}")
    if not out:
        raise SystemExit("no runs")
    keys = list(out[0])
    with open(root / "runs.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(out)
    # One line per (scenario, controller): median over repeats, collisions and stops summed,
    # minimum clearance is the worst run, solve-time max is the max over runs.
    groups = {}
    for r in out:
        groups.setdefault((r["scenario"], r["controller"]), []).append(r)
    agg = []
    for (scen, ctl), rs in groups.items():
        a = {"scenario": scen, "controller": ctl, "runs": len(rs), "completed": sum(r["completed"] for r in rs)}
        for k in keys[3:]:
            v = np.array([r[k] for r in rs], dtype=float)
            if k in ("collisions", "stops"):
                a[k] = int(v.sum())
            elif k == "min_clearance_cm":
                a[k] = float(np.nanmin(v)) if np.isfinite(v).any() else float("nan")
            elif k == "solve_max_ms":
                a[k] = float(np.nanmax(v)) if np.isfinite(v).any() else float("nan")
            else:
                a[k] = float(np.nanmedian(v)) if np.isfinite(v).any() else float("nan")
        agg.append(a)
    with open(root / "summary.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(agg[0]))
        w.writeheader()
        w.writerows(agg)
    out = agg
    order = {c: i for i, c in enumerate(CONTROLLERS)}
    out.sort(key=lambda r: (list(scenarios.SCENARIOS).index(r["scenario"]), order.get(r["controller"], 9)))
    lines = ["Median over runs; collisions and stops summed over runs; clearance is the worst run.", "",
             "| Scenario | Controller | Lap done | Lap time [s] | Tracking RMSE / max, odom [cm] | Tracking RMSE, true [cm] | Min clearance [cm] "
             "| Collisions | Stops | RMS ang. accel [rad/s²] | Solve median / p99 / max [ms] |",
             "|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in out:
        lines.append(f"| {r['scenario']} | {LABEL.get(r['controller'], r['controller'])} | {r['completed']}/{r['runs']} "
                     f"| {fmt(r['lap_time_s'])} | {fmt(r['cte_odom_rmse_cm'])} / {fmt(r['cte_odom_max_cm'])} | {fmt(r['cte_rmse_cm'])} "
                     f"| {fmt(r['min_clearance_cm'])} | {r['collisions']} | {r['stops']} | {fmt(r['ang_accel_rms'], 2)} "
                     f"| {fmt(r['solve_med_ms'], 2)} / {fmt(r['solve_p99_ms'], 2)} / {fmt(r['solve_max_ms'], 2)} |")
    (root / "summary.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()

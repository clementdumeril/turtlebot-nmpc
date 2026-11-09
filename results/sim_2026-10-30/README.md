# Webots benchmark, 30 October 2026

## Configuration

- TurtleBot3 Burger in Webots R2025a (Windows), ROS 2 Jazzy in WSL2, Intel i7-6700HQ laptop. Real-time simulation, 20 ms physics step, lidar at 5 Hz (as the real LDS-01).
- Figure eight 3 m x 1.5 m (9.15 m, `path_size` 1.5), one lap, started from rest at the origin.
- Same caps for every controller: 0.20 m/s, 2.0 rad/s, forward only. NMPC reference speed 0.18 m/s, RPP desired speed 0.18 m/s; MPPI has no target speed and drives at its cap.
- Scenarios ([scenarios.py](../../src/tb_nmpc_sim/tb_nmpc_sim/scenarios.py)):
  - `clear`: no obstacle.
  - `static`: a 0.15 m cylinder 18 cm off the path at the tip of the left lobe, a 0.3 m box 12 cm off the path near the end.
  - `crossing`: a 0.15 m-radius "person" walks across the first diagonal at 0.5 m/s, from the robot's left, timed to reach the path when the robot would at 0.18 m/s. They do not yield.
  - `crossing_fast`: the same at 1.0 m/s.
- Two runs per scenario and controller, each in a fresh simulation: `bash scripts/run_benchmark.sh results/sim_2026-10-30`, 80 min.
- Per run: `<scenario>_<controller>_<repeat>/run.csv.gz` (50 Hz ground truth, odometry, commands, solver stats, the pedestrian track) and `meta.json`. Metrics: `python3 scripts/metrics.py results/sim_2026-10-30` writes [summary.md](summary.md), [summary.csv](summary.csv) (aggregated) and [runs.csv](runs.csv) (per run).

## Results

Update, 31 October: 5 more NMPC runs of the 0.5 m/s crossing (and one more MPPI run) are in [sim_2026-10-31_crossing_repeats](../sim_2026-10-31_crossing_repeats). Over 7 comparable runs the NMPC with prediction made contact once (-2.2 cm); the 0/2 below was a small sample.

From [summary.md](summary.md). Tracking error is measured over the whole lap, detours included, in the odometry frame (what the controller sees). Clearance is edge to edge with the true geometry; negative means contact.

| Scenario | Controller | Contact (runs) | Worst clearance [cm] | Tracking RMSE, odom [cm] | Lap [s] |
|---|---|---|---|---|---|
| clear | NMPC | - | - | 0.2 | 53.0 |
| clear | Nav2 MPPI | - | - | 8.6 | 45.7 |
| clear | Nav2 RPP | - | - | 2.1 | 64.3 |
| static | NMPC | 0/2 | 4.0 | 9.6 | 54.9 |
| static | Nav2 MPPI | 1/2 | -7.1 | 9.9 | 51.2 |
| static | Nav2 RPP | 2/2 | -7.0 | 2.1 | 65.1 |
| crossing 0.5 m/s | NMPC + prediction | 0/2 | 11.0 | 1.9 | 56.8 |
| crossing 0.5 m/s | NMPC, no prediction | 2/2 | -9.2 | 1.2 | 55.8 |
| crossing 0.5 m/s | Nav2 MPPI | 2/2 | -21.8 | 8.1 | 48.4 |
| crossing 0.5 m/s | Nav2 RPP | 2/2 | -7.3 | 2.3 | 65.8 |
| crossing 1.0 m/s | NMPC + prediction | 2/2 | -8.5 | 0.8 | 56.2 |
| crossing 1.0 m/s | NMPC, no prediction | 2/2 | -20.5 | 0.5 | 54.3 |
| crossing 1.0 m/s | Nav2 MPPI | 2/2 | -25.0 | 8.6 | 47.6 |
| crossing 1.0 m/s | Nav2 RPP | 2/2 | -15.7 | 2.1 | 66.2 |

NMPC solve time (acados `time_tot`), all 16 NMPC runs: median 0.79 to 0.91 ms per run, p99 under 2.2 ms in 15 runs, worst 11.7 ms (one run with a p99 of 10.4 ms, probably WSL scheduling: the problem was the same clear lap as the others), against a 100 ms control period. Pedestrian track against the truth while they walk (NMPC runs, median per run): position error 8 to 16 cm and velocity error 0.06 to 0.11 m/s at 0.5 m/s; 20 to 25 cm and 0.11 to 0.21 m/s at 1.0 m/s.

## What the curves show

- At 0.5 m/s the predictive NMPC slows down and turns behind the person ([paths](../../docs/figures/paths_crossing.png), [clearance](../../docs/figures/clearance_crossing.png)). It stops for a moment while they pass, which counts as 2 to 4 stops per run. Without prediction the same NMPC treats the person as standing still where it last saw them, keeps going, and is hit from the side. MPPI, which sees the person only in the current costmap, is hit deeper (-18 and -22 cm).
- At 1.0 m/s the person covers the 1.8 m to the path in 1.8 s. The tracker needs 3 scans at 5 Hz (0.6 s) before it trusts a velocity, so the NMPC has about one second, and at 0.2 m/s and 0.5 m/s² it cannot get out of the way. Prediction still roughly halves the depth of contact (-8.5 vs -20.5 cm) but does not avoid it.
- After a contact the person pushes the robot, its wheels slip and the odometry no longer matches the ground truth: the rest of the true path is shifted (RPP and NMPC without prediction in `paths_crossing.png`). This is why the true tracking error in `summary.md` jumps to 30 cm or more in those runs.
- MPPI drives at its 0.20 m/s cap and cuts the lobes: 8.6 cm RMSE on the clear lap. RPP follows closely (2.1 cm) but has no avoidance: it touches both static obstacles.
- The NMPC's commanded angular acceleration is higher than MPPI's when it avoids (0.68 vs 0.36 rad/s² RMS at 0.5 m/s), lower on the clear lap (0.13 vs 0.29).

## Cause and next step

The 1.0 m/s failure is a sensing and authority limit, not a solver one: 5 Hz lidar, three scans to confirm a velocity, 0.2 m/s top speed. Next: confirm tracks after two scans, and check whether letting the NMPC reverse (v_min < 0) helps. Then the same scenarios on the real robot ([how to run it](../../docs/REPRODUCING.md#on-a-turtlebot3)).

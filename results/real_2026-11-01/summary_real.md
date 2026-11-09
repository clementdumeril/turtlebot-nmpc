Generated with `python3 scripts/metrics_real.py results/real_2026-11-01 --path-shape circle --path-size 0.5` (circle runs) and `--path-shape eight --path-size 0.5` (clear_nmpc_1).

| Run | Controller | Done | Lap [s] | Tracking RMSE / max, odom [cm] | Lidar clearance [cm] | Stops | RMS ang. accel [rad/s²] | Solve median / max [ms] |
|---|---|---|---|---|---|---|---|---|
| circle_clear_nmpc_1 | NMPC + prediction | yes | 75.4 | 1.9 / 8.6 | 44.9 | 1 | 1.01 | 1.31 / 2.70 |
| circle_clear_nmpc_2 | NMPC + prediction | yes | 70.0 | 1.0 / 3.4 | 72.0 | 0 | 0.46 | 1.30 / 2.20 |
| circle_object__mppi_1 | Nav2 MPPI | yes | 97.4 | 29.4 / 49.3 | 10.1 | 4 | 0.64 | - / - |
| circle_obstacle_nmpc_1 | NMPC + prediction | yes | 57.2 | 16.2 / 34.4 | 8.3 | 3 | 1.45 | 1.45 / 2.96 |
| clear_nmpc_1 | NMPC + prediction | yes | 68.3 | 6.1 / 11.2 | 13.5 | 5 | 0.91 | 1.18 / 2.26 |

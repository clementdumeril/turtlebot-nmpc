Median over runs; collisions and stops summed over runs; clearance is the worst run.

| Scenario | Controller | Lap done | Lap time [s] | Tracking RMSE / max, odom [cm] | Tracking RMSE, true [cm] | Min clearance [cm] | Collisions | Stops | RMS ang. accel [rad/s²] | Solve median / p99 / max [ms] |
|---|---|---|---|---|---|---|---|---|---|---|
| clear | NMPC + prediction | 2/2 | 53.0 | 0.2 / 0.6 | 3.2 | - | 0 | 0 | 0.13 | 0.85 / 1.89 / 3.48 |
| clear | NMPC, no prediction | 2/2 | 52.9 | 0.2 / 0.6 | 3.2 | - | 0 | 0 | 0.21 | 0.89 / 6.42 / 11.71 |
| clear | Nav2 MPPI | 2/2 | 45.7 | 8.6 / 19.4 | 8.8 | - | 0 | 0 | 0.29 | - / - / - |
| clear | Nav2 RPP | 2/2 | 64.3 | 2.1 / 4.2 | 2.1 | - | 0 | 0 | 0.04 | - / - / - |
| static | NMPC + prediction | 2/2 | 54.9 | 9.6 / 26.4 | 11.9 | 4.0 | 0 | 0 | 0.89 | 0.90 / 1.96 / 3.61 |
| static | NMPC, no prediction | 2/2 | 52.2 | 7.3 / 19.9 | 9.3 | 4.7 | 0 | 0 | 0.45 | 0.89 / 1.98 / 3.23 |
| static | Nav2 MPPI | 2/2 | 51.2 | 9.9 / 21.2 | 11.6 | -7.1 | 1 | 1 | 0.24 | - / - / - |
| static | Nav2 RPP | 2/2 | 65.1 | 2.1 / 4.2 | 1.5 | -7.0 | 2 | 2 | 0.04 | - / - / - |
| crossing | NMPC + prediction | 2/2 | 56.8 | 1.9 / 9.2 | 5.2 | 11.0 | 0 | 6 | 0.68 | 0.84 / 1.84 / 2.66 |
| crossing | NMPC, no prediction | 2/2 | 55.8 | 1.2 / 6.8 | 33.3 | -9.2 | 2 | 2 | 0.49 | 0.83 / 1.87 / 3.73 |
| crossing | Nav2 MPPI | 2/2 | 48.4 | 8.1 / 19.6 | 11.6 | -21.8 | 2 | 0 | 0.36 | - / - / - |
| crossing | Nav2 RPP | 2/2 | 65.8 | 2.3 / 5.0 | 32.5 | -7.3 | 2 | 2 | 0.13 | - / - / - |
| crossing_fast | NMPC + prediction | 2/2 | 56.2 | 0.8 / 4.3 | 35.1 | -8.5 | 2 | 3 | 0.50 | 0.86 / 2.05 / 10.33 |
| crossing_fast | NMPC, no prediction | 2/2 | 54.3 | 0.5 / 2.3 | 7.3 | -20.5 | 2 | 1 | 0.43 | 0.81 / 1.74 / 11.63 |
| crossing_fast | Nav2 MPPI | 2/2 | 47.6 | 8.6 / 19.2 | 9.8 | -25.0 | 2 | 0 | 0.34 | - / - / - |
| crossing_fast | Nav2 RPP | 2/2 | 66.2 | 2.1 / 4.2 | 38.1 | -15.7 | 2 | 2 | 0.08 | - / - / - |

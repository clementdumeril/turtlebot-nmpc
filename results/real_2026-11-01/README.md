# Real TurtleBot3 runs, 1 November 2026

Lab TurtleBot3 Burger "apple" (C106A, ROS 2 Humble), controller on the lab workstation. 0.12 m/s cap. No ground truth: errors are in the odometry frame, clearance is the smallest lidar range.

| Run | Path | Controller | Time [s] | Tracking RMSE / max, odom [cm] | Smallest lidar range [m] |
|---|---|---|---|---|---|
| circle_clear_nmpc_1 | circle r = 0.5 m, 2 laps | NMPC | 75.4 | 1.9 / 8.6 | 0.52 |
| circle_clear_nmpc_2 | circle r = 0.5 m, 2 laps | NMPC | 70.0 | 1.0 / 3.4 | 0.80 |
| circle_obstacle_nmpc_1 | same + box | NMPC | 57.2 | 16.2 / 34.4 | 0.16 |
| circle_object__mppi_1 | same + box | Nav2 MPPI | 97.4 | 29.4 / 49.3 | 0.18 |
| clear_nmpc_1 | figure eight a = 0.5 m | NMPC | 68.3 | 6.1 / 11.2 | 0.21 |

Solve time 1.2 to 1.5 ms median, under 3 ms max, on the lab workstation.

The box runs are not a valid comparison yet. MPPI got 2.5 to 4 m path segments on a 3.14 m circle, so each segment overlapped itself and it cut through the middle; segments are now capped at 0.6 lap. The NMPC wandered inside the circle, probably because its progress search reached 0.8 m ahead and snapped to another part of the small circle; it is now 0.3 m, which I could not check on the robot yet. `clear_nmpc_1` was overwritten by a later lap: the first lap (1.6 cm RMSE) is lost.

Figure: `python3 scripts/plot_real.py results/real_2026-11-01 docs/figures/real_circle_box.png circle_clear_nmpc_2 circle_obstacle_nmpc_1 circle_object__mppi_1`

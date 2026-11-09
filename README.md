# TurtleBot NMPC: lidar obstacle prediction against Nav2 MPPI

<img src="docs/figures/crossing_nmpc_vs_mppi.gif" width="70%" alt="Webots, simulated pedestrian crossing at 0.5 m/s: the NMPC passes behind it, Nav2 MPPI is hit">

A model predictive controller for a TurtleBot3 that tracks moving obstacles with its 2D lidar, predicts where they will be over the next 3 s and plans around them.

- In Webots, a simulated pedestrian crossing at 0.5 m/s hit it in 1 of 7 runs, against 3 of 3 for Nav2's MPPI controller.
- The acados solver takes about 1 ms (median) per 100 ms control step.
- On the lab TurtleBot3 it tracks a 0.5 m circle to 1 to 2 cm RMS (odometry, 2 runs of 2 laps) and drives a 5 min 30 s autonomous exploration.

[Results](docs/RESULTS.md) · [Formulation](docs/FORMULATION.md) · [Exploration](docs/EXPLORATION.md) · [Checks and reproduction](docs/REPRODUCING.md) · [Bug notes](docs/DEVELOPMENT_NOTES.md)

## How it works

![Overview: lidar and reference in, perception and Kalman tracker, NMPC on predicted circles, TurtleBot3; results in Webots and on the lab robot](docs/figures/overview.png)

- Each lidar scan is cut into circles; each circle is tracked by a constant-velocity Kalman filter.
- Every track is moved to the time of each of the 30 MPC stages (0 to 3 s), with a radius that grows with its velocity uncertainty.
- acados solves a unicycle NMPC (RK4, SQP-RTI) every 0.1 s, with soft clearance constraints to the 6 most relevant predicted circles.
- `predict:=false` freezes every obstacle where it was last seen: that is the ablation, and what a costmap controller sees.
- Baselines: Nav2 MPPI and Regulated Pure Pursuit on the same path, costmap and speed caps.

## Run it

Tested on Ubuntu 24.04 (WSL2), ROS 2 Jazzy, Webots R2025a, acados v0.5.5, casadi 3.7.2; the lab runs used ROS 2 Humble.

```bash
bash scripts/install_nav2_userspace.sh   # or apt install the ros-jazzy-nav2-* controllers
bash scripts/setup_ws.sh && source scripts/env.sh
python3 -m pytest src/tb_nmpc/test src/tb_explore/test   # 21 tests, no ROS needed
python3 scripts/offline_check.py                         # NMPC + tracker without ROS, ~10 s
bash scripts/sim.sh controller:=nmpc scenario:=crossing  # then: ros2 service call /tb_nmpc/start std_srvs/srv/Trigger
```

Controllers: `nmpc`, `nmpc_nopred`, `mppi`, `rpp`. Scenarios: `clear`, `static`, `crossing`, `crossing_fast`. Real robot, benchmark and figure commands: [docs/REPRODUCING.md](docs/REPRODUCING.md).

## Limitations

- Contact rates come from simulation, 2 to 7 runs per cell; on the real robot only tracking is measured, in the odometry frame.
- The simulated pedestrian is a 0.15 m cylinder moving straight at constant speed.
- At 1.0 m/s every controller is hit: 5 Hz lidar, three scans to confirm a velocity, 0.20 m/s top speed.
- Nav2 runs without a planner or velocity smoother, with default MPPI parameters.

## Credits and license

[acados](https://github.com/acados/acados) and [CasADi](https://web.casadi.org) for the solver; [Nav2](https://github.com/ros-navigation/navigation2) for MPPI, RPP and the costmap; [webots_ros2](https://github.com/cyberbotics/webots_ros2) for the TurtleBot3 driver configuration and robot model (Apache-2.0, notices in [`tb_nmpc_sim/resource`](src/tb_nmpc_sim/resource)); [cartographer](https://github.com/cartographer-project/cartographer_ros) for the map. The lab TurtleBots were made available by UC Berkeley EECS C106A. The photo reconstructions in the exploration notes use my course team's code, which is not in this repository.

MIT license ([LICENSE](LICENSE)). Citation metadata: [CITATION.cff](CITATION.cff).

# Checks and reproduction

How the numbers in the [README](../README.md) were checked, and the command behind each result and figure.

## How I checked it

- 21 unit tests: 13 for perception, tracker and reference ([`tb_nmpc/test`](../src/tb_nmpc/test)), 8 for the explorer ([`tb_explore/test`](../src/tb_explore/test)).
- [`offline_check.py`](../scripts/offline_check.py): NMPC and tracker on a plain unicycle, no ROS. Over 5 runs each, minimum clearance 23.3 to 26.1 cm with prediction and -20.8 to -17.9 cm without, the same ordering as in Webots ([log](../results/offline_check_2026-11-03/README.md)).
- Tracker against the Webots ground truth, person at 0.5 m/s: 8 to 16 cm position and 0.06 to 0.11 m/s velocity error (median per run).
- Baselines: Nav2 Jazzy MPPI defaults and default inflation, one costmap for MPPI and RPP; only MPPI's goal-critic thresholds changed, 1.4 to 0.6 m ([`nav2_mppi.yaml`](../src/tb_nmpc/config/nav2_mppi.yaml)).
- The seven NMPC runs at 0.5 m/s: 2 benchmark runs, the video run and 4 repeats on 31 October ([repeats](../results/sim_2026-10-31_crossing_repeats/README.md)); 5 runs of an intermediate version with a clock bug are kept apart.
- Bugs that changed the numbers while I built it: [DEVELOPMENT_NOTES.md](DEVELOPMENT_NOTES.md).

## Simulation details

- MPPI has no target speed and drives at its cap: it laps fastest (45.7 s against 53.0 s) and cuts the lobes (8.6 cm tracking RMSE on the clear lap).
- At 1.0 m/s prediction reduces the contact depth (-8.5 against -20.5 cm worst case) but does not avoid contact.
- NMPC solve time on a 2016 laptop CPU (i7-6700HQ), against a 100 ms period: about 1 ms median with the final code (1.03 ms over the 4 runs in [nmpc_final](../results/sim_2026-10-31_crossing_repeats/nmpc_final/summary.md), 0.89 to 1.23 ms per run); 0.79 to 0.91 ms per run in the 30 October benchmark; worst 12.9 ms.

Per-scenario figures: [`docs/figures/paths_*.png`](figures) (true paths) and [`clearance_*.png`](figures) (clearance over time).

## Lab robot details

Lab TurtleBot3 Burger, ROS 2 Humble, controller on the lab workstation, 0.12 m/s cap. Solve time 1.2 to 1.5 ms median, under 3 ms max. The runs with a box on the circle are not a valid comparison yet: both path setups had a bug ([run notes](../results/real_2026-11-01/README.md)).

![Real TurtleBot3 on the r = 0.5 m circle: path coloured by speed and cross-track error over time](figures/real_laps.png)

## Commands

Simulation setup: acados in `~/acados`; with Webots on Windows and ROS in WSL2, the Windows firewall must let WSL reach Webots on TCP 1234.

| Output | Command |
|---|---|
| `results/sim_2026-10-30/` (32 runs) | `bash scripts/run_benchmark.sh results/sim_2026-10-30` (about 80 min) |
| `summary.md` from existing logs | `python3 scripts/metrics.py results/sim_2026-10-30` |
| `docs/figures/paths_*.png`, `clearance_*.png` | `python3 scripts/plot_benchmark.py results/sim_2026-10-30 docs/figures` |
| real-robot circle table | `python3 scripts/metrics_real.py results/real_2026-11-01 --path-shape circle --path-size 0.5` |
| exploration animation | `python3 scripts/bag_animation.py <decompressed bag> docs/figures/real_exploration_run3.gif` |
| `docs/figures/overview.png` | hand-written `overview.svg`, rendered at 2x with headless Chrome |

## On a TurtleBot3

The controller runs on a Ubuntu machine on the robot's network (WSL2 behind NAT does not see the robot) or on the robot itself. Everything installs in user space:

```bash
git clone https://github.com/clementdumeril/turtlebot-nmpc.git ~/turtlebot-nmpc
git clone --branch v0.5.5 --depth 1 --recurse-submodules --shallow-submodules https://github.com/acados/acados.git ~/acados
cd ~/acados && mkdir -p build && cd build && cmake -DACADOS_WITH_QPOASES=OFF -DACADOS_INSTALL_DIR=$HOME/acados .. && make install -j4
pip install --user casadi && pip install --user -e ~/acados/interfaces/acados_template
cd ~/turtlebot-nmpc && bash scripts/setup_ws.sh && source scripts/env.sh
python3 scripts/offline_check.py      # generates and compiles the solver
```

On the robot: `export TURTLEBOT3_MODEL=burger && ros2 launch turtlebot3_bringup robot.launch.py`. Check the robot's `ROS_DOMAIN_ID`, whether `/cmd_vel` is `Twist` or `TwistStamped`, and the lidar frame (`ros2 run tf2_ros tf2_echo base_link <scan frame>`).

```bash
# controller (one of)
ros2 launch tb_nmpc nmpc.launch.py robot_model:=burger max_speed:=0.12 path_size:=0.5 path_shape:=circle
ros2 launch tb_nmpc nav2_controller.launch.py controller:=mppi robot_model:=burger max_speed:=0.12 path_size:=0.5 path_shape:=circle cmd_vel_type:=twist
# starts the lap and logs it like the simulation
python3 scripts/benchmark.py --real --controller nmpc --cmd-type twist --out results/<date>/<run name>
python3 scripts/metrics_real.py results/<date> --path-shape circle --path-size 0.5
```

The figure eight starts at the robot, heading forward, and lies behind it and to its right: with half-width a = `path_size` it spans 1.71 a behind to 0.29 a ahead and 0 to a to the right. `python3 scripts/step_test.py --cmd-type twist --v 0.10 --w 0.8` measures the command-to-odometry delay; if the NMPC oscillates, set `latency:=<delay + tau/2>`.

Safety: `max_speed` is clamped below the hardware limit and the NMPC never reverses. It sends zero and ends the run if odometry is older than 0.5 s or the lidar older than 1 s, and stops if a lidar return is within 3 cm of the body ahead. Ctrl-C sends five zero commands; `ros2 service call /tb_nmpc/stop std_srvs/srv/Trigger` stops a run. A hard kill (`kill -9`) sends nothing, so keep a hand near the power switch.

| Symptom | Check |
|---|---|
| No topics | `ROS_DOMAIN_ID`, same network, `ROS_LOCALHOST_ONLY` unset |
| Robot does not move | `cmd_vel_type`, namespace, `ros2 topic echo /cmd_vel` |
| Solver build fails | `ACADOS_SOURCE_DIR`, `~/acados/bin/t_renderer` present, run `offline_check.py` alone |
| Obstacles in the wrong place (`/tb_nmpc/obstacles` in RViz) | lidar frame and yaw from TF, `laser_yaw` fallback |
| Legs seen as two obstacles | `leg_merge_dist` (default 0.35 m) |

## Code structure

| Path | Role |
|---|---|
| [`ocp.py`](../src/tb_nmpc/tb_nmpc/ocp.py) | the optimal control problem, in acados ([formulation](FORMULATION.md)) |
| [`perception.py`](../src/tb_nmpc/tb_nmpc/perception.py), [`tracker.py`](../src/tb_nmpc/tb_nmpc/tracker.py) | scan to circles; Kalman tracks and their prediction |
| [`nmpc_node.py`](../src/tb_nmpc/tb_nmpc/nmpc_node.py), [`reference.py`](../src/tb_nmpc/tb_nmpc/reference.py) | ROS 2 node, watchdog, safety stop; paths and horizon sampling |
| [`nav2_follow.py`](../src/tb_nmpc/tb_nmpc/nav2_follow.py), [`config/`](../src/tb_nmpc/config) | Nav2 baselines on the same path; robot limits |
| [`tb_nmpc_sim`](../src/tb_nmpc_sim), [`tb_explore`](../src/tb_explore) | Webots worlds and simulated pedestrian; frontier exploration |
| [`scripts/`](../scripts) | benchmark, metrics, figures, offline check |

The code is mine except the TurtleBot3 driver configuration, adapted from `webots_ros2_turtlebot`. A personal project started on 30 October 2026. Related work read before starting: G. Turrisi's acados NMPC for a differential-drive robot in ROS 2, and MPC with control barrier functions on TurtleBots with moving obstacles.

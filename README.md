# TurtleBot NMPC with lidar obstacle prediction, against Nav2 MPPI

An acados NMPC for a TurtleBot3 that tracks moving obstacles with the 2D lidar and plans around where they will be over the next 3 s. In Webots it was the only one of four controllers that never touched a person crossing its path at 0.5 m/s (2 of 2 runs, 11 cm closest), with a 0.85 ms median solve. Nav2's MPPI made contact in both runs. Real-robot runs are planned for 31 October 2026.

![Top view drawn from the logged Webots ground truth, person crossing at 0.5 m/s: the NMPC slows and passes behind them, Nav2 MPPI keeps going and is hit](docs/figures/crossing_nmpc_vs_mppi.gif)

A personal project started on 30 October 2026, while I have access to the EECS C106A lab TurtleBots at UC Berkeley. My other control projects ([quadrotor NMPC](https://github.com/clementdumeril/quadrotor-ros2), UR5e, fixed-wing RL) only run in simulation. The question here: if the MPC predicts where a person walking across the path will be over its 3 s horizon, does it avoid them better than Nav2's MPPI controller, which only sees the current costmap? Same robot limits, same path, same scenarios for every controller.

## Status

- [x] NMPC, lidar tracking and prediction, ROS 2 node (Jazzy, also written for Humble)
- [x] Webots benchmark against Nav2 MPPI and Regulated Pure Pursuit, 4 scenarios x 2 runs
- [ ] Real TurtleBot, planned 31 October 2026 ([checklist](docs/REAL_ROBOT_CHECKLIST.md)). Every number below is from simulation.

## Results

Simulation only: Webots, TurtleBot3 Burger, lidar at 5 Hz, two runs per case, same speed caps (0.20 m/s, 2.0 rad/s) for every controller. Contact is counted from the Webots ground truth; clearance is edge to edge, negative means contact. From [results/sim_2026-10-30](results/sim_2026-10-30) ([summary](results/sim_2026-10-30/summary.md), [notes](results/sim_2026-10-30/README.md)).

| | NMPC + prediction | NMPC, no prediction | Nav2 MPPI | Nav2 RPP |
|---|---|---|---|---|
| Person crossing at 0.5 m/s: contact, closest | **0/2, +11.0 cm** | 2/2, -9.2 cm | 2/2, -21.8 cm | 2/2, -7.3 cm |
| Person crossing at 1.0 m/s | 2/2, -8.5 cm | 2/2, -20.5 cm | 2/2, -25.0 cm | 2/2, -15.7 cm |
| Two static obstacles near the path | 0/2, +4.0 cm | 0/2, +4.7 cm | 1/2, -7.1 cm | 2/2, -7.0 cm |
| Clear lap: tracking RMSE (odometry frame), lap time | 0.2 cm, 53.0 s | 0.2 cm, 52.9 s | 8.6 cm, 45.7 s | 2.1 cm, 64.3 s |

![True robot paths of the four controllers with the person's crossing line](docs/figures/paths_crossing.png)

![Distance to the person over time, 0.5 m/s crossing](docs/figures/clearance_crossing.png)

![Webots view: the TurtleBot waits and turns behind the person crossing at 0.5 m/s](docs/figures/webots_nmpc_crossing.gif)

The Webots view above is a third NMPC run, recorded separately and not in the table ([log](results/sim_2026-10-30_video/crossing_nmpc_recorded)): 24 cm closest.

At 0.5 m/s the predictive NMPC slows down and passes behind the person. Without prediction, the same NMPC sees them standing where they were and is hit from the side, as are MPPI and RPP, which react to the current costmap only. At 1.0 m/s every controller is hit: the tracker needs three 5 Hz scans to trust a velocity, which leaves about one second at 0.2 m/s. Prediction cuts the depth of contact from 20.5 to 8.5 cm there but does not avoid it.

NMPC solve time over the 16 NMPC runs: 0.79 to 0.91 ms median per run, worst 11.7 ms, on a 2016 laptop CPU (i7-6700HQ) against a 100 ms period. The whole Python control step around it takes 3.4 ms median.

MPPI has no target speed and drives at its cap, which is part of why it laps fastest and cuts the lobes (8.6 cm). RPP tracks well but has no avoidance and touches both static obstacles.

## What I built

All of it, except the upstream pieces listed in Credits.

- [`ocp.py`](src/tb_nmpc/tb_nmpc/ocp.py): the NMPC in acados. Unicycle kinematics with the velocities as states (x, y, θ, v, ω; inputs a, α), so acceleration limits are hard constraints. 30 stages of 0.1 s, nonlinear least squares on position, heading (as cos/sin), speeds and accelerations, SQP-RTI with HPIPM.
- [`perception.py`](src/tb_nmpc/tb_nmpc/perception.py): /scan to circles. Points split where consecutive beams jump, small clusters become one circle each, long ones (walls) are cut into static 0.3 m pieces.
- [`tracker.py`](src/tb_nmpc/tb_nmpc/tracker.py): constant-velocity Kalman filter per obstacle with nearest-neighbour gating, and the prediction over the horizon, whose radius grows with the velocity uncertainty.
- [`nmpc_node.py`](src/tb_nmpc/tb_nmpc/nmpc_node.py): the ROS 2 node. /scan and /odom in, /cmd_vel out (Twist or TwistStamped, detected from the subscriber), the lidar pose from TF, robot files for the [Burger](src/tb_nmpc/config/burger.yaml) and the [TurtleBot4](src/tb_nmpc/config/tb4.yaml), a safety stop, RViz markers.
- [`nav2_follow.py`](src/tb_nmpc/tb_nmpc/nav2_follow.py) and [`nav2_controller.launch.py`](src/tb_nmpc/launch/nav2_controller.launch.py): the baselines. Nav2's controller_server with [MPPI](src/tb_nmpc/config/nav2_mppi.yaml) or [RPP](src/tb_nmpc/config/nav2_rpp.yaml) and a local costmap, fed the same figure eight.
- [`tb_nmpc_sim`](src/tb_nmpc_sim): the Webots world, the [scenarios](src/tb_nmpc_sim/tb_nmpc_sim/scenarios.py), and a supervisor plugin that walks the pedestrian across the path and publishes the ground truth.
- [`scripts/`](scripts): benchmark runner, recorder, metrics, figures, GIF, an offline closed-loop check without ROS, and a step test for the real base.

## How it works

```
/scan --> clusters --> circles --> Kalman tracks --> predicted circles at t_k = k dt --> NMPC constraints
/odom ------------------------------------------------------------------------------> x0, path progress
```

Each stage k of the horizon gets the M = 6 most relevant obstacles as parameters, at their predicted position at time k·dt. The constraint is

  sqrt((x_k − o_x,k)² + (y_k − o_y,k)²) ≥ r_obs + σ_v·t_k + r_robot + 5 cm,

softened with an L1 + L2 slack so the QP stays feasible when someone steps next to the robot. With `predict:=false` the same obstacles are frozen where they are now: that is the ablation, and also what a costmap-based controller like MPPI sees.

The reference advances from the robot's projection on the path at the nominal speed, so the NMPC follows the path rather than a clock and does not rush after a detour. Reference points that fall inside a static obstacle are pushed past it along the path; without that, an obstacle sitting on the path holds the whole 0.5 m reference inside its radius and stopping in front of it is optimal.

## How I checked it

- 11 unit tests ([`test/`](src/tb_nmpc/test)): circle fitting on synthetic scans with the Webots beam order, a wall cut into static pieces, the Kalman velocity of a target at 0.5 m/s, the path projection through the self-crossing, the reference skip.
- [`offline_check.py`](scripts/offline_check.py): the NMPC and the tracker in a plain unicycle simulation with a person crossing, no ROS. With prediction: 23.5 cm clearance, 0.5 cm RMSE; without: contact (-20.6 cm). The same ordering as in Webots.
- Perception against the Webots ground truth while the person walks: 8 to 16 cm position error and 0.06 to 0.11 m/s velocity error at 0.5 m/s, about twice that at 1.0 m/s.
- Fair baselines: same caps, the Nav2 Jazzy MPPI defaults and the Nav2 default inflation (0.70 m, scaling 3.0), same costmap for MPPI and RPP. The one change to MPPI is the goal and path-follow critic thresholds (1.4 to 0.6 m), explained below.
- Tracking error is reported in the odometry frame (what each controller sees) and against the ground truth. On the clear lap the NMPC tracks its odometry to 0.2 cm but is 3.2 cm off in truth: the rest is odometry drift of the Webots TurtleBot. After a contact the robot is pushed, its wheels slip, and the true error jumps above 30 cm.

Things that changed the numbers while I built it:

- The soft obstacle constraints had no penalty on their upper slack. The QP often hit its 50-iteration cap (6 ms median per solve); with the penalty it converges in about 12 iterations (1 ms).
- MPPI first took 100 s for a lap the NMPC does in 53 s, creeping at 0.01 m/s for a minute. The path reaches it in segments (a closed loop sent whole ends where it starts), and on a figure eight a segment end could be within 1.4 m of the robot, which switches on the goal critics. Segments now end at their farthest point (at least 1.34 m away) and the thresholds are 0.6 m.
- With the cylinder exactly on the path at a lobe tip, both my NMPC and MPPI stopped in front of it for good: the whole reference sat inside the inflated obstacle. I added the reference skip to the NMPC, and moved the cylinder 18 cm off the path so the test is about local avoidance, which is what both controllers do without a global planner.
- In two of my first starts, ros2_control read the placeholder robot description before the Webots driver replaced it, and the wheels never came up. The placeholder now carries the ros2_control block.
- The box was first scored as its circumscribed circle, which counts contacts that are not there. Clearance now uses the true box.

## Limitations

- Simulation only until the real-robot runs. Two runs per case, one arena, one path.
- The simulated person is a 0.15 m cylinder walking straight at constant speed, who does not yield. A real person's legs show up as two small clusters that swap places, merged here when closer than 0.35 m.
- At 1.0 m/s nobody avoids the person, including the NMPC: 5 Hz lidar, three scans to confirm a track, 0.2 m/s top speed.
- Nav2 runs without a planner or velocity smoother, fed the path by my node. A full Nav2 stack, or MPPI tuned for this robot, could do better than the defaults used here.
- When it avoids, the NMPC turns harder than MPPI (RMS angular acceleration 0.68 vs 0.36 rad/s² at 0.5 m/s), and it stops briefly to let the person pass.

## Reproduce

WSL2 (Ubuntu 24.04) with ROS 2 Jazzy, `ros-jazzy-webots-ros2`, Webots R2025a on Windows, acados v0.5.5 in `~/acados` and casadi (`pip install casadi`, a venv is fine). For the baselines, Nav2:

```bash
sudo apt install ros-jazzy-nav2-controller ros-jazzy-nav2-mppi-controller ros-jazzy-nav2-regulated-pure-pursuit-controller ros-jazzy-nav2-costmap-2d ros-jazzy-nav2-lifecycle-manager
# or, without sudo:
bash scripts/install_nav2_userspace.sh
```

```bash
bash scripts/setup_ws.sh                  # ~/tb_ws, links src/, builds
source scripts/env.sh
python3 scripts/offline_check.py          # NMPC closed loop without ROS, 10 s
python3 -m pytest src/tb_nmpc/test        # 11 tests
bash scripts/sim.sh controller:=nmpc scenario:=crossing      # then: ros2 service call /tb_nmpc/start std_srvs/srv/Trigger
bash scripts/run_benchmark.sh results/sim_2026-10-30         # every scenario and controller, about 80 min
python3 scripts/plot_benchmark.py results/sim_2026-10-30 docs/figures
python3 scripts/make_gif.py results/sim_2026-10-30 crossing docs/figures/crossing_nmpc_vs_mppi.gif
# Webots video of one run (file on the Windows side), then cut to a GIF with Webots' ffmpeg:
python3 scripts/benchmark.py --controller nmpc --scenario crossing --out /tmp/rec --record C:/Users/<you>/Videos/run.mp4
```

The Windows firewall must let WSL reach Webots on TCP 1234. Real robot: [docs/REAL_ROBOT_CHECKLIST.md](docs/REAL_ROBOT_CHECKLIST.md).

## Credits

- [acados](https://github.com/acados/acados) (BSD-2) and [CasADi](https://web.casadi.org) (LGPL) for the solver.
- [Nav2](https://github.com/ros-navigation/navigation2) (Apache-2.0): MPPI controller, Regulated Pure Pursuit, costmap. MPPI parameters start from the Nav2 Jazzy defaults.
- [webots_ros2](https://github.com/cyberbotics/webots_ros2) (Apache-2.0): the TurtleBot3 Burger driver configuration (`ros2control.yml`, URDF plugin block) is adapted from `webots_ros2_turtlebot`. The robot model is Cyberbotics' TurtleBot3Burger PROTO.
- Related work I read before starting: G. Turrisi's acados NMPC for a differential-drive robot in ROS 2, and MPC with control barrier functions on TurtleBots with moving obstacles, both in simulation.

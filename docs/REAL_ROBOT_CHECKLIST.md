# Real TurtleBot: Saturday checklist

Goal of the session: the same figure eight and the same crossing on the real robot, NMPC vs. Nav2 MPPI (and RPP), logged the same way as in simulation. Everything below runs without sudo.

## 0. Before leaving (Friday evening)

- [ ] Where the controller runs. WSL2 on Windows 10 is behind NAT: ROS 2 discovery with a robot on the lab network does not work from it. Use, in this order:
  1. a lab Ubuntu workstation on the robot's network;
  2. the robot's own computer (TurtleBot3: Raspberry Pi; TurtleBot4: Raspberry Pi 4), over ssh;
  3. a laptop running Ubuntu natively.
- [ ] On that machine (user space, about 10 min):
  ```bash
  git clone https://github.com/clementdumeril/turtlebot-nmpc.git ~/turtlebot-nmpc
  git clone --branch v0.5.5 --depth 1 --recurse-submodules --shallow-submodules https://github.com/acados/acados.git ~/acados
  cd ~/acados && mkdir -p build && cd build && cmake -DACADOS_WITH_QPOASES=OFF -DACADOS_INSTALL_DIR=$HOME/acados .. && make install -j4
  pip install --user casadi && pip install --user -e ~/acados/interfaces/acados_template
  cd ~/turtlebot-nmpc && bash scripts/setup_ws.sh && source scripts/env.sh
  python3 scripts/offline_check.py      # generates and compiles the solver (needs internet the first time for t_renderer)
  ```
  On Humble, `source /opt/ros/humble/setup.bash` first; `scripts/env.sh` uses `$ROS_DISTRO`. Nav2 (MPPI, RPP) is usually installed on lab machines: check `ros2 pkg list | grep nav2_mppi`.
- [ ] Print this page, bring tape (path start, crossing line, trigger mark), a tape measure and a cardboard box.

## 1. Identify the robot (5 min)

```bash
printenv ROS_DISTRO
export ROS_DOMAIN_ID=<number on the robot>        # TurtleBot4 may use a discovery server: ask the TA
ros2 topic list                                    # namespaced? (/tb4_xx/cmd_vel): pass the topics below
ros2 topic info -v /cmd_vel | grep -i type         # Twist or TwistStamped
ros2 topic hz /scan                                # TB3 LDS: ~5 Hz, TB4 RPLIDAR: ~7 Hz
ros2 topic echo --once /scan --field header.frame_id
ros2 run tf2_ros tf2_echo base_link <that frame>   # lidar mounting, TB4 is rotated
```

TurtleBot3 bring-up, on the robot: `export TURTLEBOT3_MODEL=burger && ros2 launch turtlebot3_bringup robot.launch.py`. TurtleBot4 starts its bring-up at boot.

Write down: distro, model, cmd_vel type, scan rate, lidar frame and yaw.

## 2. Safety

- First runs with `max_speed:=0.10` and `path_size:=0.8` (a 1.6 m x 0.8 m figure eight), then 0.15, then the full 0.20 (Burger) / 0.25 (TB4).
- Clear 3 m x 2 m, nobody inside except the "pedestrian".
- One person on the keyboard with this ready: `ros2 service call /tb_nmpc/stop std_srvs/srv/Trigger` (or `/nav2_follow/stop`). Ctrl-C on the node also sends a zero command.
- Check once, wheels in the air: kill the node while it drives. Does the base stop by itself? (TurtleBot4's Create 3 has a command timeout; I have not checked the TurtleBot3 firmware.) If it does not, keep a hand near the power switch.
- The NMPC has a safety stop (lidar return within 3 cm of the body, ahead). Do not rely on it at speed.

## 3. Identify the base (5 min)

```bash
python3 scripts/step_test.py --cmd-type twist --v 0.10 --w 0.8 --out step_burger.csv   # or --cmd-type stamped
```

It prints the command-to-odometry delay and time constant. If the NMPC oscillates, set `latency:=<delay + tau/2>` in the launch.

## 4. Runs

Lay out: start mark (robot at the origin, facing +x, i.e. along the top of the right lobe), the crossing line on the first diagonal (s = 2.9 m for `path_size:=1.5`; scale with the size), and a trigger mark on the floor where the walker starts walking when the robot passes it.

Terminal A (controller), terminal B (recorder), terminal C (bag):

```bash
# A: one of
ros2 launch tb_nmpc nmpc.launch.py robot_model:=burger max_speed:=0.15 path_size:=1.0
ros2 launch tb_nmpc nmpc.launch.py robot_model:=burger max_speed:=0.15 path_size:=1.0 predict:=false
ros2 launch tb_nmpc nav2_controller.launch.py controller:=mppi robot_model:=burger max_speed:=0.15 path_size:=1.0 cmd_vel_type:=twist
ros2 launch tb_nmpc nav2_controller.launch.py controller:=rpp  robot_model:=burger max_speed:=0.15 path_size:=1.0 cmd_vel_type:=twist
# C:
ros2 bag record -o bags/<name> /scan /odom /cmd_vel /tf /tf_static /tb_nmpc/stats /tb_nmpc/tracks /nav2_follow/stats
# B: starts the lap and logs it like the simulation (no ground truth on the real robot)
python3 scripts/benchmark.py --real --controller nmpc --cmd-type twist --out results/real_2026-10-31/<name>
```

| # | Scenario | Controllers | Repeats |
|---|---|---|---|
| 1 | clear, 0.10 m/s, small figure | nmpc | 1 (shake-down) |
| 2 | clear | nmpc, mppi, rpp | 3 each |
| 3 | box on the path | nmpc, mppi, rpp | 3 each |
| 4 | person crossing at a steady walk, same trigger mark | nmpc, nmpc no prediction, mppi | 3 each |

Name runs `<scenario>_<controller>_<repeat>`. Film runs 2 to 4 from above (chair or stairs, phone in landscape, whole figure in frame).

## 5. What to bring back

- `results/real_2026-10-31/` (run.csv, meta.json), `bags/`, `step_*.csv`, the videos.
- Notes: robot model and distro, where the controller ran, floor type, anything that went wrong.
- On the real robot there is no ground truth: tracking error is in the odometry frame, and clearance must come from the bag (minimum lidar range) or the video.

## 6. If something does not work

| Symptom | Check |
|---|---|
| No topics | `ROS_DOMAIN_ID`, same network, `ROS_LOCALHOST_ONLY` unset, discovery server on TB4 |
| Robot does not move | cmd_vel type (`cmd_vel_type:=twist`), namespace, `ros2 topic echo /cmd_vel` |
| Solver build fails | `ACADOS_SOURCE_DIR`, `~/acados/bin/t_renderer` present, run `offline_check.py` alone |
| Obstacles in the wrong place in RViz (`/tb_nmpc/obstacles`) | lidar frame and yaw from TF, `laser_yaw` fallback |
| Legs seen as two obstacles | `leg_merge_dist` (default 0.35 m) |
| NMPC oscillates | `latency`, or lower `max_speed` |
| Nav2 does not start | `ros2 lifecycle get /controller_server`, odom -> base_link TF exists |

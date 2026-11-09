# Autonomous exploration and photo capture

The TurtleBot maps a room it does not know and takes posed photos for a 3D reconstruction by itself.

![Offline check: explored map, robot path and photo spots](figures/explore_offline.png)

## How it works

```
/scan, /odom --> cartographer (or simple_mapper) --> /map + TF map -> odom
/map --> explorer: frontiers, Dijkstra on a costmap --> /tb_nmpc/path --> tb_nmpc (NMPC) --> /cmd_vel
explorer --> /auto_capture/capture --> auto_capture: photo + TF map -> base_link --> run_*/img_XXXX.png, poses.csv
```

1. **Spin.** The robot turns in place in 12 steps of 30°. At each step it stops, waits 0.6 s, and asks `auto_capture` for a photo. It turns again only once the photo is saved (`/auto_capture/saved`).
2. **Plan.** On the latest `/map`, frontiers are free cells next to unknown ones, grouped into clusters of at least 8 cells (0.4 m). Dijkstra runs on a costmap: obstacles plus 17 cm are forbidden, and the cost rises within 40 cm of an obstacle, so paths keep to the middle. The chosen frontier minimises path length − 0.5 × frontier length. Frontiers already visited are blacklisted.
3. **Drive.** The robot turns to face the path, then the NMPC follows it (`path_source:=topic`), still avoiding what the lidar sees, and publishes `/follow/done` at the end.
4. Back to 1. With no reachable frontier left on three maps in a row, the robot drives back to its start and stops.

[`auto_capture`](../src/tb_explore/tb_explore/auto_capture.py) writes `run_YYYYmmdd_HHMMSS/img_0000.png …` and `poses.csv` with `file, image_stamp, tf_stamp, tf_source, x, y, z, qx, qy, qz, qw, yaw_deg`.

## Checks

- Unit tests (`src/tb_explore/test`): frontier on a half-known room, costmap, plan to the frontier that never crosses an occupied cell, blacklist, path home, log-odds map.
- [`scripts/explore_offline.py`](../scripts/explore_offline.py): the planner and the NMPC in a simulated 4 m × 3 m room, with an inner wall (0.8 m door) and two boxes, a simulated lidar and the log-odds map. Result: 99 % of the room explored, 6 goals, 8 photo spots, back within 3 cm of the start, never closer than 9 cm to a wall, 292 s of robot time.
- Webots, same room (`world:=room.wbt`), the whole ROS stack with `mapper:=simple`: 3 goals, 4 spins, 46 photos with poses, back home, 205 s. Photos were taken at 30° steps to within 2°. One photo was lost at the very start, before the camera streamed; there is now one retry.
- Real robot, 2 November 2026 ([run](../results/explore_2026-11-02/README.md)): 2 goals, 36 photos with poses, back home in 257 s; bundle adjustment 0.30 px, but only 88 points. Fixed since: frontiers behind cartographer's uncertain band, and photos every 0.3 m while driving (`capture_spacing`).
- Real robot, runs 2 and 3 ([run 2](../results/explore_2026-11-02b/README.md), [run 3](../results/explore_2026-11-02c/README.md)): run 2 mapped 10.8 m x 9.3 m over 19.1 m before the battery ran low; run 3 took 5 min 30 s, drove 13.9 m, mapped 13.2 m x 8.4 m and took 76 photos (33 while driving), with 219 reconstructed points at 0.21 px. 360 degree panoramas are stitched from the spins with the robot's yaw (`scripts/spin_panorama.py`).

![Final map of run 3 with the robot's path, where each photo was taken, and ten of them pinned to their spot](figures/real_photo_map.png)

## Run it on the robot

On the TurtleBot (ssh), two sessions:

```bash
ros2 launch turtlebot3_bringup robot.launch.py
ros2 launch usb_cam usb_cam.launch.py
```

On the workstation, in `distrobox enter ros2` (after `git pull` and `colcon build --symlink-install` in `~/tb_ws`):

```bash
source ~/turtlebot-nmpc/scripts/env.sh
ros2 launch tb_explore explore.launch.py
ros2 service call /explorer/start std_srvs/srv/Trigger      # second terminal
ros2 service call /explorer/stop std_srvs/srv/Trigger       # to stop at any time (or Ctrl-C the launch)
```

On apple the camera topic is `/apple/image_raw`. Useful options: `max_speed:=0.10`, `max_time:=600`, `captures_per_spin:=8`, `image_topic:=/apple/image_raw` (if the camera is namespaced), `capture:=false`, `mapper:=simple` (if cartographer misbehaves), `rviz:=true`. The photos go to `~/explore_captures/run_<date>/`.

Watch in RViz: `/map`, `/explorer/frontiers` (blue points, orange goal), `/tb_nmpc/reference_path`, `/tb_nmpc/obstacles`.

## Reconstruction

The run folder (photos and `poses.csv`) feeds any SIFT + bundle-adjustment pipeline with a calibrated camera; the reconstructions in `results/` used a separate one, not in this repository. Matching every pair of photos grows fast (48 photos means 1128 pairs): start with `captures_per_spin:=8` or a short run. [`scripts/colmap_known_poses.py`](../scripts/colmap_known_poses.py) writes the robot's camera poses as a COLMAP model.

## In Webots

```bash
bash scripts/sim.sh world:=room.wbt controller:=none scenario:=clear
ros2 launch tb_explore explore.launch.py mapper:=simple use_sim_time:=true cmd_vel_type:=stamped \
    image_topic:=/TurtleBot3Burger/camera/image_color autostart:=true
```

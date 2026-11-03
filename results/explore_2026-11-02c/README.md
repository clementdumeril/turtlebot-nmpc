# Exploration run 3, 2 November 2026

Same setup as [run 2](../explore_2026-11-02b/README.md), with photos while driving (`capture_spacing` 0.3 m) and `image_topic:=/apple/image_raw`. Rosbag of 5 min 30 s.

| | Run 1 (spins only) | Run 3 (spins + while driving) |
|---|---|---|
| Photos | 36 | 76 (33 while driving) |
| Map | not saved | 13.2 m x 8.4 m |
| Driven | about 11 m | 13.9 m, 4 frontier goals |
| Reconstructed points (Lab 6 code, no top mask) | 88 | 219 |
| RMS reprojection after bundle adjustment | 0.30 px | 0.21 px |
| Largest camera shift from the SLAM poses | 15 mm | 39 mm |

![The map growing, the robot's path and its goals](../../docs/figures/real_exploration_run3.gif)

![Reconstructed points and photo spots](../../docs/figures/real_reconstruction_run3.png)

The photos taken while driving give the parallax that the spins lack, and the point count went up 2.5 times. The cloud is still sparse: the Lab 6 method keeps only SIFT points matched in several views. 5 of the 76 photos are blurred (variance of the Laplacian under 40).

Files: [run_20261102_170423](run_20261102_170423) (photos and poses.csv in the Lab 6 format), [reconstruction/ba.ply](reconstruction/ba.ply), [explore_20261102_c](explore_20261102_c) (zstd-compressed bag). Animation: `python3 scripts/bag_animation.py <decompressed bag> docs/figures/real_exploration_run3.gif`.

# Bugs that changed the numbers

Found while building and benchmarking the controller. Back to the [README](../README.md).

- The soft obstacle constraints had no penalty on their upper slack, and the QP often hit its 50-iteration cap. With the penalty it converges well before it.
- MPPI first crawled through the figure eight: a path segment could end within 1.4 m of the robot, which switches on the goal critics. Segments now end at their farthest point and the thresholds are 0.6 m.
- With the cylinder exactly on the path, both the NMPC and MPPI stopped in front of it for good. The NMPC's reference now skips past static obstacles, and the cylinder sits 18 cm off the path.
- ros2_control sometimes read the placeholder robot description before the Webots driver replaced it. The placeholder now carries the ros2_control block.
- My first fix for a clock offset between robot and computer dropped the scan's transport delay, and contact at 0.5 m/s went to 3 of 5 runs. The offset is now the minimum of (receive time - stamp) over 10 s.

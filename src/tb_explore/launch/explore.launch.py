"""Autonomous exploration: mapper + NMPC (following planned paths) + explorer + photo capture.

On the robot (C106A lab, Humble), with the bring-up and the camera running on the TurtleBot:
    ros2 launch tb_explore explore.launch.py
Then:  ros2 service call /explorer/start std_srvs/srv/Trigger

In Webots (no cartographer installed there):
    ros2 launch tb_explore explore.launch.py mapper:=simple use_sim_time:=true image_topic:=/camera/image_color

mapper:=cartographer runs the same two nodes as Lab 4's slam_launch.py (turtlebot3_lds_2d.lua).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HARDWARE_MAX = {"burger": 0.22, "tb4": 0.30}


def make_nodes(context):
    get = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    sim = get("use_sim_time") == "true"
    nodes = []
    if get("mapper") == "cartographer":
        carto = get_package_share_directory("turtlebot3_cartographer")
        nodes += [
            # output="log": cartographer prints a warning per dropped scan point, which buried the
            # explorer's own messages in the terminal. Its output stays in ~/.ros/log.
            Node(package="cartographer_ros", executable="cartographer_node", name="cartographer_node", output="log",
                 parameters=[{"use_sim_time": sim}],
                 arguments=["-configuration_directory", os.path.join(carto, "config"),
                            "-configuration_basename", "turtlebot3_lds_2d.lua"]),
            Node(package="cartographer_ros", executable="cartographer_occupancy_grid_node",
                 name="cartographer_occupancy_grid_node", output="log", parameters=[{"use_sim_time": sim}],
                 arguments=["-resolution", "0.05", "-publish_period_sec", "1.0"]),
        ]
    else:
        nodes.append(Node(package="tb_explore", executable="simple_mapper", output="screen",
                          parameters=[{"use_sim_time": sim}]))

    v = min(float(get("max_speed")), HARDWARE_MAX[get("robot_model")] - 0.02)
    nmpc_share = get_package_share_directory("tb_nmpc")
    nodes.append(Node(
        package="tb_nmpc", executable="nmpc_node", name="tb_nmpc", output="screen",
        parameters=[os.path.join(nmpc_share, "config", get("robot_model") + ".yaml"),
                    {"use_sim_time": sim, "path_source": "topic", "v_max": v, "v_ref": 0.9 * v,
                     "cmd_vel_type": get("cmd_vel_type"), "safety_margin": 0.03,
                     "track_v_static": float(get("track_v_static")), "track_min_hits": int(get("track_min_hits"))}]))
    nodes.append(Node(package="tb_explore", executable="explorer", output="screen",
                      parameters=[{"use_sim_time": sim, "cmd_vel_type": get("cmd_vel_type"),
                                   "autostart": get("autostart") == "true",
                                   "max_time": float(get("max_time")),
                                   "captures_per_spin": int(get("captures_per_spin"))}]))
    if get("capture") == "true":
        nodes.append(Node(package="tb_explore", executable="auto_capture", output="screen",
                          parameters=[{"use_sim_time": sim, "image_topic": get("image_topic"),
                                       "output_dir": os.path.expanduser(get("output_dir"))}]))
    if get("rviz") == "true":
        nodes.append(Node(package="rviz2", executable="rviz2", output="screen", parameters=[{"use_sim_time": sim}]))
    return nodes


def generate_launch_description():
    args = [
        ("mapper", "cartographer", "cartographer (robot) or simple (odometry only, Webots)"),
        ("use_sim_time", "false", ""),
        ("robot_model", "burger", "burger or tb4"),
        ("max_speed", "0.12", "[m/s]"),
        ("cmd_vel_type", "auto", "auto, twist or stamped"),
        ("image_topic", "/image_raw", "camera topic (C106A apple: /apple/image_raw, Webots: /TurtleBot3Burger/camera/image_color)"),
        ("output_dir", "~/explore_captures", "a run_YYYYmmdd_HHMMSS folder is created inside"),
        ("capture", "true", "false = explore without photos"),
        ("captures_per_spin", "12", "photos per turn in place (30 degrees apart)"),
        ("max_time", "900", "[s] exploration budget"),
        ("track_v_static", "0.2", ""),
        ("track_min_hits", "4", ""),
        ("autostart", "false", ""),
        ("rviz", "false", ""),
    ]
    return LaunchDescription([DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in args]
                             + [OpaqueFunction(function=make_nodes)])

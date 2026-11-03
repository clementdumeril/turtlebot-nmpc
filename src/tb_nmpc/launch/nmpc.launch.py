"""NMPC node on a real or simulated TurtleBot.

    ros2 launch tb_nmpc nmpc.launch.py robot_model:=burger max_speed:=0.12 path_size:=0.8
Defaults are deliberately slow and small for a first real run (0.12 m/s, 2 m x 1 m).
    ros2 launch tb_nmpc nmpc.launch.py robot_model:=tb4 predict:=false use_sim_time:=false
Then:  ros2 service call /tb_nmpc/start std_srvs/srv/Trigger
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HARDWARE_MAX = {"burger": 0.22, "tb4": 0.30}  # [m/s] max_speed is clamped below these


def make_nodes(context):
    """nmpc_node with config/<robot_model>.yaml, overridden by the launch arguments (speed capped by HARDWARE_MAX)."""
    get = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    share = get_package_share_directory("tb_nmpc")
    overrides = {
        "use_sim_time": get("use_sim_time") == "true",
        "predict": get("predict") == "true",
        "path_size": float(get("path_size")),
        "path_shape": get("path_shape"),
        "track_v_static": float(get("track_v_static")),
        "track_min_hits": int(get("track_min_hits")),
        "laps": float(get("laps")),
        "cmd_vel_topic": get("cmd_vel_topic"),
        "cmd_vel_type": get("cmd_vel_type"),
        "scan_topic": get("scan_topic"),
        "odom_topic": get("odom_topic"),
        "autostart": get("autostart") == "true",
        "latency": float(get("latency")),
        "chair_radius": float(get("chair_radius")),
    }
    if get("max_speed"):
        v = min(float(get("max_speed")), HARDWARE_MAX[get("robot_model")] - 0.02)
        overrides.update(v_max=v, v_ref=0.9 * v)  # reference speed 10 % under the cap
    # later parameter entries win: the overrides replace the values from the yaml
    return [Node(package="tb_nmpc", executable="nmpc_node", name="tb_nmpc", output="screen",
                 parameters=[os.path.join(share, "config", get("robot_model") + ".yaml"), overrides])]


def generate_launch_description():
    """Declare the launch arguments (name, default, help) and build the node in make_nodes."""
    args = [
        ("robot_model", "burger", "burger or tb4"),
        ("use_sim_time", "false", ""),
        ("predict", "true", "false = obstacles frozen at their current position"),
        ("max_speed", "0.12", "speed cap [m/s], below the robot file's v_max; empty = v_max"),
        ("path_size", "1.0", "half-width a of the figure eight, or circle radius [m]; see the checklist"),
        ("path_shape", "eight", "eight or circle (starts heading forward, turns left)"),
        ("track_v_static", "0.10", "[m/s] tracks slower than this are predicted static"),
        ("track_min_hits", "3", "detections before a track's velocity is used"),
        ("laps", "1.0", ""),
        ("cmd_vel_topic", "/cmd_vel", ""),
        ("cmd_vel_type", "auto", "auto, twist or stamped"),
        ("scan_topic", "/scan", ""),
        ("odom_topic", "/odom", ""),
        ("autostart", "false", ""),
        ("latency", "0.0", "[s] forward-predict the initial state"),
        ("chair_radius", "0.0", "[m] > 0: thin lidar clusters (chair columns) get a chair-base radius"),
    ]
    return LaunchDescription([DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in args]
                             + [OpaqueFunction(function=make_nodes)])

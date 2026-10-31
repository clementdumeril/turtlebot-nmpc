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
    get = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    share = get_package_share_directory("tb_nmpc")
    overrides = {
        "use_sim_time": get("use_sim_time") == "true",
        "predict": get("predict") == "true",
        "path_size": float(get("path_size")),
        "laps": float(get("laps")),
        "cmd_vel_topic": get("cmd_vel_topic"),
        "cmd_vel_type": get("cmd_vel_type"),
        "scan_topic": get("scan_topic"),
        "odom_topic": get("odom_topic"),
        "autostart": get("autostart") == "true",
        "latency": float(get("latency")),
    }
    if get("max_speed"):
        v = min(float(get("max_speed")), HARDWARE_MAX[get("robot_model")] - 0.02)
        overrides.update(v_max=v, v_ref=0.9 * v)
    return [Node(package="tb_nmpc", executable="nmpc_node", name="tb_nmpc", output="screen",
                 parameters=[os.path.join(share, "config", get("robot_model") + ".yaml"), overrides])]


def generate_launch_description():
    args = [
        ("robot_model", "burger", "burger or tb4"),
        ("use_sim_time", "false", ""),
        ("predict", "true", "false = obstacles frozen at their current position"),
        ("max_speed", "0.12", "speed cap [m/s], below the robot file's v_max; empty = v_max"),
        ("path_size", "1.0", "half-width a of the figure eight [m]; it covers 2a x a, see the checklist"),
        ("laps", "1.0", ""),
        ("cmd_vel_topic", "/cmd_vel", ""),
        ("cmd_vel_type", "auto", "auto, twist or stamped"),
        ("scan_topic", "/scan", ""),
        ("odom_topic", "/odom", ""),
        ("autostart", "false", ""),
        ("latency", "0.0", "[s] forward-predict the initial state"),
    ]
    return LaunchDescription([DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in args]
                             + [OpaqueFunction(function=make_nodes)])

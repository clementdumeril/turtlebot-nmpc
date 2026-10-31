"""Nav2 controller_server (MPPI or Regulated Pure Pursuit) + the path feeder, no map, no planner.

    ros2 launch tb_nmpc nav2_controller.launch.py controller:=mppi robot_model:=burger use_sim_time:=false
Then:  ros2 service call /nav2_follow/start std_srvs/srv/Trigger
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

ROBOTS = {  # the same caps as config/<robot>.yaml
    "burger": {"robot_radius": "0.105", "vx_max": "0.20", "wz_max": "2.0", "desired_linear_vel": "0.18"},
    "tb4": {"robot_radius": "0.18", "vx_max": "0.25", "wz_max": "1.9", "desired_linear_vel": "0.22"},
}


def make_nodes(context):
    get = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    share = get_package_share_directory("tb_nmpc")
    rewrites = dict(ROBOTS[get("robot_model")])
    if get("max_speed"):
        v = min(float(get("max_speed")), {"burger": 0.20, "tb4": 0.28}[get("robot_model")])
        rewrites.update(vx_max=str(v), desired_linear_vel=str(0.9 * v))
    rewrites["use_sim_time"] = get("use_sim_time")
    rewrites["enable_stamped_cmd_vel"] = "true" if get("cmd_vel_type") == "stamped" else "false"
    rewrites["topic"] = get("scan_topic")
    rewrites["odom_topic"] = get("odom_topic")
    params = RewrittenYaml(source_file=os.path.join(share, "config", f"nav2_{get('controller')}.yaml"),
                           param_rewrites=rewrites, convert_types=True)
    sim = get("use_sim_time") == "true"
    remap = [("cmd_vel", get("cmd_vel_topic"))]
    return [
        Node(package="nav2_controller", executable="controller_server", output="screen",
             parameters=[params], remappings=remap),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_control",
             output="screen", parameters=[{"use_sim_time": sim, "autostart": True, "node_names": ["controller_server"]}]),
        Node(package="tb_nmpc", executable="nav2_follow", output="screen",
             parameters=[{"use_sim_time": sim, "path_size": float(get("path_size")), "laps": float(get("laps")),
                          "odom_topic": get("odom_topic")}]),
    ]


def generate_launch_description():
    args = [
        ("controller", "mppi", "mppi or rpp"),
        ("robot_model", "burger", "burger or tb4"),
        ("use_sim_time", "false", ""),
        ("max_speed", "0.12", "speed cap [m/s]; empty = the robot's cap in ROBOTS"),
        ("path_size", "1.0", "half-width a of the figure eight [m]"),
        ("laps", "1.0", ""),
        ("cmd_vel_topic", "/cmd_vel", ""),
        ("cmd_vel_type", "stamped", "twist or stamped (what the base subscribes to)"),
        ("scan_topic", "/scan", ""),
        ("odom_topic", "/odom", ""),
    ]
    return LaunchDescription([DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in args]
                             + [OpaqueFunction(function=make_nodes)])

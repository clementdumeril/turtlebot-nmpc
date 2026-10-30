"""Webots TurtleBot3 Burger on the figure eight, with one controller.

    ros2 launch tb_nmpc_sim sim.launch.py controller:=nmpc scenario:=crossing
    controller: nmpc | nmpc_nopred | mppi | rpp | none
    scenario:   clear | static | crossing | crossing_fast | mixed
Then start the run:  ros2 service call /tb_nmpc/start std_srvs/srv/Trigger   (or /nav2_follow/start)
"""
import os
import subprocess

import webots_ros2_driver.utils as webots_utils
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher


def windows_host_ip():
    """IP of the Windows host seen from WSL2 (NAT mode): the default gateway.

    webots_ros2 reads it from /etc/resolv.conf, which is wrong under systemd-resolved.
    """
    route = subprocess.run(["ip", "route", "show", "default"], capture_output=True, text=True).stdout.split()
    return route[route.index("via") + 1] if "via" in route else None


def controller_nodes(context):
    controller = LaunchConfiguration("controller").perform(context)
    nmpc_share = get_package_share_directory("tb_nmpc")
    if controller in ("nmpc", "nmpc_nopred"):
        return [Node(package="tb_nmpc", executable="nmpc_node", name="tb_nmpc", output="screen",
                     parameters=[os.path.join(nmpc_share, "config", "burger.yaml"),
                                 {"use_sim_time": True, "predict": controller == "nmpc", "cmd_vel_type": "stamped"}])]
    if controller in ("mppi", "rpp"):
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nmpc_share, "launch", "nav2_controller.launch.py")),
            launch_arguments={"controller": controller, "robot_model": "burger", "use_sim_time": "true",
                              "cmd_vel_type": "stamped"}.items())]
    return []


def generate_launch_description():
    share = get_package_share_directory("tb_nmpc_sim")
    host_ip = os.environ.get("WEBOTS_HOST_IP") or windows_host_ip()
    if host_ip:
        webots_utils.get_wsl_ip_address = lambda: host_ip

    def set_scenario(context):
        # Read by the scene_supervisor plugin inside the driver process.
        os.environ["TB_SCENARIO"] = LaunchConfiguration("scenario").perform(context)
        return []

    webots = WebotsLauncher(world=os.path.join(share, "worlds", "figure_eight.wbt"),
                            mode=LaunchConfiguration("mode"), ros2_supervisor=True)
    driver = WebotsController(
        robot_name="TurtleBot3Burger",
        parameters=[{"robot_description": os.path.join(share, "resource", "turtlebot.urdf"),
                     "use_sim_time": True, "set_robot_state_publisher": True},
                    os.path.join(share, "resource", "ros2control.yml")],
        remappings=[("/diffdrive_controller/cmd_vel", "/cmd_vel"), ("/diffdrive_controller/odom", "/odom")],
        respawn=True,
    )
    # The driver replaces this description with the one generated from the Webots model. The
    # placeholder already carries the ros2_control block: controller_manager reads
    # /robot_description once, and if it got an empty placeholder first (a race I hit about one
    # start in three) the wheels never came up.
    urdf = open(os.path.join(share, "resource", "turtlebot.urdf")).read()
    wheels = "".join(
        f'<link name="{s}_wheel"/><joint name="{s} wheel motor" type="continuous"><parent link="base_link"/>'
        f'<child link="{s}_wheel"/><axis xyz="0 1 0"/></joint>' for s in ("left", "right"))
    placeholder = urdf.replace("<webots>", '<link name="base_link"/>' + wheels + "<webots>", 1)
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
               parameters=[{"robot_description": placeholder, "use_sim_time": True}])
    footprint = Node(package="tf2_ros", executable="static_transform_publisher", output="screen",
                     arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"])
    timeout = ["--controller-manager-timeout", "50"]
    spawners = [Node(package="controller_manager", executable="spawner", output="screen",
                     arguments=[name] + timeout,
                     parameters=[{"use_sim_time": True}]) for name in ("diffdrive_controller", "joint_state_broadcaster")]
    waiting = WaitForControllerConnection(target_driver=driver, nodes_to_start=spawners)

    return LaunchDescription([
        DeclareLaunchArgument("controller", default_value="nmpc",
                              choices=["nmpc", "nmpc_nopred", "mppi", "rpp", "none"]),
        DeclareLaunchArgument("scenario", default_value="crossing",
                              choices=["clear", "static", "crossing", "crossing_fast", "mixed"]),
        DeclareLaunchArgument("mode", default_value="realtime"),
        OpaqueFunction(function=set_scenario),
        webots,
        webots._supervisor,
        rsp,
        footprint,
        driver,
        waiting,
        OpaqueFunction(function=controller_nodes),
        RegisterEventHandler(OnProcessExit(target_action=webots, on_exit=[EmitEvent(event=Shutdown())])),
    ])

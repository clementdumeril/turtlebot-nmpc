"""Feed the same figure eight to Nav2's controller_server (MPPI or Regulated Pure Pursuit).

The path is a closed loop, so sending it whole would end the goal at once (its last pose is
where the robot starts). Instead this node sends the next few metres ahead of the robot
(2.5 to 4 m) as a new FollowPath goal at 2 Hz, the way bt_navigator replans, and the last segment ends at
the end of the lap.

Services ~/start, ~/stop (Trigger); publishes ~/stats [t, s, cte] and /follow/done (Bool).
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Bool, Float64MultiArray
from std_srvs.srv import Trigger

from .nmpc_node import yaw_of
from .reference import anchored


class Nav2Follow(Node):
    """Client of Nav2's FollowPath action: sends the path ahead of the robot, piece by piece."""

    def __init__(self):
        super().__init__("nav2_follow")
        for name, default in (("odom_topic", "/odom"), ("path_size", 1.5), ("path_shape", "eight"), ("laps", 1.0), ("window_min", 2.5), ("window_max", 4.0),
                              ("rate", 2.0), ("controller_id", "FollowPath"), ("goal_checker_id", "general_goal_checker"),
                              ("autostart", False)):
            self.declare_parameter(name, default)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.g = g
        self.pose = None
        self.frame = "odom"
        self.path = None
        self.s = 0.0
        self.running = False
        self.final_sent = False
        self.client = ActionClient(self, FollowPath, "follow_path")
        self.create_subscription(Odometry, g("odom_topic"), self.on_odom, qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.done_pub = self.create_publisher(Bool, "/follow/done", latched)
        self.ref_pub = self.create_publisher(PathMsg, "~/reference_path", latched)
        self.stats_pub = self.create_publisher(Float64MultiArray, "~/stats", 10)
        self.create_service(Trigger, "~/start", self.on_start)
        self.create_service(Trigger, "~/stop", self.on_stop)
        self.create_timer(1.0 / g("rate"), self.tick)
        self.create_timer(0.1, self.track)
        self.goal_handle = None

    def on_odom(self, msg):
        """Store the current pose and the odometry frame."""
        p = msg.pose.pose
        self.frame = msg.header.frame_id or "odom"
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_start(self, _req, resp):
        """~/start: build the path from the current pose and send the first segment."""
        if self.pose is None or not self.client.wait_for_server(timeout_sec=5.0):
            resp.success, resp.message = False, "no odometry or no follow_path action server"
            return resp
        self.path = anchored(self.g("path_size"), self.g("laps"), self.pose, self.g("path_shape"))
        self.s = 0.0
        self.final_sent = False
        self.running = True
        self.done_pub.publish(Bool(data=False))
        self.ref_pub.publish(self.to_msg(self.path.xy[::20]))
        self.tick()
        resp.success, resp.message = True, f"following a {self.path.length:.2f} m figure eight with Nav2"
        return resp

    def on_stop(self, _req, resp):
        """~/stop: cancel the current FollowPath goal."""
        self.running = False
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        resp.success, resp.message = True, "stopped"
        return resp

    def to_msg(self, xy, theta=None):
        """Points (n, 2) and optional headings as a nav_msgs/Path in the odometry frame."""
        m = PathMsg()
        m.header.frame_id = self.frame
        m.header.stamp = self.get_clock().now().to_msg()
        for i, (x, y) in enumerate(xy):
            ps = PoseStamped()
            ps.header = m.header
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            th = 0.0 if theta is None else float(theta[i])
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
            m.poses.append(ps)
        return m

    def track(self):
        """Every 0.1 s: progress along the path and cross-track error on ~/stats; finish at the end."""
        if not self.running or self.pose is None:
            return
        self.s, cte = self.path.project(self.pose[:2], self.s, back=0.1, ahead=0.3)
        now = self.get_clock().now().nanoseconds * 1e-9
        self.stats_pub.publish(Float64MultiArray(data=[now, self.s, cte]))
        if self.s >= self.path.length - 0.03:
            self.finish()

    def tick(self):
        """At `rate` Hz: send the next segment of the path as a new FollowPath goal."""
        if not self.running or self.final_sent:
            return
        # Segment of window_min..window_max metres, ending at the point farthest from the robot in
        # a straight line. The figure eight folds back on itself: a segment end close to the
        # robot switches on MPPI's goal critics, which stalled it at 0.01 m/s in my first runs.
        # 4 m is also short enough never to contain both passes through the centre crossing.
        # On a small path the segment must also stay well under one lap, or it overlaps itself
        # (on the real 3.14 m circle a 4 m segment wrapped around and MPPI cut through the middle).
        lap = self.path.length / max(self.g("laps"), 1e-6)
        wmin, wmax = min(self.g("window_min"), 0.4 * lap), min(self.g("window_max"), 0.6 * lap)
        if self.s + wmax >= self.path.length:
            end = self.path.length
        else:
            cand = np.arange(self.s + wmin, self.s + wmax, 0.05)
            cx, cy, _, _ = self.path.sample(cand)
            end = float(cand[np.argmax(np.hypot(cx - self.pose[0], cy - self.pose[1]))])
        ss = np.arange(max(self.s - 0.05, 0.0), end, 0.05)
        ss = np.r_[ss, end]
        x, y, th, _ = self.path.sample(ss)
        goal = FollowPath.Goal()
        goal.path = self.to_msg(np.stack([x, y], axis=1), th)
        goal.controller_id = self.g("controller_id")
        goal.goal_checker_id = self.g("goal_checker_id")
        self.final_sent = end >= self.path.length
        fut = self.client.send_goal_async(goal)
        fut.add_done_callback(self.on_goal)

    def on_goal(self, fut):
        """Goal accepted: keep its handle; for the last segment, wait for its result."""
        gh = fut.result()
        if gh is None or not gh.accepted:
            return
        self.goal_handle = gh
        if self.final_sent:
            gh.get_result_async().add_done_callback(self.on_final_result)

    def on_final_result(self, fut):
        """Last segment done: finish if it succeeded, else send it again."""
        res = fut.result()
        if res is not None and res.status == GoalStatus.STATUS_SUCCEEDED:
            self.finish()
        else:
            self.final_sent = False  # aborted (blocked, no progress): keep sending

    def finish(self):
        """Stop and publish /follow/done = true."""
        if self.running:
            self.running = False
            self.done_pub.publish(Bool(data=True))
            self.get_logger().info("end of path reached")


def main():
    rclpy.init()
    node = Nav2Follow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

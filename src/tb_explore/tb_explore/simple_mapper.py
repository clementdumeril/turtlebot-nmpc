"""Occupancy map from /scan and /odom, with a fixed map = odom frame (no loop closure).

For Webots, where cartographer is not installed, and as a backup on the robot. Publishes
/map (OccupancyGrid, latched, 1 Hz) and the identity TF map -> odom.
"""
from __future__ import annotations

import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener

from .mapping import LogOddsMap


def yaw_of(q) -> float:
    """Yaw [rad] of a quaternion message."""
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class SimpleMapper(Node):
    """Odometry-only mapper: LogOddsMap fed by /scan at the interpolated /odom pose."""

    def __init__(self):
        """Declare parameters, broadcast the static map -> odom identity, set up /map and the inputs."""
        super().__init__("simple_mapper")
        for name, default in (("size", 12.0), ("resolution", 0.05), ("max_range", 3.0), ("map_frame", "map"),
                              ("odom_frame", "odom"), ("base_frame", "base_link"), ("scan_topic", "/scan"),
                              ("odom_topic", "/odom"), ("publish_period", 1.0),
                              ("laser_x", -0.032), ("laser_y", 0.0), ("laser_yaw", 0.0)):
            self.declare_parameter(name, default)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.g = g
        self.map = None
        self.odom: deque = deque(maxlen=200)
        self.laser = None
        self.first_scan = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_tf = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = g("map_frame"), g("odom_frame")
        t.transform.rotation.w = 1.0
        self.static_tf.sendTransform(t)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(OccupancyGrid, "/map", latched)
        self.create_subscription(Odometry, g("odom_topic"), self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, g("scan_topic"), self.on_scan, qos_profile_sensor_data)
        self.create_timer(g("publish_period"), self.publish)

    def on_odom(self, msg):
        """Store (t, x, y, yaw) in the last-200 history; the first message centres the map on the robot."""
        p = msg.pose.pose
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        self.odom.append((t, p.position.x, p.position.y, yaw_of(p.orientation)))
        if self.map is None:
            self.map = LogOddsMap(self.g("size"), self.g("resolution"), (p.position.x, p.position.y))

    def pose_at(self, t):
        """Odometry (x, y, yaw) at time t [s], interpolated linearly; the newest pose if t is out of range."""
        h = list(self.odom)
        for a, b in zip(h[:-1], h[1:]):
            if a[0] <= t <= b[0]:
                f = (t - a[0]) / max(b[0] - a[0], 1e-9)
                dyaw = math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))  # shortest way across +-pi
                return a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), a[3] + f * dyaw
        return h[-1][1:]

    def on_scan(self, msg: LaserScan):
        """Integrate one scan: lidar pose = odometry pose at the scan stamp composed with the lidar mount.

        The mount (base_frame -> lidar frame) is read from TF once, with laser_x/y/yaw as fallback.
        """
        if self.map is None or not self.odom:
            return
        if self.laser is None:
            try:
                tf = self.tf_buffer.lookup_transform(self.g("base_frame"), msg.header.frame_id, Time(), Duration(seconds=0.0))
                self.laser = (tf.transform.translation.x, tf.transform.translation.y, yaw_of(tf.transform.rotation))
            except Exception:  # noqa: BLE001
                # Without the lidar TF the map would stay empty: after 2 s use the Burger's mounting.
                now = self.get_clock().now().nanoseconds * 1e-9
                self.first_scan = self.first_scan or now
                if now - self.first_scan < 2.0:
                    return
                self.laser = (self.g("laser_x"), self.g("laser_y"), self.g("laser_yaw"))
                self.get_logger().warn(f"no TF {self.g('base_frame')} -> {msg.header.frame_id}, "
                                       f"using laser_x/y/yaw = {self.laser}")
        x, y, yaw = self.pose_at(Time.from_msg(msg.header.stamp).nanoseconds * 1e-9)
        lx, ly, lyaw = self.laser
        c, s = math.cos(yaw), math.sin(yaw)
        # rotate the mount offset by the robot yaw and add it to the robot position
        sensor =(x + c * lx - s * ly, y + s * lx + c * ly, yaw + lyaw)
        self.map.update(sensor, msg.ranges, msg.angle_min, msg.angle_increment, max(msg.range_min, 0.1),
                        max_range=min(self.g("max_range"), msg.range_max))

    def publish(self):
        """Timer: publish the map as a latched OccupancyGrid on /map in map_frame."""
        if self.map is None:
            return
        g = self.map.grid()
        m = OccupancyGrid()
        m.header.frame_id = self.g("map_frame")
        m.header.stamp = self.get_clock().now().to_msg()
        m.info.resolution = float(g.res)
        m.info.height, m.info.width = g.data.shape
        m.info.origin.position.x, m.info.origin.position.y = float(g.ox), float(g.oy)
        m.info.origin.orientation.w = 1.0
        m.data = np.clip(g.data, -1, 100).astype(np.int8).ravel().tolist()
        self.pub.publish(m)


def main():
    """Run the simple_mapper node until Ctrl-C."""
    rclpy.init()
    node = SimpleMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

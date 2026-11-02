"""Photo + pose capture on a service call, in the Lab 6 format.

Same output as lab6_capture/capture_pose.py, so the Lab 6 reconstruction scripts read it as is:
<output_dir>/run_YYYYmmdd_HHMMSS/img_0000.png ... and poses.csv with the columns
file, image_stamp, tf_stamp, tf_source, x, y, z, qx, qy, qz, qw, yaw_deg
(pose of base_link in the map at the image time, or the latest one if the clocks disagree).

The difference is the trigger: ~/capture (Trigger) instead of the space bar. The photo is the
second frame that arrives after the call (the first may have been taken while turning), and
~/saved (Int32, photos so far) tells the explorer it can turn again.
"""
from __future__ import annotations

import csv
import math
import os
import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

CSV_FIELDS = ["file", "image_stamp", "tf_stamp", "tf_source",
              "x", "y", "z", "qx", "qy", "qz", "qw", "yaw_deg"]


def stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class AutoCapture(Node):
    def __init__(self):
        super().__init__("auto_capture")
        for name, default in (("image_topic", "/image_raw"), ("target_frame", "map"), ("source_frame", "base_link"),
                              ("output_dir", os.path.expanduser("~/explore_captures")), ("max_image_age", 1.0),
                              ("skip_frames", 1)):
            self.declare_parameter(name, default)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.g = g
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lock = threading.Lock()
        self.pending_since = None
        self.last_arrival = 0.0
        self.count = 0
        self.out = os.path.join(os.path.expanduser(g("output_dir")), time.strftime("run_%Y%m%d_%H%M%S"))
        os.makedirs(self.out, exist_ok=True)
        self.csv_path = os.path.join(self.out, "poses.csv")
        self.create_subscription(Image, g("image_topic"), self.on_image, qos_profile_sensor_data)
        self.saved_pub = self.create_publisher(Int32, "~/saved", 10)
        self.skip = 0
        self.create_service(Trigger, "~/capture", self.on_capture)
        self.create_timer(2.0, self.check_camera)
        self.get_logger().info(f"listening on {g('image_topic')}, saving to {self.out}")

    def check_camera(self):
        if self.pending_since is not None and time.monotonic() - self.pending_since > self.g("max_image_age"):
            self.get_logger().warn(f"no new image on {self.g('image_topic')} for "
                                   f"{time.monotonic() - self.pending_since:.1f} s, is the camera running?")

    def on_capture(self, _req, resp):
        with self.lock:
            self.pending_since = time.monotonic()
            self.skip = self.g("skip_frames")  # frames already in flight were taken before the call
        resp.success, resp.message = True, f"capturing img_{self.count:04d}"
        return resp

    def lookup_pose(self, stamp):
        try:
            tf = self.tf_buffer.lookup_transform(self.g("target_frame"), self.g("source_frame"),
                                                 Time.from_msg(stamp), timeout=Duration(seconds=0.2))
            return tf, "image_stamp"
        except TransformException:
            tf = self.tf_buffer.lookup_transform(self.g("target_frame"), self.g("source_frame"), Time())
            return tf, "latest"

    def on_image(self, msg: Image):
        arrival = time.monotonic()
        with self.lock:
            take = self.pending_since is not None and arrival > self.pending_since
            if take and self.skip > 0:
                self.skip -= 1
                take = False
            if take:
                self.pending_since = None
        if not take:
            return
        try:
            tf, source = self.lookup_pose(msg.header.stamp)
        except TransformException as e:
            self.get_logger().error(f"no {self.g('target_frame')} -> {self.g('source_frame')} transform ({e}), photo skipped")
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"could not convert the {msg.encoding} image: {e}")
            return
        name = f"img_{self.count:04d}.png"
        cv2.imwrite(os.path.join(self.out, name), frame)
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        row = {"file": name, "image_stamp": f"{stamp_to_sec(msg.header.stamp):.3f}",
               "tf_stamp": f"{stamp_to_sec(tf.header.stamp):.3f}", "tf_source": source,
               "x": f"{t.x:.4f}", "y": f"{t.y:.4f}", "z": f"{t.z:.4f}",
               "qx": f"{q.x:.5f}", "qy": f"{q.y:.5f}", "qz": f"{q.z:.5f}", "qw": f"{q.w:.5f}",
               "yaw_deg": f"{yaw:.2f}"}
        new = not os.path.exists(self.csv_path)
        with open(self.csv_path, "a", newline="") as f:
            w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
            if new:
                w.writeheader()
            w.writerow(row)
        self.count += 1
        self.saved_pub.publish(Int32(data=self.count))
        self.get_logger().info(f"saved {name}: x={t.x:.3f} y={t.y:.3f} yaw={yaw:.1f} deg ({source})")


def main():
    rclpy.init()
    node = AutoCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

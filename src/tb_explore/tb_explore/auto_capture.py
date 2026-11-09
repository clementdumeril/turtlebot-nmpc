"""Photo + pose capture on a service call.

Output:
<output_dir>/run_YYYYmmdd_HHMMSS/img_0000.png ... and poses.csv with the columns
file, image_stamp, tf_stamp, tf_source, x, y, z, qx, qy, qz, qw, yaw_deg
(pose of base_link in the map at the image time, or the latest one if the clocks disagree).

The trigger is ~/capture (Trigger). The photo is the
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
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
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
    """Saves one camera frame and its map pose per ~/capture call, in a new run_* folder."""

    def __init__(self):
        """Declare parameters, create the run folder, subscribe to the camera, offer ~/capture."""
        super().__init__("auto_capture")
        for name, default in (("image_topic", "/image_raw"), ("target_frame", "map"), ("source_frame", "base_link"),
                              ("output_dir", os.path.expanduser("~/explore_captures")), ("max_image_age", 1.0),
                              ("skip_frames", 1), ("image_reliable", True)):
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
        # Reliable QoS: an image is hundreds of UDP fragments over the wifi, and
        # with best effort one lost fragment drops the whole image (photos then stopped coming for
        # tens of seconds on the lab TurtleBot). Set image_reliable:=false for a best-effort camera.
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE) if g("image_reliable") else qos_profile_sensor_data
        self.qos = qos
        self.topic = g("image_topic")
        self.sub = self.create_subscription(Image, self.topic, self.on_image, qos)
        self.t0 = time.monotonic()
        self.saved_pub = self.create_publisher(Int32, "~/saved", 10)
        self.skip = 0
        self.create_service(Trigger, "~/capture", self.on_capture)
        self.create_timer(2.0, self.check_camera)
        self.get_logger().info(f"listening on {g('image_topic')}, saving to {self.out}")

    def check_camera(self):
        """2 s timer: find the camera topic if the configured one is silent, warn if a capture waits too long."""
        # Nothing on the given topic after 3 s: switch to the live image topic if there is exactly
        # one (the lab robots publish under their name, /apple/image_raw; a run on 2 November
        # took no photo because the launch was started without image_topic:=).
        if self.last_arrival == 0.0 and time.monotonic() - self.t0 > 3.0:
            live = [n for n, types in self.get_topic_names_and_types()
                    if "sensor_msgs/msg/Image" in types and n != self.topic and self.count_publishers(n) > 0
                    and not n.endswith(("compressed", "theora", "compressedDepth"))]
            raw = [n for n in live if n.endswith("image_raw")] or live
            if len(raw) == 1:
                self.get_logger().warn(f"no image on {self.topic}: switching to {raw[0]}")
                self.destroy_subscription(self.sub)
                self.topic = raw[0]
                self.sub = self.create_subscription(Image, self.topic, self.on_image, self.qos)
                self.t0 = time.monotonic()
            elif live:
                self.get_logger().error(f"no image on {self.topic}; live image topics: {', '.join(live)}. "
                                        f"Relaunch with image_topic:=<one of them>", throttle_duration_sec=10.0)
        if self.pending_since is not None and time.monotonic() - self.pending_since > self.g("max_image_age"):
            self.get_logger().warn(f"no new image on {self.topic} for "
                                   f"{time.monotonic() - self.pending_since:.1f} s, is the camera running?")

    def on_capture(self, _req, resp):
        """~/capture: arm a capture; on_image saves a later frame. Returns at once, before the photo."""
        with self.lock:
            self.pending_since = time.monotonic()
            self.skip = self.g("skip_frames")  # frames already in flight were taken before the call
        resp.success, resp.message = True, f"capturing img_{self.count:04d}"
        return resp

    def lookup_pose(self, stamp):
        """target_frame -> source_frame TF at the image stamp, else the latest one; returns (tf, source label).

        Raises TransformException if neither is available.
        """
        try:
            tf = self.tf_buffer.lookup_transform(self.g("target_frame"), self.g("source_frame"),
                                                 Time.from_msg(stamp), timeout=Duration(seconds=0.2))
            return tf, "image_stamp"
        except TransformException:
            tf = self.tf_buffer.lookup_transform(self.g("target_frame"), self.g("source_frame"), Time())
            return tf, "latest"

    def on_image(self, msg: Image):
        """Camera callback: if a capture is armed, skip skip_frames frames, then save the next one.

        Writes img_NNNN.png and a poses.csv row, then publishes the new count on ~/saved.
        """
        arrival = time.monotonic()
        self.last_arrival = arrival
        with self.lock:  # on_capture may run concurrently
            take = self.pending_since is not None and arrival > self.pending_since
            if take and self.skip > 0:  # frame possibly exposed before the call: drop it
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
    """Run the auto_capture node until Ctrl-C."""
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

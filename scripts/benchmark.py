"""Run one lap with the controller that is already running and log it to CSV.

    python3 scripts/benchmark.py --controller nmpc --scenario crossing --out results/sim/crossing_nmpc
    python3 scripts/benchmark.py --controller mppi --real --out results/real/run1   # real robot, no ground truth

Simulation: one row per Webots step (50 Hz) from /sim/ground_truth. Real robot (--real): one row
per odometry message. Writes run.csv and meta.json in --out.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import Trigger

COLUMNS = ["t", "gt_x", "gt_y", "gt_yaw", "ped_x", "ped_y", "ped_vx", "ped_vy", "ped_state",
           "odom_x", "odom_y", "odom_yaw", "odom_v", "odom_w", "cmd_v", "cmd_w", "cmd_t",
           "solve_ms", "status", "wall_ms", "safety_stop", "n_tracks", "s", "cte",
           "est_x", "est_y", "est_vx", "est_vy", "scan_min"]


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Recorder(Node):
    def __init__(self, args):
        super().__init__("benchmark_recorder",
                         parameter_overrides=[Parameter("use_sim_time", value=not args.real)])
        self.args = args
        self.rows = []
        self.last = {c: float("nan") for c in COLUMNS}
        self.done = False
        self.started = False
        self.gt_seen = False
        self.odom_seen = False
        self.tracks = []
        self.create_subscription(Odometry, args.odom_topic, self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Float64MultiArray, "/sim/ground_truth", self.on_gt, 50)
        self.create_subscription(LaserScan, args.scan_topic, self.on_scan, qos_profile_sensor_data)
        cmd_type = Twist if args.cmd_type == "twist" else TwistStamped
        self.create_subscription(cmd_type, args.cmd_topic, self.on_cmd, 10)
        self.create_subscription(Float64MultiArray, "/tb_nmpc/stats", self.on_nmpc_stats, 10)
        self.create_subscription(Float64MultiArray, "/tb_nmpc/tracks", self.on_tracks, 10)
        self.create_subscription(Float64MultiArray, "/nav2_follow/stats", self.on_nav2_stats, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/follow/done", self.on_done, latched)
        self.record_pub = self.create_publisher(String, "/sim/record", 1)
        prefix = "/tb_nmpc" if args.controller.startswith("nmpc") else "/nav2_follow"
        self.start_cli = self.create_client(Trigger, prefix + "/start")
        self.stop_cli = self.create_client(Trigger, prefix + "/stop")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, m):
        self.odom_seen = True
        L = self.last
        L["odom_x"], L["odom_y"] = m.pose.pose.position.x, m.pose.pose.position.y
        L["odom_yaw"] = yaw_of(m.pose.pose.orientation)
        L["odom_v"], L["odom_w"] = m.twist.twist.linear.x, m.twist.twist.angular.z
        if self.args.real and self.started:
            L["t"] = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            self.rows.append(dict(L))

    def on_gt(self, m):
        self.gt_seen = True
        d = m.data
        L = self.last
        (L["t"], L["gt_x"], L["gt_y"], L["gt_yaw"], L["ped_x"], L["ped_y"], L["ped_vx"], L["ped_vy"],
         L["ped_state"]) = d[:9]
        # Track closest to the true pedestrian (perception check).
        if L["ped_state"] == 1 and self.tracks:
            best = min(self.tracks, key=lambda tr: math.hypot(tr[1] - L["ped_x"], tr[2] - L["ped_y"]))
            if math.hypot(best[1] - L["ped_x"], best[2] - L["ped_y"]) < 0.6:
                L["est_x"], L["est_y"], L["est_vx"], L["est_vy"] = best[1:5]
            else:
                L["est_x"] = L["est_y"] = L["est_vx"] = L["est_vy"] = float("nan")
        if self.started and not self.args.real:
            self.rows.append(dict(L))

    def on_scan(self, m):
        r = [x for x in m.ranges if math.isfinite(x) and x >= max(m.range_min, 0.05)]
        self.last["scan_min"] = min(r) if r else float("nan")

    def on_cmd(self, m):
        tw = m.twist if isinstance(m, TwistStamped) else m
        self.last["cmd_v"], self.last["cmd_w"] = tw.linear.x, tw.angular.z
        self.last["cmd_t"] = self.now()

    def on_nmpc_stats(self, m):
        d = m.data
        L = self.last
        L["solve_ms"], L["status"], L["wall_ms"], L["n_tracks"], L["s"], L["cte"], L["safety_stop"] = d[1:8]

    def on_tracks(self, m):
        d = list(m.data)
        self.tracks = [d[i:i + 6] for i in range(0, len(d), 6)]

    def on_nav2_stats(self, m):
        self.last["s"], self.last["cte"] = m.data[1], m.data[2]

    def on_done(self, m):
        if self.started and m.data:
            self.done = True

    def call(self, cli, timeout=10.0):
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--controller", required=True, choices=["nmpc", "nmpc_nopred", "mppi", "rpp"])
    ap.add_argument("--scenario", default="")
    ap.add_argument("--out", required=True)
    ap.add_argument("--timeout", type=float, default=110.0, help="[s] of sim (or wall) time for the lap")
    ap.add_argument("--real", action="store_true", help="real robot: no ground truth, rows from odometry")
    ap.add_argument("--odom-topic", default="/odom")
    ap.add_argument("--scan-topic", default="/scan")
    ap.add_argument("--cmd-topic", default="/cmd_vel")
    ap.add_argument("--cmd-type", default="stamped", choices=["stamped", "twist"])
    ap.add_argument("--record", default="", help="video file on the Webots host (simulation only)")
    args = ap.parse_args()

    rclpy.init()
    node = Recorder(args)
    t_wall = time.time()
    while not (node.odom_seen and (node.gt_seen or args.real)):
        rclpy.spin_once(node, timeout_sec=0.2)
        if time.time() - t_wall > 120:
            raise SystemExit("no odometry / ground truth")
    # The Nav2 feeder refuses to start until controller_server is active: retry.
    for _ in range(60):
        res = node.call(node.start_cli)
        if res is not None and res.success:
            node.get_logger().info(res.message)
            break
        time.sleep(1.0)
    else:
        raise SystemExit("controller did not start")
    if args.record:
        node.record_pub.publish(String(data=args.record))
    node.started = True
    t0 = node.now()
    w0 = time.time()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.now() - t0 > args.timeout or time.time() - w0 > 3 * args.timeout:
            node.get_logger().warn("timeout")
            break
    t_end = node.now()
    # A second of data after the end.
    while node.now() - t_end < 1.0 and time.time() - w0 < 3 * args.timeout + 5:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.call(node.stop_cli, timeout=3.0)
    if args.record:
        node.record_pub.publish(String(data=""))
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "run.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=COLUMNS)
        w.writeheader()
        w.writerows(node.rows)
    meta = {"controller": args.controller, "scenario": args.scenario, "completed": node.done,
            "t_start": t0, "t_end": t_end, "rows": len(node.rows), "real": args.real}
    (out / "meta.json").write_text(json.dumps(meta, indent=1))
    print(json.dumps(meta))
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()

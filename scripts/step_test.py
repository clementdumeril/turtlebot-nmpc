"""Step response of the base: command latency and first-order time constant, for v and omega.

    python3 scripts/step_test.py --cmd-type twist            # TurtleBot3 Humble, TurtleBot4
    python3 scripts/step_test.py --cmd-type stamped --v 0.1 --w 0.8

Drives 2 s forward at --v, stops, then 2 s turning in place at --w, stops. Needs about 0.5 m of
free space ahead. Prints the delay (command to 10 % of the step) and the time constant
(10 % to 63 %) measured on /odom, and saves the raw data to --out.
"""
from __future__ import annotations

import argparse
import csv
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def analyse(t, y, t_step, target):
    y = np.asarray(y)
    t = np.asarray(t)
    after = t >= t_step
    if target == 0 or not after.any():
        return float("nan"), float("nan")
    frac = y[after] / target
    tt = t[after] - t_step
    i10 = np.argmax(frac >= 0.1) if np.any(frac >= 0.1) else None
    i63 = np.argmax(frac >= 0.63) if np.any(frac >= 0.63) else None
    delay = float(tt[i10]) if i10 is not None else float("nan")
    tau = float(tt[i63] - tt[i10]) if i10 is not None and i63 is not None else float("nan")
    return delay, tau


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cmd-topic", default="/cmd_vel")
    ap.add_argument("--cmd-type", default="twist", choices=["twist", "stamped"])
    ap.add_argument("--odom-topic", default="/odom")
    ap.add_argument("--v", type=float, default=0.10)
    ap.add_argument("--w", type=float, default=0.8)
    ap.add_argument("--out", default="step_test.csv")
    args = ap.parse_args()

    rclpy.init()
    node = Node("step_test")
    stamped = args.cmd_type == "stamped"
    pub = node.create_publisher(TwistStamped if stamped else Twist, args.cmd_topic, 10)
    log = []
    node.create_subscription(Odometry, args.odom_topic,
                             lambda m: log.append((time.time(), m.twist.twist.linear.x, m.twist.twist.angular.z)),
                             qos_profile_sensor_data)

    def send(v, w):
        tw = Twist()
        tw.linear.x, tw.angular.z = v, w
        if stamped:
            m = TwistStamped()
            m.header.stamp = node.get_clock().now().to_msg()
            m.header.frame_id = "base_link"
            m.twist = tw
            pub.publish(m)
        else:
            pub.publish(tw)

    plan = [(1.0, 0.0, 0.0), (2.0, args.v, 0.0), (1.5, 0.0, 0.0), (2.0, 0.0, args.w), (1.5, 0.0, 0.0)]
    steps = []
    try:
        for dur, v, w in plan:
            t0 = time.time()
            steps.append((t0, v, w))
            while time.time() - t0 < dur:
                send(v, w)
                rclpy.spin_once(node, timeout_sec=0.02)
    finally:
        for _ in range(5):
            send(0.0, 0.0)
            time.sleep(0.02)
    t = [r[0] for r in log]
    dv, tv = analyse(t, [r[1] for r in log], steps[1][0], args.v)
    dw, tw_ = analyse(t, [r[2] for r in log], steps[3][0], args.w)
    print(f"linear : delay {dv * 1e3:.0f} ms, time constant {tv * 1e3:.0f} ms (odom rate {len(log) / (t[-1] - t[0]):.0f} Hz)")
    print(f"angular: delay {dw * 1e3:.0f} ms, time constant {tw_ * 1e3:.0f} ms")
    print("Set the NMPC's latency parameter to about delay + time constant / 2 if the runs oscillate.")
    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "v", "w"])
        w.writerows(log)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()

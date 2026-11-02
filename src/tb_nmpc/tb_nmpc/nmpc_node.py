"""ROS 2 node: lidar obstacle tracking + acados NMPC following a figure eight.

Subscribes  /scan (LaserScan), /odom (Odometry)
Publishes   /cmd_vel (Twist or TwistStamped, see cmd_vel_type)
            ~/stats  Float64MultiArray [t, solve_ms, status, wall_ms, n_tracks, s, cte, safety_stop, v, w]
            ~/tracks Float64MultiArray [id, x, y, vx, vy, r] * n   (odometry frame)
            ~/predicted_path, ~/reference_path (Path), ~/obstacles (MarkerArray)
            /follow/done (Bool) at the end of the path
Services    ~/start, ~/stop (std_srvs/Trigger). start anchors the figure eight at the current pose.
"""
from __future__ import annotations

import math
import os
import signal
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64MultiArray
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .ocp import FAR, Nmpc, OcpConfig
from .perception import PerceptionParams, detect
from .reference import Path, anchored
from .tracker import Tracker, TrackerParams


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class NmpcNode(Node):
    def __init__(self):
        super().__init__("tb_nmpc")
        P = self.declare_parameter
        P("robot_model", "burger")
        P("cmd_vel_topic", "/cmd_vel")
        P("cmd_vel_type", "auto")          # auto | twist | stamped
        P("odom_topic", "/odom")
        P("scan_topic", "/scan")
        P("base_frame", "base_link")
        P("path_shape", "eight")           # eight | circle
        P("path_size", 1.5)                # [m] half-width a of the figure eight, or circle radius
        P("laps", 1.0)
        P("v_ref", 0.18)
        P("v_max", 0.20)
        P("v_min", 0.0)
        P("w_max", 2.0)
        P("a_max", 0.5)
        P("alpha_max", 3.0)
        P("robot_radius", 0.105)
        P("safety_margin", 0.05)           # [m] added to every obstacle radius in the MPC
        P("stop_margin", 0.03)             # [m] safety stop if a lidar point is this close to the body, ahead
        P("predict", True)                 # False: obstacles frozen at their current position (ablation)
        P("horizon_steps", 30)
        P("dt", 0.1)                       # stage length and control period
        P("max_obstacles", 6)
        P("rti_iters", 1)
        P("velocity_state", "command")     # command | odom: where v, omega of the initial state come from
        P("latency", 0.0)                  # [s] forward-predict the initial state by this much
        P("autostart", False)
        P("laser_x", 0.0)                  # fallback lidar mounting if TF is missing
        P("laser_y", 0.0)
        P("laser_yaw", 0.0)
        P("leg_merge_dist", 0.35)
        P("track_v_static", 0.10)          # [m/s] tracks slower than this are predicted static
        P("track_min_hits", 3)             # detections before a track's velocity is used
        P("odom_timeout", 0.5)             # [s] zero command if odometry is older than this
        P("scan_timeout", 1.0)             # [s] zero command if the lidar is older than this
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.g = g

        cfg = OcpConfig(N=g("horizon_steps"), dt=g("dt"), M=g("max_obstacles"),
                        v_min=g("v_min"), v_max=g("v_max"), w_max=g("w_max"),
                        a_max=g("a_max"), alpha_max=g("alpha_max"))
        t0 = time.time()
        self.mpc = Nmpc(cfg)
        self.get_logger().info(f"acados solver ready in {time.time() - t0:.1f} s (N={cfg.N}, dt={cfg.dt}, M={cfg.M})")
        self.cfg = cfg
        self.perc = PerceptionParams(merge_dist=g("leg_merge_dist"))
        self.tracker = Tracker(TrackerParams(v_static=g("track_v_static"), min_hits=g("track_min_hits")))
        self.static = np.zeros((0, 3))
        self.static_inflated = np.zeros((0, 3))
        self.scan_xy = np.zeros((0, 2))

        self.odom_hist: deque = deque(maxlen=200)
        self.v_state = 0.0
        self.w_state = 0.0
        self.path: Path | None = None
        self.s = 0.0
        self.running = False
        self.stop_count = 0
        self._stopping = False
        self.laser_tf = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, g("odom_topic"), self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, g("scan_topic"), self.on_scan, qos_profile_sensor_data)
        self.cmd_pub = None
        self.stats_pub = self.create_publisher(Float64MultiArray, "~/stats", 10)
        self.tracks_pub = self.create_publisher(Float64MultiArray, "~/tracks", 10)
        self.pred_pub = self.create_publisher(PathMsg, "~/predicted_path", 1)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.ref_pub = self.create_publisher(PathMsg, "~/reference_path", latched)
        self.marker_pub = self.create_publisher(MarkerArray, "~/obstacles", 1)
        self.done_pub = self.create_publisher(Bool, "/follow/done", latched)
        self.create_service(Trigger, "~/start", self.on_start)
        self.create_service(Trigger, "~/stop", self.on_stop)
        self.create_timer(cfg.dt, self.step)
        self.odom_frame = "odom"
        # Local receive times (watchdog), and the offset between this computer's clock and the
        # robot's (scan stamps, which the tracks use): the minimum of (receive time - stamp) over
        # the last 10 s, i.e. the clock offset plus the smallest transport delay. With synced
        # clocks (simulation) it is ~0 and the prediction is timed exactly as with stamps alone.
        self.odom_rx = None
        self.scan_rx = None
        self.clock_offsets: deque = deque(maxlen=50)
        self._stale = False
        self.started_at = 0.0
        if g("autostart"):
            self.create_timer(1.0, self._autostart_once)

    # ---------- inputs ----------
    def on_odom(self, msg: Odometry):
        p = msg.pose.pose
        self.odom_frame = msg.header.frame_id or "odom"
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        self.odom_hist.append((t, p.position.x, p.position.y, yaw_of(p.orientation),
                               msg.twist.twist.linear.x, msg.twist.twist.angular.z))
        self.odom_rx = self.local_now()

    def pose_at(self, t: float):
        """Odometry pose interpolated at time t (yaw interpolated on the short arc)."""
        h = self.odom_hist
        if not h:
            return None
        if t <= h[0][0]:
            return h[0][1:4]
        if t >= h[-1][0]:
            return h[-1][1:4]
        for a, b in zip(reversed(list(h)[:-1]), reversed(list(h)[1:])):
            if a[0] <= t <= b[0]:
                f = (t - a[0]) / max(b[0] - a[0], 1e-9)
                dyaw = math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))
                return (a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), a[3] + f * dyaw)
        return h[-1][1:4]

    def laser_offset(self, frame: str):
        if self.laser_tf is None:
            try:
                tf = self.tf_buffer.lookup_transform(self.g("base_frame"), frame, Time(), Duration(seconds=0.0))
                tr = tf.transform
                self.laser_tf = (tr.translation.x, tr.translation.y, yaw_of(tr.rotation))
                self.get_logger().info(f"lidar mounting from TF {self.g('base_frame')} -> {frame}: "
                                       f"x={self.laser_tf[0]:.3f} y={self.laser_tf[1]:.3f} yaw={self.laser_tf[2]:.3f}")
            except Exception:  # noqa: BLE001
                return (self.g("laser_x"), self.g("laser_y"), self.g("laser_yaw"))
        return self.laser_tf

    def on_scan(self, msg: LaserScan):
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        pose = self.pose_at(t)
        if pose is None:
            return
        lx, ly, lyaw = self.laser_offset(msg.header.frame_id)
        c, s = math.cos(pose[2]), math.sin(pose[2])
        sensor = (pose[0] + c * lx - s * ly, pose[1] + s * lx + c * ly, pose[2] + lyaw)
        small, static = detect(msg.ranges, msg.angle_min, msg.angle_increment, sensor, self.perc)
        self.tracker.update(t, small)
        self.static = static
        self.scan_rx = self.local_now()
        self.clock_offsets.append(self.scan_rx - t)
        r = np.asarray(msg.ranges, dtype=float)
        a = msg.angle_min + msg.angle_increment * np.arange(len(r)) + sensor[2]
        ok = np.isfinite(r) & (r > self.perc.range_min) & (r < 1.0)
        self.scan_xy = np.stack([sensor[0] + r[ok] * np.cos(a[ok]), sensor[1] + r[ok] * np.sin(a[ok])], axis=1)
        out = []
        for tr in self.tracker.tracks:
            v = self.tracker.velocity(tr)
            out += [tr.id, tr.x[0], tr.x[1], v[0], v[1], tr.r]
        self.tracks_pub.publish(Float64MultiArray(data=[float(x) for x in out]))

    # ---------- services ----------
    def _autostart_once(self):
        if not self.running and self.odom_hist and self.path is None:
            self.on_start(None, Trigger.Response())

    def on_start(self, _req, resp):
        if not self.odom_hist:
            resp.success, resp.message = False, "no odometry yet"
            return resp
        self.setup_cmd_pub()
        pose = self.odom_hist[-1][1:4]
        self.path = anchored(self.g("path_size"), self.g("laps"), pose, self.g("path_shape"))
        self.s = 0.0
        self.v_state = self.w_state = 0.0
        self.mpc.reset()
        self.mpc.set_limits(self.g("v_min"), self.g("v_max"), self.g("w_max"), self.g("a_max"), self.g("alpha_max"))
        self.running = True
        self.started_at = self.local_now()
        self.done_pub.publish(Bool(data=False))
        self.publish_path(self.ref_pub, self.path.xy[::20])
        resp.success, resp.message = True, f"following a {self.path.length:.2f} m {self.g('path_shape')}"
        self.get_logger().info(resp.message)
        return resp

    def on_stop(self, _req, resp):
        self.running = False
        self.publish_cmd(0.0, 0.0)
        resp.success, resp.message = True, "stopped"
        return resp

    def setup_cmd_pub(self):
        if self.cmd_pub is not None:
            return
        topic, kind = self.g("cmd_vel_topic"), self.g("cmd_vel_type")
        if kind == "auto":
            subs = self.get_subscriptions_info_by_topic(topic)
            types = {s.topic_type for s in subs}
            if "geometry_msgs/msg/TwistStamped" in types:
                kind = "stamped"
            elif "geometry_msgs/msg/Twist" in types:
                kind = "twist"
            else:
                kind = "stamped" if os.environ.get("ROS_DISTRO", "") not in ("humble", "iron") else "twist"
            self.get_logger().info(f"cmd_vel type: {kind} (subscribers: {sorted(types) or 'none'})")
        self.stamped = kind == "stamped"
        self.cmd_pub = self.create_publisher(TwistStamped if self.stamped else Twist, topic, 10)

    def publish_cmd(self, v: float, w: float):
        if self.cmd_pub is None:
            return
        tw = Twist()
        tw.linear.x, tw.angular.z = float(v), float(w)
        if self.stamped:
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = self.g("base_frame")
            m.twist = tw
            self.cmd_pub.publish(m)
        else:
            self.cmd_pub.publish(tw)

    # ---------- control ----------
    def obstacle_params(self, now: float) -> np.ndarray:
        """(N+1, M, 3) predicted circles for the stages, the M most relevant ones."""
        N, M, dt = self.cfg.N, self.cfg.M, self.cfg.dt
        inflate = self.g("robot_radius") + self.g("safety_margin")
        times = dt * np.arange(N + 1)
        # Now, on the robot's clock (the one the scan stamps and the tracks use).
        t_robot = now - min(self.clock_offsets) if self.clock_offsets else now
        cands = []
        for tr in self.tracker.tracks:
            cands.append(self.tracker.predict(tr, (t_robot - tr.t) + times, use_velocity=self.g("predict")))
        for c in self.static:
            cands.append(np.tile(c, (N + 1, 1)))
        obs = np.tile([FAR, FAR, 0.0], (N + 1, M, 1))
        # Inflated circles that do not move over the horizon (reference skip, see Path.skip_blocked).
        self.static_inflated = np.array([[*c[0, :2], c[0, 2] + inflate] for c in cands
                                         if np.allclose(c[0, :2], c[-1, :2])]).reshape(-1, 3)
        if not cands:
            return obs
        traj = self.mpc.X[:, :2] if self.mpc.initialized else np.tile(self.odom_hist[-1][1:3], (N + 1, 1))
        score = [float(np.min(np.hypot(*(traj - c[:, :2]).T) - c[:, 2])) for c in cands]
        order = [i for i in np.argsort(score) if score[i] < 2.0][:M]
        for j, i in enumerate(order):
            obs[:, j, :2] = cands[i][:, :2]
            obs[:, j, 2] = cands[i][:, 2] + inflate
        return obs

    def local_now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def step(self):
        if not self.running or not self.odom_hist:
            return
        now = self.local_now()
        # Watchdog: stale odometry or lidar (dropped link, bring-up restarted) ends the run. It does
        # not resume on its own: an earlier version did, and the robot drove off as soon as the
        # bring-up was relaunched. A new ~/start is needed. The first second after start is a
        # grace period for the first scan.
        stale = (self.odom_rx is None or now - self.odom_rx > self.g("odom_timeout")
                 or self.scan_rx is None or now - self.scan_rx > self.g("scan_timeout"))
        if stale:
            self.v_state = self.w_state = 0.0
            self.publish_cmd(0.0, 0.0)
            if now - self.started_at > 1.0:
                self.running = False
                self.mpc.reset()
                self.get_logger().warn("odometry or lidar lost: run stopped, call ~/start again")
            return
        try:
            self.control(now)
        except Exception as e:  # noqa: BLE001
            self.running = False
            self.publish_cmd(0.0, 0.0)
            self.get_logger().error(f"control step failed, stopped: {e!r}")

    def control(self, now: float):
        t_odom, x, y, yaw, v_odom, w_odom = self.odom_hist[-1]
        if self.g("velocity_state") == "odom":
            v0, w0 = v_odom, w_odom
        else:
            v0, w0 = self.v_state, self.w_state
        lat = self.g("latency")
        if lat > 0:
            x += v0 * math.cos(yaw) * lat
            y += v0 * math.sin(yaw) * lat
            yaw += w0 * lat
        # Progress may only move a little per step (0.012 m at 0.12 m/s): a narrow window keeps a
        # detour inside a small circle from snapping the progress to another part of the path,
        # which made the robot cut across on the real circle (r = 0.5 m) with the 0.8 m default.
        self.s, cte = self.path.project((x, y), self.s, back=0.1, ahead=0.3)
        obs = self.obstacle_params(now)
        s_ref, v_ref = self.path.horizon_s(self.s, self.g("v_ref"), self.cfg.dt, self.cfg.N)
        s_ref = self.path.skip_blocked(s_ref, self.static_inflated)
        ref = self.path.at(s_ref, v_ref)
        ref[:, 2] += np.round((yaw - ref[0, 2]) / (2 * np.pi)) * 2 * np.pi  # same branch as the robot
        x0 = np.array([x, y, yaw, v0, w0])
        t0 = time.perf_counter()
        status, t_solve = self.mpc.solve(x0, ref, obs, self.g("rti_iters"))
        wall = time.perf_counter() - t0
        if status not in (0, 2):
            # QP failure: brake and cold-start next time.
            self.get_logger().warn(f"acados status {status}, braking", throttle_duration_sec=1.0)
            v_cmd = max(0.0, v0 - self.g("a_max") * self.cfg.dt)
            w_cmd = 0.5 * w0
            self.mpc.reset()
        else:
            v_cmd, w_cmd = float(self.mpc.X[1, 3]), float(self.mpc.X[1, 4])
        # Safety stop: a lidar return within the body + stop_margin, in front of the robot.
        stop = 0.0
        if len(self.scan_xy) and v_cmd > 0:
            d = self.scan_xy - (x, y)
            ahead = d @ np.array([math.cos(yaw), math.sin(yaw)]) > 0
            close = np.hypot(*d.T) < self.g("robot_radius") + self.g("stop_margin")
            if np.any(ahead & close):
                v_cmd, stop = 0.0, 1.0
                if not self._stopping:
                    self.stop_count += 1
                    self.get_logger().warn("safety stop")
        self._stopping = bool(stop)
        finished = self.s >= self.path.length - 0.03
        if finished:
            v_cmd = w_cmd = 0.0
        self.v_state, self.w_state = v_cmd, w_cmd
        self.publish_cmd(v_cmd, w_cmd)
        self.stats_pub.publish(Float64MultiArray(data=[
            now, t_solve * 1e3, float(status), wall * 1e3, float(len(self.tracker.tracks)),
            self.s, cte, stop, v_cmd, w_cmd]))
        self.publish_path(self.pred_pub, self.mpc.X[:, :2])
        self.publish_markers(obs)
        if finished:
            self.running = False
            self.done_pub.publish(Bool(data=True))
            self.get_logger().info(f"end of path reached, {self.stop_count} safety stops")

    # ---------- visualisation ----------
    def publish_path(self, pub, xy):
        m = PathMsg()
        m.header.frame_id = self.odom_frame
        m.header.stamp = self.get_clock().now().to_msg()
        for px, py in xy:
            ps = PoseStamped()
            ps.header = m.header
            ps.pose.position.x, ps.pose.position.y = float(px), float(py)
            ps.pose.orientation.w = 1.0
            m.poses.append(ps)
        pub.publish(m)

    def publish_markers(self, obs):
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for j in range(self.cfg.M):
            for k, alpha in ((0, 0.8), (self.cfg.N, 0.25)):
                mk = Marker()
                mk.header.frame_id, mk.header.stamp = self.odom_frame, stamp
                mk.ns, mk.id = "obstacles", 2 * j + (k > 0)
                mk.type = Marker.CYLINDER
                ox, oy, r = obs[k, j]
                mk.action = Marker.ADD if ox < FAR / 2 else Marker.DELETE
                mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = float(ox), float(oy), 0.1
                mk.pose.orientation.w = 1.0
                mk.scale.x = mk.scale.y = float(2 * max(r, 0.01))
                mk.scale.z = 0.2
                mk.color.r, mk.color.g, mk.color.b, mk.color.a = 1.0, 0.3 * (k > 0), 0.0, alpha
                arr.markers.append(mk)
        self.marker_pub.publish(arr)


def main():
    # Keep the ROS context alive on Ctrl-C so the zero command below still goes out (with
    # rclpy's own handler the context is already shut down and the publish is lost).
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()
    node = NmpcNode()

    def on_term(*_):  # ros2 launch escalates to SIGTERM 5 s after Ctrl-C
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, on_term)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        for _ in range(5):
            try:
                node.publish_cmd(0.0, 0.0)
            except Exception:  # noqa: BLE001
                break
            time.sleep(0.02)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

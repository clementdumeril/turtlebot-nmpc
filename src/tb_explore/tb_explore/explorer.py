"""Autonomous exploration with photo capture, for a 3D reconstruction.

Loop: spin in place taking `captures_per_spin` photos -> pick the best reachable frontier on
/map -> turn to face the path -> send the path to the NMPC (/tb_nmpc/path) -> wait for
/follow/done -> spin and capture again ... until no frontier is left, then drive home.

Subscribes  /map (OccupancyGrid), /odom, /follow/done, TF map -> base_link
Publishes   /tb_nmpc/path (Path, map frame), /cmd_vel (only while turning in place),
            ~/frontiers, ~/goal (MarkerArray), ~/state (String)
Services    ~/start, ~/stop (Trigger); calls /tb_nmpc/stop and /auto_capture/capture
"""
from __future__ import annotations

import math
import os
import signal
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, Twist, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as PathMsg
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .grid import Grid, plan_to_frontier, plan_to_point


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class Explorer(Node):
    """Frontier-exploration state machine: IDLE -> SPIN -> PLAN -> ALIGN -> DRIVE -> SPIN ... -> HOME -> DONE."""

    def __init__(self):
        """Declare parameters, set up subscriptions, publishers, service clients and the 10 Hz tick."""
        super().__init__("explorer")
        for name, default in (
            ("map_topic", "/map"), ("map_frame", "map"), ("base_frame", "base_link"), ("odom_topic", "/odom"),
            ("path_topic", "/tb_nmpc/path"), ("nmpc_stop", "/tb_nmpc/stop"), ("capture_service", "/auto_capture/capture"),
            ("cmd_vel_topic", "/cmd_vel"), ("cmd_vel_type", "auto"),
            ("hard_radius", 0.17),      # [m] no path closer than this to an obstacle (robot 0.105 + margin)
            ("soft_radius", 0.40),      # [m] paths prefer to stay this far from obstacles
            ("min_frontier", 8),        # cells (8 x 5 cm = 0.4 m of frontier)
            ("size_weight", 0.5),       # score = path length - size_weight * frontier length
            ("min_goal_dist", 0.35),    # [m] frontiers closer than this are left to the spin
            ("blacklist_radius", 0.35),
            ("goal_timeout", 60.0),     # [s] give up on a goal after this
            ("max_time", 900.0),        # [s] whole exploration
            ("captures_per_spin", 12), ("spin_speed", 0.6), ("settle", 0.6),
            ("capture_at_goals", True), ("return_home", True), ("autostart", False),
            ("chair_radius", 0.32),     # [m] thin obstacles (chair columns) planned around as chair bases
            ("chair_cells", 6),
            ("capture_spacing", 0.3),   # [m] also take a photo every this much driving (0 = off): the
                                        # spin photos share one point, the reconstruction needs parallax
        ):
            self.declare_parameter(name, default)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.g = g
        self.grid = None
        self.yaw_odom = None
        self.state = "IDLE"
        self.t_state = 0.0
        self.blacklist: list = []
        self.home = None
        self.goal = None
        self.path_xy = None
        self.done_after = None
        self.done_flag = False
        self.cmd_pub = None
        self.n_goals = 0
        self.t_start = 0.0
        self.no_frontier = 0
        self.last_shot = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Volatile subscriber: compatible with a latched /map (simple_mapper) and with a plain one
        # (cartographer_occupancy_grid_node publishes it again every second anyway).
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, g("map_topic"), self.on_map, map_qos)
        self.create_subscription(Odometry, g("odom_topic"), self.on_odom, qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/follow/done", self.on_done, latched)
        self.saved = 0
        self.create_subscription(Int32, "/auto_capture/saved", self.on_saved, 10)
        self.path_pub = self.create_publisher(PathMsg, g("path_topic"), 1)
        self.marker_pub = self.create_publisher(MarkerArray, "~/frontiers", 1)
        self.state_pub = self.create_publisher(String, "~/state", 1)
        self.stop_cli = self.create_client(Trigger, g("nmpc_stop"))
        self.capture_cli = self.create_client(Trigger, g("capture_service"))
        self.create_service(Trigger, "~/start", self.on_start)
        self.create_service(Trigger, "~/stop", self.on_stop)
        self.create_timer(0.1, self.tick)
        if g("autostart"):
            self.create_timer(3.0, self._autostart)

    # ---------- inputs ----------
    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_map(self, msg: OccupancyGrid):
        """Keep the latest /map as a Grid (the origin rotation is assumed to be zero)."""
        info = msg.info
        data = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
        self.grid = Grid(data, info.resolution, info.origin.position.x, info.origin.position.y)

    def on_odom(self, msg: Odometry):
        """Latest odometry yaw [rad], used to track the turns in place (SPIN and ALIGN)."""
        self.yaw_odom = yaw_of(msg.pose.pose.orientation)

    def on_done(self, msg: Bool):
        """/follow/done from the NMPC. Ignored until done_after, so a latched True from the previous
        path does not end the new one immediately."""
        if msg.data and self.done_after is not None and self.now() > self.done_after:
            self.done_flag = True

    def on_saved(self, msg: Int32):
        """Photo count from /auto_capture/saved; a rise means the last requested photo is on disk."""
        self.saved = msg.data

    def robot_pose(self):
        """(x, y, yaw) of base_link in the map frame from the latest TF, or None if not available."""
        try:
            tf = self.tf_buffer.lookup_transform(self.g("map_frame"), self.g("base_frame"), Time(), Duration(seconds=0.0))
        except Exception:  # noqa: BLE001
            return None
        t = tf.transform.translation
        return t.x, t.y, yaw_of(tf.transform.rotation)

    # ---------- services ----------
    def _autostart(self):
        """Timer (autostart:=true): start once the map, odometry and TF are all available, only once."""
        if self.state == "IDLE" and self.grid is not None and self.robot_pose() is not None and self.t_start == 0.0:
            self.on_start(None, Trigger.Response())

    def on_start(self, _req, resp):
        """~/start: record home, reset the blacklist and goal count, and begin with a capture spin.

        Fails with the list of missing inputs if /map, /odom or the map -> base_link TF is not there yet.
        """
        pose = self.robot_pose()
        if pose is None or self.grid is None or self.yaw_odom is None:
            missing = [n for n, ok in (("/map", self.grid is not None), ("/odom", self.yaw_odom is not None),
                                       ("TF map -> base_link", pose is not None)) if not ok]
            resp.success, resp.message = False, "waiting for " + ", ".join(missing)
            return resp
        self.setup_cmd_pub()
        self.home = pose[:2]
        self.blacklist = []
        self.n_goals = 0
        self.t_start = self.now()
        self.begin_spin("PLAN")
        resp.success, resp.message = True, "exploring"
        self.get_logger().info("exploration started")
        return resp

    def on_stop(self, _req, resp):
        """~/stop: stop the NMPC and the robot, go back to IDLE."""
        self.stop_nmpc()
        self.set_state("IDLE")
        self.cmd(0.0, 0.0)
        resp.success, resp.message = True, "stopped"
        return resp

    def setup_cmd_pub(self):
        """Create the cmd_vel publisher once, as Twist or TwistStamped.

        With cmd_vel_type auto, use the type the base already subscribes with; if nobody
        subscribes yet, guess from ROS_DISTRO (Twist on Humble and Iron, stamped after).
        """
        if self.cmd_pub is not None:
            return
        topic, kind = self.g("cmd_vel_topic"), self.g("cmd_vel_type")
        if kind == "auto":
            types = {s.topic_type for s in self.get_subscriptions_info_by_topic(topic)}
            kind = "stamped" if "geometry_msgs/msg/TwistStamped" in types else "twist" if types else (
                "twist" if os.environ.get("ROS_DISTRO", "") in ("humble", "iron") else "stamped")
        self.stamped = kind == "stamped"
        self.cmd_pub = self.create_publisher(TwistStamped if self.stamped else Twist, topic, 10)

    def cmd(self, v: float, w: float):
        """Publish v [m/s], w [rad/s] on cmd_vel; does nothing before the first start."""
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

    def stop_nmpc(self):
        """Ask the NMPC to stop following its path (non-blocking, skipped if the service is down)."""
        if self.stop_cli.service_is_ready():
            self.stop_cli.call_async(Trigger.Request())

    # ---------- state machine ----------
    def set_state(self, s: str):
        """Switch state, restart the state timer t_state, and publish the name on ~/state."""
        self.state = s
        self.t_state = self.now()
        self.state_pub.publish(String(data=s))
        self.get_logger().info(f"state {s}")

    def begin_spin(self, after: str):
        """Turn in place in captures_per_spin steps, one photo after each step."""
        self.spin_after = after
        self.spin_k = 0
        self.spin_yaw0 = self.yaw_odom
        self.set_state("SPIN")
        self.spin_phase, self.t_phase = "settle", self.now()   # first photo where we stand, once still

    def tick(self):
        """10 Hz state-machine step: enforce max_time, then run tick_<state> (tick_spin, tick_plan ...).

        Nothing runs in IDLE and DONE; once the budget is used up, every state but HOME goes home.
        """
        if self.state in ("IDLE", "DONE"):
            return
        if self.now() - self.t_start > self.g("max_time") and self.state != "HOME":
            self.get_logger().info("time budget used up")
            self.go_home_or_finish()
            return
        getattr(self, "tick_" + self.state.lower())()

    def tick_spin(self):
        """SPIN: cycle settle -> shoot -> wait -> rotate for each of the captures_per_spin headings.

        Headings are 360 / n degrees apart from the yaw at the start of the spin. After the last
        photo the state goes to spin_after (PLAN). A missing camera never blocks the spin.
        """
        n = self.g("captures_per_spin")
        if self.spin_phase == "rotate":
            target = self.spin_yaw0 + self.spin_k * 2 * math.pi / n
            err = wrap(target - self.yaw_odom)
            if abs(err) < 0.04 or self.now() - self.t_phase > 8.0:  # reached, or gave up after 8 s
                self.cmd(0.0, 0.0)
                self.spin_phase, self.t_phase = "settle", self.now()
            else:
                # proportional turn rate, at least 0.25 rad/s, at most spin_speed
                w = math.copysign(min(self.g("spin_speed"), max(0.25, 1.5 * abs(err))), err)
                self.cmd(0.0, w)
        elif self.spin_phase == "settle":
            self.cmd(0.0, 0.0)
            if self.now() - self.t_phase > self.g("settle"):
                self.spin_phase = "shoot"
        elif self.spin_phase == "shoot":
            self.saved_before = self.saved
            self.retried = False
            self.capturing = self.capture_cli.service_is_ready()
            if self.capturing:
                self.capture_cli.call_async(Trigger.Request())
            elif self.spin_k == 0:
                self.get_logger().warn("auto_capture is not running: exploring without photos")
            self.spin_k += 1
            self.spin_phase, self.t_phase = "wait", self.now()
        elif self.spin_phase == "wait":  # stay still until the photo is saved
            saved = not self.capturing or self.saved > self.saved_before
            if not saved and not self.retried and self.now() - self.t_phase > 1.5:
                self.retried = True  # the camera may not have been streaming yet
                self.capture_cli.call_async(Trigger.Request())
            if saved or self.now() - self.t_phase > 3.0:
                if not saved:
                    self.get_logger().warn("no photo saved within 3 s, turning on")
                if self.spin_k >= n:
                    self.set_state(self.spin_after)
                else:
                    self.spin_phase, self.t_phase = "rotate", self.now()

    def tick_plan(self):
        """PLAN: pick the best reachable frontier on the current map and start driving to it.

        No frontier is accepted as final only after 3 tries, 1.5 s apart (new maps); then go home.
        """
        if self.no_frontier and self.now() - self.t_state < 1.5:  # wait for a newer map between retries
            return
        pose = self.robot_pose()
        if pose is None or self.grid is None:
            return
        if int(self.grid.free.sum()) < 200:  # 0.5 m2: the map is not built yet
            if self.now() - self.t_state > 5.0:
                self.get_logger().warn("map still (nearly) empty: is the mapper receiving /scan?", throttle_duration_sec=5.0)
            return
        t0 = time.perf_counter()
        plan, clusters = plan_to_frontier(
            self.grid, pose[:2], self.g("hard_radius"), self.g("soft_radius"), self.g("min_frontier"),
            self.g("min_goal_dist"), self.g("size_weight"), self.blacklist, self.g("blacklist_radius"),
            chair_radius=self.g("chair_radius"), chair_cells=self.g("chair_cells"))
        self.publish_markers(clusters, plan.goal_xy if plan else None)
        if plan is None:
            # Confirm on later maps (the mapper publishes at 1 Hz) before calling it finished.
            self.no_frontier += 1
            if self.no_frontier < 3:
                self.t_state = self.now()
                return
            self.get_logger().info(f"no reachable frontier left ({len(clusters)} unreachable), "
                                   f"{self.n_goals} goals visited")
            self.go_home_or_finish()
            return
        self.no_frontier = 0
        self.get_logger().info(f"goal {self.n_goals + 1}: ({plan.goal_xy[0]:.2f}, {plan.goal_xy[1]:.2f}), "
                               f"path {plan.length:.2f} m, frontier {plan.cluster_size} cells, "
                               f"planned in {time.perf_counter() - t0:.2f} s")
        self.goal = plan.goal_xy
        self.start_path(plan.path_xy, "DRIVE")

    def start_path(self, xy, drive_state: str):
        """Face the first leg of the path, then hand the path to the NMPC."""
        self.path_xy = xy
        self.drive_state = drive_state
        pose = self.robot_pose()
        k = min(len(xy) - 1, 1)
        heading = math.atan2(xy[k, 1] - pose[1], xy[k, 0] - pose[0])
        # the turn is measured in the map frame but tracked with the odometry yaw
        self.align_target = self.yaw_odom + wrap(heading - pose[2])
        self.set_state("ALIGN")

    def tick_align(self):
        """ALIGN: turn in place to within 0.15 rad of the path (10 s max), then publish the path
        to the NMPC and go to drive_state (DRIVE or HOME)."""
        err = wrap(self.align_target - self.yaw_odom)
        if abs(err) > 0.15 and self.now() - self.t_state < 10.0:
            self.cmd(0.0, math.copysign(min(self.g("spin_speed"), max(0.25, 1.5 * abs(err))), err))
            return
        self.cmd(0.0, 0.0)
        self.publish_path(self.path_xy)
        self.done_flag = False
        self.done_after = self.now() + 0.3  # ignore /follow/done messages about the previous path
        self.set_state(self.drive_state)

    def capture_on_the_way(self):
        """Request a photo every capture_spacing metres driven, for parallax between viewpoints."""
        spacing = self.g("capture_spacing")
        pose = self.robot_pose()
        if spacing <= 0 or pose is None or not self.capture_cli.service_is_ready():
            return
        if self.last_shot is None or math.hypot(pose[0] - self.last_shot[0], pose[1] - self.last_shot[1]) >= spacing:
            self.capture_cli.call_async(Trigger.Request())
            self.last_shot = pose[:2]

    def tick_drive(self):
        """DRIVE: the NMPC follows the path. On /follow/done spin and capture (or plan again);
        after goal_timeout stop the NMPC and replan. Either way the goal is blacklisted."""
        self.capture_on_the_way()
        if self.done_flag:
            self.n_goals += 1
            self.blacklist.append(self.goal)  # reached: do not pick a frontier here again
            if self.g("capture_at_goals"):
                self.begin_spin("PLAN")
            else:
                self.set_state("PLAN")
        elif self.now() - self.t_state > self.g("goal_timeout"):
            self.get_logger().warn("goal timed out, blacklisting it")
            self.stop_nmpc()
            self.blacklist.append(self.goal)
            self.set_state("PLAN")

    def go_home_or_finish(self):
        """Drive back to the start pose if return_home, it is over 0.3 m away and reachable; else finish."""
        pose = self.robot_pose()
        if self.g("return_home") and pose is not None and self.home is not None and self.grid is not None \
                and math.hypot(pose[0] - self.home[0], pose[1] - self.home[1]) > 0.3:
            xy = plan_to_point(self.grid, pose[:2], self.home, self.g("hard_radius"), self.g("soft_radius"),
                               chair_radius=self.g("chair_radius"), chair_cells=self.g("chair_cells"))
            if xy is not None:
                self.get_logger().info("driving home")
                self.goal = self.home
                self.start_path(xy, "HOME")
                return
        self.finish()

    def tick_home(self):
        """HOME: finish when the NMPC reports done or after goal_timeout."""
        if self.done_flag or self.now() - self.t_state > self.g("goal_timeout"):
            self.finish()

    def finish(self):
        """Stop the NMPC and the robot, log the summary, go to DONE."""
        self.stop_nmpc()
        self.cmd(0.0, 0.0)
        self.get_logger().info(f"exploration finished: {self.n_goals} goals in {self.now() - self.t_start:.0f} s")
        self.set_state("DONE")

    # ---------- output ----------
    def publish_path(self, xy):
        """Publish (n, 2) map-frame points as a nav_msgs/Path on path_topic (orientations left at identity)."""
        m = PathMsg()
        m.header.frame_id = self.g("map_frame")
        m.header.stamp = self.get_clock().now().to_msg()
        for x, y in xy:
            ps = PoseStamped()
            ps.header = m.header
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.w = 1.0
            m.poses.append(ps)
        self.path_pub.publish(m)

    def publish_markers(self, clusters, goal):
        """RViz markers on ~/frontiers: frontier cells as blue points, the goal as an orange sphere
        (deleted when goal is None)."""
        arr = MarkerArray()
        mk = Marker()
        mk.header.frame_id = self.g("map_frame")
        mk.header.stamp = self.get_clock().now().to_msg()
        mk.ns, mk.id, mk.type, mk.action = "frontiers", 0, Marker.POINTS, Marker.ADD
        mk.scale.x = mk.scale.y = self.grid.res
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.1, 0.5, 1.0, 0.9
        for cl in clusters:
            xs, ys = self.grid.world(cl[:, 0], cl[:, 1])
            mk.points += [Point(x=float(x), y=float(y), z=0.02) for x, y in zip(xs, ys)]
        arr.markers.append(mk)
        gm = Marker()
        gm.header = mk.header
        gm.ns, gm.id, gm.type = "goal", 1, Marker.SPHERE
        gm.action = Marker.ADD if goal else Marker.DELETE
        if goal:
            gm.pose.position.x, gm.pose.position.y, gm.pose.position.z = goal[0], goal[1], 0.05
        gm.pose.orientation.w = 1.0
        gm.scale.x = gm.scale.y = gm.scale.z = 0.12
        gm.color.r, gm.color.g, gm.color.b, gm.color.a = 1.0, 0.4, 0.0, 1.0
        arr.markers.append(gm)
        self.marker_pub.publish(arr)


def main():
    """Spin the explorer; on Ctrl-C or SIGTERM send a few zero velocities before shutting down.

    rclpy's own signal handlers are disabled so the node is still alive to publish the stop.
    """
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()
    node = Explorer()

    def on_term(*_):
        """Turn SIGTERM (sent by ros2 launch) into a KeyboardInterrupt."""
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, on_term)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5):
            try:
                node.cmd(0.0, 0.0)
            except Exception:  # noqa: BLE001
                break
            time.sleep(0.02)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

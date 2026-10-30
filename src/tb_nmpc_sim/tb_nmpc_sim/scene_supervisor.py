"""Webots plugin (runs inside the TurtleBot3 driver, robot has supervisor TRUE).

Places the static obstacles of the scenario (env TB_SCENARIO), walks the pedestrian across the
path when the robot gets close, and publishes the ground truth at every simulation step:
    /sim/ground_truth  Float64MultiArray [t, x, y, yaw, ped_x, ped_y, ped_vx, ped_vy, ped_state]
ped_state: 0 waiting, 1 walking, 2 gone. /sim/record (String): a file name on the Webots host
starts a video, an empty string stops it.
"""
import math
import os

import numpy as np
import rclpy
from std_msgs.msg import Float64MultiArray, String

from tb_nmpc_sim import scenarios

HIDDEN = (40.0, 40.0)


class SceneSupervisor:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot
        self.scenario = os.environ.get("TB_SCENARIO", "clear")
        try:
            rclpy.init(args=None)
        except RuntimeError:
            pass
        self.node = rclpy.create_node("scene_supervisor")
        self.pub = self.node.create_publisher(Float64MultiArray, "/sim/ground_truth", 10)
        self.node.create_subscription(String, "/sim/record", self.on_record, 1)
        self.me = self.robot.getSelf()
        self.path = scenarios.path()
        self.s = 0.0
        placed = {d: (x, y) for d, x, y, _ in scenarios.static_obstacles(self.scenario, self.path)}
        for d in ("OBS1", "OBS2"):
            n = self.robot.getFromDef(d)
            if n is not None:
                z = n.getField("translation").getSFVec3f()[2]
                n.getField("translation").setSFVec3f([*placed.get(d, HIDDEN), z])
        self.ped_node = self.robot.getFromDef("PED")
        self.ped = scenarios.pedestrian(self.scenario, self.path)
        self.ped_state = 0
        self.ped_pos = np.array(self.ped[0]) if self.ped else np.array(HIDDEN)
        self.ped_t0 = 0.0
        self.set_ped(self.ped_pos)
        self.node.get_logger().info(f"scenario '{self.scenario}': static {list(placed)}, "
                                    f"pedestrian {'yes' if self.ped else 'no'}")

    def set_ped(self, xy):
        if self.ped_node is not None:
            f = self.ped_node.getField("translation")
            f.setSFVec3f([float(xy[0]), float(xy[1]), f.getSFVec3f()[2]])

    def on_record(self, msg):
        if msg.data:
            self.robot.movieStartRecording(msg.data, 1280, 720, 0, 90, 1, False)
            self.node.get_logger().info(f"recording {msg.data}")
        else:
            self.robot.movieStopRecording()

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        t = self.robot.getTime()
        x, y, _ = self.me.getPosition()
        R = self.me.getOrientation()
        yaw = math.atan2(R[3], R[0])
        self.s, _ = self.path.project((x, y), self.s)
        vel = np.zeros(2)
        if self.ped is not None:
            start, end, v, s_trig = self.ped
            if self.ped_state == 0 and self.s >= s_trig:
                self.ped_state, self.ped_t0 = 1, t
            if self.ped_state == 1:
                self.ped_pos = np.asarray(start) + v * (t - self.ped_t0)
                vel = v
                if np.dot(self.ped_pos - end, v) >= 0:
                    self.ped_state, self.ped_pos = 2, np.array(HIDDEN)
                self.set_ped(self.ped_pos)
        self.pub.publish(Float64MultiArray(data=[t, x, y, yaw, float(self.ped_pos[0]), float(self.ped_pos[1]),
                                                 float(vel[0]), float(vel[1]), float(self.ped_state)]))

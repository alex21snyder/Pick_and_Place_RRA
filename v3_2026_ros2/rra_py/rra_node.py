#!/usr/bin/env python3
"""ROS 2 node around the resolved rates algorithm.

In : /goal_pose  (geometry_msgs/PoseStamped, frame base_link, position in METERS, orientation as quaternion)
Out: /joint_states (sensor_msgs/JointState, radians)  -> robot_state_publisher -> RViz

Parameter demo (default true): plays Motions 1-3 from main_06.ino once at startup.
One RRA iteration is published every delta_time (28 ms), same pace as the real arm's loop.
"""

import os
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from rra import MOTIONS, RRAParams, home_steps, rra_steps  # noqa: E402

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
PAUSE_S = 1.0  # pause between demo motions


def quat_to_matrix(x, y, z, w):
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class RraNode(Node):
    def __init__(self):
        super().__init__("rra_node")
        self.declare_parameter("demo", True)
        self.params = RRAParams()
        self.q = np.zeros(6)       # degrees, like state.q in the Arduino code
        self.gen = None            # active rra_steps generator
        self.queue = []            # pending (name, Xd_mm, Rd, param overrides)
        self.wait_until = 0.0

        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.on_goal, 10)
        self.timer = self.create_timer(self.params.delta_time, self.tick)
        self.publish()

        if self.get_parameter("demo").value:
            self.queue = list(MOTIONS) + [("Motion 4 (home)", None, None, {})]
            self.wait_until = self.now() + 8.0  # give RViz time to open
            self.get_logger().info("Demo: playing Motions 1-3")
        else:
            self.get_logger().info("Waiting for a PoseStamped on /goal_pose")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_goal(self, msg):
        p, o = msg.pose.position, msg.pose.orientation
        Xd = np.array([p.x, p.y, p.z]) * 1000.0  # m -> mm
        Rd = quat_to_matrix(o.x, o.y, o.z, o.w)
        self.queue = [("goal_pose", Xd, Rd, {})]
        self.gen = None
        self.wait_until = 0.0
        self.get_logger().info(f"New goal (mm): {np.round(Xd, 1).tolist()}")

    def start(self, item):
        name, Xd, Rd, overrides = item
        for k, v in overrides.items():
            setattr(self.params, k, v)
        if Xd is None:  # Motion 4: back home, joint-space line
            self.gen = home_steps(self.q, delta_time=self.params.delta_time)
        else:
            self.gen = rra_steps(self.q, Xd, Rd, self.params)
        self.current = name
        self.get_logger().info(f"{name}: running")

    def tick(self):
        if self.gen is None:
            if self.queue and self.now() >= self.wait_until:
                self.start(self.queue.pop(0))
            return
        try:
            q, posE, thetaE, done = next(self.gen)
        except StopIteration:
            self.gen = None
            self.wait_until = self.now() + PAUSE_S
            return
        self.q = q
        self.publish()
        if done:
            if self.current.startswith("Motion 4"):
                self.q = np.zeros(6)
                self.publish()
                self.get_logger().info("Motion 4: home. done")
            else:
                self.get_logger().info(
                    f"{self.current}: converged (posE={posE:.2f} mm, thetaE={thetaE:.3f} rad) "
                    f"q={np.round(q, 2).tolist()} deg")
            self.gen = None
            self.wait_until = self.now() + PAUSE_S

    def publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS
        msg.position = [float(v) for v in np.radians(self.q)]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = RraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()

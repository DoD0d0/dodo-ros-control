#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist

import numpy as np


class DodoController(Node):
    def __init__(self):
        super().__init__("dodo_controller")

        # -------------------------
        # Parameters
        # -------------------------
        self.declare_parameter(
            "command_topic",
            "/dodo/joint_group_position_controller/commands",
        )
        self.declare_parameter(
            "joint_state_topic",
            "/joint_states",
        )
        self.declare_parameter(
            "base_state_topic",
            "/dodo/base_state",
        )
        self.declare_parameter(
            "control_rate",
            200.0,
        )
        self.declare_parameter(
            "num_joints",
            12,
        )

        self.command_topic = (
            self.get_parameter("command_topic")
            .get_parameter_value()
            .string_value
        )
        self.joint_state_topic = (
            self.get_parameter("joint_state_topic")
            .get_parameter_value()
            .string_value
        )
        self.base_state_topic = (
            self.get_parameter("base_state_topic")
            .get_parameter_value()
            .string_value
        )
        self.control_rate = (
            self.get_parameter("control_rate")
            .get_parameter_value()
            .double_value
        )
        self.num_joints = (
            self.get_parameter("num_joints")
            .get_parameter_value()
            .integer_value
        )

        # -------------------------
        # Internal state
        # -------------------------
        self.latest_joint_state = None
        self.latest_base_state = None

        # warn_once 대체 플래그
        self._warned_no_joint_state = False
        self._warned_no_base_state = False

        # -------------------------
        # ROS interfaces
        # -------------------------
        self.joint_state_sub = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            10,
        )

        self.base_state_sub = self.create_subscription(
            Twist,
            self.base_state_topic,
            self.base_state_callback,
            10,
        )

        self.command_pub = self.create_publisher(
            Float64MultiArray,
            self.command_topic,
            10,
        )

        self.timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop,
        )

        # -------------------------
        # Logging
        # -------------------------
        self.get_logger().info("DodoController initialized")
        self.get_logger().info(f"  Command topic      : {self.command_topic}")
        self.get_logger().info(f"  Joint state topic  : {self.joint_state_topic}")
        self.get_logger().info(f"  Base state topic   : {self.base_state_topic}")
        self.get_logger().info(f"  Control rate [Hz]  : {self.control_rate}")
        self.get_logger().info(f"  Number of joints   : {self.num_joints}")

    # -------------------------
    # Callbacks
    # -------------------------
    def joint_state_callback(self, msg: JointState):
        self.latest_joint_state = msg

    def base_state_callback(self, msg: Twist):
        self.latest_base_state = msg

    # -------------------------
    # Control loop
    # -------------------------
    def control_loop(self):
        if self.latest_joint_state is None:
            if not self._warned_no_joint_state:
                self.get_logger().warn("No joint state received yet")
                self._warned_no_joint_state = True
            return

        if self.latest_base_state is None:
            if not self._warned_no_base_state:
                self.get_logger().warn("No base state received yet")
                self._warned_no_base_state = True
            return

        # Example: hold current joint positions
        current_positions = np.array(
            self.latest_joint_state.position[: self.num_joints],
            dtype=np.float64,
        )

        cmd = Float64MultiArray()
        cmd.data = current_positions.tolist()

        self.command_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = DodoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import math
from typing import List, Optional

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray


class DodoController(Node):
    """
    Minimal skeleton for a Dodo controller node.

    Responsibilities:
    - Subscribe to joint states and base state from simulation
    - Run a control loop at a fixed frequency
    - Publish joint position commands (or torque/velocity) to Isaac Sim

    Replace the TODO sections with your own control / RL policy logic.
    """

    def __init__(self) -> None:
        super().__init__("dodo_controller")

        # Parameters (can be overridden via ROS2 parameters or launch files)
        self.declare_parameter("command_topic", "/dodo/joint_group_position_controller/commands")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("base_state_topic", "/dodo/base_state")
        self.declare_parameter("control_rate_hz", 200.0)
        self.declare_parameter("num_joints", 12)

        self.command_topic = self.get_parameter("command_topic").get_parameter_value().string_value
        self.joint_state_topic = self.get_parameter("joint_state_topic").get_parameter_value().string_value
        self.base_state_topic = self.get_parameter("base_state_topic").get_parameter_value().string_value
        self.control_rate_hz = self.get_parameter("control_rate_hz").get_parameter_value().double_value
        self.num_joints = int(self.get_parameter("num_joints").get_parameter_value().double_value)

        # State buffers
        self.latest_joint_state: Optional[JointState] = None
        self.latest_odom: Optional[Odometry] = None

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            10,
        )
        self.base_state_sub = self.create_subscription(
            Odometry,
            self.base_state_topic,
            self.base_state_callback,
            10,
        )

        # Publisher
        self.command_pub = self.create_publisher(
            Float64MultiArray,
            self.command_topic,
            10,
        )

        # Control loop timer
        period = 1.0 / self.control_rate_hz
        self.control_timer = self.create_timer(period, self.control_loop)

        self.get_logger().info("DodoController initialized")
        self.get_logger().info(f"  Command topic      : {self.command_topic}")
        self.get_logger().info(f"  Joint state topic  : {self.joint_state_topic}")
        self.get_logger().info(f"  Base state topic   : {self.base_state_topic}")
        self.get_logger().info(f"  Control rate [Hz]  : {self.control_rate_hz}")
        self.get_logger().info(f"  Number of joints   : {self.num_joints}")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def joint_state_callback(self, msg: JointState) -> None:
        """Store the latest joint state."""
        self.latest_joint_state = msg

    def base_state_callback(self, msg: Odometry) -> None:
        """Store the latest base / body state."""
        self.latest_odom = msg

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def control_loop(self) -> None:
        """
        Main control loop.

        This function is called at a fixed rate (control_rate_hz).
        It computes commands based on the latest state and publishes them.
        """
        if self.latest_joint_state is None:
            self.get_logger().warn_once("No joint state received yet")
            return

        # Optional: wait for base state as well
        if self.latest_odom is None:
            self.get_logger().warn_once("No base state received yet")
            # You may return here if base state is required for control
            # return

        # TODO: Fill observation vector from self.latest_joint_state and self.latest_odom
        observation = self.build_observation()

        # TODO: Call your controller / policy here
        # Example: joint_targets = self.simple_stand_policy(observation)
        joint_targets = self.simple_zero_policy()

        # Publish
        self.publish_joint_commands(joint_targets)

    # ------------------------------------------------------------------
    # Helper methods
    # ------------------------------------------------------------------

    def build_observation(self) -> List[float]:
        """
        Build an observation vector from the latest sensor data.

        Replace this with a structure that matches your controller or RL policy.
        """
        obs: List[float] = []

        # Joint positions and velocities
        if self.latest_joint_state is not None:
            obs.extend(self.latest_joint_state.position)
            obs.extend(self.latest_joint_state.velocity)

        # Base pose (position + orientation) and twist
        if self.latest_odom is not None:
            p = self.latest_odom.pose.pose.position
            q = self.latest_odom.pose.pose.orientation
            v = self.latest_odom.twist.twist

            obs.extend([p.x, p.y, p.z])
            obs.extend([q.x, q.y, q.z, q.w])
            obs.extend([v.linear.x, v.linear.y, v.linear.z])
            obs.extend([v.angular.x, v.angular.y, v.angular.z])

        return obs

    def simple_zero_policy(self) -> List[float]:
        """
        Very simple placeholder policy.

        Returns zero position offsets for all joints.
        Replace this with your own controller or RL policy output.
        """
        return [0.0 for _ in range(self.num_joints)]

    def publish_joint_commands(self, joint_targets: List[float]) -> None:
        """
        Publish joint position commands as Float64MultiArray.

        If your Isaac Sim side expects a different message type or topic,
        adapt this method accordingly.
        """
        if len(joint_targets) != self.num_joints:
            self.get_logger().warn(
                f"Size mismatch: expected {self.num_joints} joints, "
                f"got {len(joint_targets)}"
            )
            # Optionally clamp or resize here
            joint_targets = list(joint_targets[: self.num_joints])
            if len(joint_targets) < self.num_joints:
                joint_targets.extend(
                    [0.0] * (self.num_joints - len(joint_targets))
                )

        msg = Float64MultiArray()
        msg.data = joint_targets
        self.command_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DodoController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down DodoController")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


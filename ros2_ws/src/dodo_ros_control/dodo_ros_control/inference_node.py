# dodo_ros_control/inference_node.py

import rclpy
from rclpy.node import Node
import math
import time

from dodo_ros_control.can.can_mock import MockCANInterface


class InferenceNode(Node):
    def __init__(self):
        super().__init__("dodo_inference_node")

        # Parameters (hard-coded for now)
        self.num_joints = 6
        self.publish_rate_hz = 50.0

        # CAN interface (mock)
        self.can = MockCANInterface()
        self.can.connect()

        # Timer
        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self.control_loop
        )

        self.start_time = time.time()
        self.get_logger().info("Inference node started (Mock CAN mode)")

    def control_loop(self):
        """
        This simulates:
        observation -> policy -> torque
        """
        t = time.time() - self.start_time

        for joint_id in range(self.num_joints):
            # Fake torque signal (sine wave)
            torque = 0.5 * math.sin(t)

            self.can.send_torque_command(joint_id, torque)

    def shutdown(self):
        self.can.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = InferenceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
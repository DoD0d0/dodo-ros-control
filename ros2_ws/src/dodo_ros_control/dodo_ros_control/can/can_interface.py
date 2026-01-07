# dodo_ros_control/can/can_interface.py

import can
import time


class CANInterface:
    def __init__(self, channel="can0", bitrate=1000000):
        self.channel = channel
        self.bitrate = bitrate
        self.bus = None

    def connect(self):
        self.bus = can.interface.Bus(
            channel=self.channel,
            bustype="socketcan"
        )

    def send_torque_command(self, joint_id: int, torque: float):
        """
        Placeholder:
        - joint_id -> CAN ID mapping will be defined later
        - torque scaling will be defined later
        """
        msg = can.Message(
            arbitration_id=joint_id,
            data=[0x00] * 8,  # TODO: encode torque
            is_extended_id=False
        )
        self.bus.send(msg)

    def shutdown(self):
        if self.bus is not None:
            self.bus.shutdown()
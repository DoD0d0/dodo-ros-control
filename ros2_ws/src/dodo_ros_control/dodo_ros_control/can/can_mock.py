# dodo_ros_control/can/can_mock.py

class MockCANInterface:
    def __init__(self):
        pass

    def connect(self):
        print("[MockCAN] Connected")

    def send_torque_command(self, joint_id: int, torque: float):
        print(f"[MockCAN] joint={joint_id}, torque={torque:.3f}")

    def shutdown(self):
        print("[MockCAN] Shutdown")
# dodo_ros_control

ROS 2 Python package for running trained RL policies on the Dodo robot.

Responsibilities:
- Policy inference
- Observation construction
- Action to joint torque mapping
- Safety checks

## Nodes

- `dodo_inference_node`
  - entrypoint: `ros2 run dodo_ros_control inference_node`
  - publishes no ROS topics/services yet (uses mock CAN only)
  - hard-coded defaults:
    - `num_joints = 6`
    - `publish_rate_hz = 50.0`
- `dummy_node`
  - entrypoint: `ros2 run dodo_ros_control dummy_node`
  - placeholder (no topics/services)

## CAN Interface Defaults

`dodo_ros_control/can/can_interface.py`:
- `channel = can0`
- `bitrate = 1000000`

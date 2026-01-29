# dodo_mujoco_bridge

This package provides a **minimal ROS 2 ↔ MuJoCo bridge scaffold**
for sim-to-sim validation of control logic before sim-to-real deployment.

At the current stage, this package focuses on:
- ROS 2 package structure validation
- Node execution sanity checks
- Preparing the interface for future MuJoCo integration

## Current Status

- ROS 2 package builds successfully with `colcon`
- Example node (`hello_node`) runs correctly
- No MuJoCo dependency inside ROS container (host-side only)

## Available Nodes

```bash
ros2 run dodo_mujoco_bridge hello_node
```

This node only prints a startup message and spins.
It is intended as a **structural placeholder**.

### Node Details

- Node name: `dodo_mujoco_hello`
- Topics / services: none (placeholder)

## Intended Future Role

In later stages, this package will:
	•	Subscribe to MuJoCo joint states (sim2sim)
	•	Publish control commands (torque / position)
	•	Act as a bridge between:
	•	host-side MuJoCo simulation
	•	ROS 2 control pipeline (dodo_ros_control)

## Design Note

MuJoCo itself is not executed inside this ROS 2 package.
MuJoCo runs on the host (if it is macOS) side, while this package handles ROS 2 messaging only.

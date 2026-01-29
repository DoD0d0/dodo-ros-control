# dodo-ros-control

This repository provides a **ROS 2–based control interface for Dodo robot sim-to-real deployment**.

The goal of this repository is to **bridge trained reinforcement learning (RL) policies from simulation (e.g., PPO)** to real hardware by providing:

- ROS 2 control pipeline
- Hardware-agnostic CAN interface (mock ↔ real)
- Minimal and reproducible Docker-based development environment

This repository **does not focus on simulation or visualization**.
Instead, it focuses on **reliable control, inference, and hardware interfacing for sim2real transfer**.

---

## TL;DR

```bash
docker build -t dodo_ros_control:humble -f docker/Dockerfile .
./scripts/dodo_run.sh
cd /ros2_ws && colcon build
ros2 run dodo_ros_control inference_node
```

---

## Repository Structure

```text
dodo-ros-control/
├── docker/                 # Dockerfiles and container entrypoint
├── scripts/                # Convenience scripts (container run, attach)
├── isaac_sim/              # Isaac Sim launch and controller code
├── mujoco/                 # Host-side MuJoCo sim2sim validation (optional)
├── ros2_ws/                # ROS 2 workspace (mounted into container)
│   └── src/
│       └── dodo_ros_control/
│           ├── can/         # CAN interfaces (mock / real)
│           ├── config/      # Runtime configuration (backend selection)
│           ├── dummy_node.py
│           ├── inference_node.py
│           └── interface_node.py
├── models/                 # Trained RL policies (not tracked by git)
├── assets/                 # Robot-specific assets (URDF, calib, etc.)
├── docker-compose.yml
└── README.md
```

---

### Key Design Intent

- **can/**  
  Abstracts hardware communication.  
  `MockCANInterface` allows testing without hardware, while real CAN backends can be added without changing control logic.

- **inference_node**  
  Loads a trained policy and produces joint-level torque commands.

- **interface_node**  
  Acts as the boundary between control logic and hardware communication.

- **models/**  
  Stores trained RL policies from simulation (e.g., PPO).  
  This folder is intentionally excluded from version control.

---

## Development Environment

This repository is designed to be used **inside a Docker container**.

### Target Environment
- OS: Linux (Ubuntu recommended), WSL, macOS (GUI / visualization support is limited)
- ROS 2: Humble
- Python: 3.10
- Architecture: amd64 / arm64 (auto-detected)

### Versions (as configured)
- ROS base image: `osrf/ros:humble`
- ROS GUI image: `osrf/ros:humble-desktop-full`
- Isaac Sim image: `nvcr.io/nvidia/isaac-sim:5.1.0`

---

## Docker Usage

Build the base (headless) image:

```bash
docker build -t dodo_ros_control:humble -f docker/Dockerfile .
```

Build the GUI-enabled image (optional):

```bash
docker build -t dodo_ros_control:humble-gui -f docker/Dockerfile.gui .
```

Run or attach to the container:

```bash
./scripts/dodo_run.sh
```

Run with GUI support (optional):

```bash
./scripts/dodo_run.sh --gui
```

### Script Options
- `scripts/dodo_run.sh [--gui]`
  - `--gui`: uses `docker/Dockerfile.gui` image (`dodo_ros_control:humble-gui`)
- `scripts/run_isaac_sim.sh`
  - no CLI options; runs Isaac Sim GUI container
- `scripts/run_isaac_ros.sh`
  - no CLI options; attaches if container is running, otherwise starts a new one

---

## ROS 2 Workspace Usage

Inside the container, the ROS 2 workspace is mounted at:

`/ros2_ws`

Build and source the workspace:

```bash
cd /ros2_ws
colcon build
source install/setup.bash
```

List available packages:

```bash
ros2 pkg list | grep dodo
```

Run example nodes:

```bash
ros2 run dodo_ros_control dummy_node
ros2 run dodo_ros_control inference_node
```

### ROS 2 Nodes, Topics, Services

#### dodo_ros_control
- Nodes
  - `dodo_inference_node` (console: `ros2 run dodo_ros_control inference_node`)
  - `dummy_node` (console: `ros2 run dodo_ros_control dummy_node`)
- Topics / services
  - None defined yet (mock CAN only; no ROS pub/sub/service calls in code)
- Parameters / options (current hard-coded defaults)
  - `num_joints = 6`
  - `publish_rate_hz = 50.0`
  - CAN defaults in interface: `channel = can0`, `bitrate = 1000000`

#### dodo_mujoco_bridge
- Nodes
  - `dodo_mujoco_hello` (console: `ros2 run dodo_mujoco_bridge hello_node`)
- Topics / services
  - None (placeholder node only)

---

## Isaac Sim (optional)

Start Isaac Sim GUI container:

```bash
./scripts/run_isaac_sim.sh
```

Example app script (runs in Isaac Sim Python environment):
- `isaac_sim/launch/run_dodo_rl.py`

Key options (current hard-coded defaults):
- `policy_name = "stand"`
- `action_scale = 0.5`
- `headless = False`
- `dodo_usd_path` is hard-coded in the script

ROS topics/services are not used in the Isaac Sim pipeline in this repo.

---

## sim2real Workflow

The intended sim-to-real pipeline is:

1. Train a control policy in simulation (e.g., PPO)
2. Export the trained policy (weights, config)
3. Load the policy inside `inference_node`
4. Convert policy output to joint torque commands
5. Send torques via CAN interface to real hardware

This repository covers **steps 3–5**.

---

## Policy and Asset Placement

### Trained Policies
Place trained RL policies in:

`models/`

Example:
- Neural network weights
- Normalization statistics
- Policy configuration files

### Robot Assets
Place robot-specific files in:
`assets/`

Example:
- Joint ordering definitions
- Hardware calibration parameters
- Motor limits and scaling
- Robot-specific configuration files

The control pipeline is designed so that policy logic does not depend on hardware details.

---

## Troubleshooting

### `ros2: command not found`
Make sure the ROS environment is sourced:

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
```

### Package builds but executables are missing
Clean and rebuild the workspace:

```bash
rm -rf build install log
colcon build
```

### Docker container attaches but environment is broken
Remove and recreate the container:

```bash
docker rm -f dodo_ros_control
./scripts/dodo_run.sh
```

---

## Notes and Scope

- This repository intentionally avoids simulation and visualization tooling.
- CAN backends are designed to be swappable without modifying control logic.
- Real hardware communication should be implemented inside `can/`.

This repository is intended to be **a stable sim2real control backbone**, not an experiment playground.

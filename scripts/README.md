# Utility Scripts

Convenience scripts for Docker and simulation runs.

- `dodo_run.sh` starts or attaches to the ROS 2 Docker container
- `run_isaac_sim.sh` runs Isaac Sim with repo defaults
- `run_isaac_ros.sh` launches ROS 2 nodes for Isaac Sim

## Options

- `dodo_run.sh [--gui]`
  - `--gui` runs the GUI image (`dodo_ros_control:humble-gui`)
- `run_isaac_sim.sh`
  - no CLI options; runs Isaac Sim GUI container (`nvcr.io/nvidia/isaac-sim:5.1.0`)
- `run_isaac_ros.sh`
  - no CLI options; attaches to existing container or starts a new one

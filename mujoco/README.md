# MuJoCo sim2sim (Dodo)

This directory contains **host-side MuJoCo simulation code**
used for sim-to-sim validation of trained control policies
before sim-to-real deployment.

MuJoCo is executed **outside Docker**, using a local Python virtual environment.

## Environment Setup

Activate the MuJoCo virtual environment:

```bash
cd mujoco
source env/bin/activate
```

Verify installation:

```bash
python -c "import mujoco; print(mujoco.__version__)"
```

## Directory Structure

- `env/`
Python virtual environment (not tracked)
- `models/`
Exported policies (ONNX, weights, stats)
- `scripts/`
Standalone MuJoCo test scripts
- `sims/`
MuJoCo XML models and simulation helpers

## Design Note

This MuJoCo setup is intentionally decoupled from ROS 2.
ROS integration is handled separately via dodo_mujoco_bridge.
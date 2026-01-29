# Isaac Sim (Sim2Sim)

Isaac Sim launch scripts and controller code for sim-to-sim validation.

Includes:
- `launch/` for running Isaac Sim experiments
- `controllers/` for policy inference and control glue

Run helpers live in `scripts/` (e.g., `run_isaac_sim.sh`).

## Versions
- Isaac Sim container image: `nvcr.io/nvidia/isaac-sim:5.1.0`

## How to Run

Start Isaac Sim GUI container:

```bash
./scripts/run_isaac_sim.sh
```

Example Isaac Sim app script:
- `isaac_sim/launch/run_dodo_rl.py`

## Controller Options (current)

`isaac_sim/controllers/dodo_rl_controller.py`:
- `policy_name` (default: `stand`)
  - policy path: `models/policies/<policy_name>/policy.pt`
- `action_scale` (default: `0.5`)
- `JOINT_NAMES` (8 joints, hard-coded)

`isaac_sim/launch/run_dodo_rl.py`:
- `headless = False`
- `dodo_usd_path` is hard-coded; adjust if your local asset path differs

## ROS Topics / Services
- None (Isaac Sim pipeline is standalone in this repo)

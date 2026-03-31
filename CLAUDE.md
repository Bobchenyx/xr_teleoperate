# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**xr_teleoperate** is a teleoperation framework for Unitree humanoid robots (G1, H1, H1_2) using XR devices (Apple Vision Pro, PICO 4, Meta Quest 3). It captures user hand/controller poses via WebXR, solves inverse kinematics, and sends joint commands to the robot over DDS (CycloneDDS via `unitree_sdk2_python`). It also supports data recording for imitation learning.

## Environment Setup

```bash
# Conda environment (Python 3.10)
conda create -n tv python=3.10 pinocchio=3.1.0 numpy=1.26.4 -c conda-forge
conda activate tv

# Clone and init submodules
git submodule update --init --depth 1

# Install submodules (from repo root)
pip install -e teleop/teleimager --no-deps
pip install -e teleop/televuer

# Install additional deps
pip install -r requirements.txt

# External dependency (not in this repo)
pip install -e /path/to/unitree_sdk2_python
```

## Running

All commands run from `teleop/` directory:

```bash
# Simulation mode (G1_29 + Dex3 hand)
python teleop_hand_and_arm.py --arm=G1_29 --ee=dex3 --sim

# Physical robot (default: G1_29, hand tracking, immersive display)
python teleop_hand_and_arm.py --arm=G1_29 --ee=dex3

# With data recording
python teleop_hand_and_arm.py --ee=dex3 --sim --record

# Headless mode (no display, e.g. on robot's PC2)
python teleop_hand_and_arm.py --ee=dex3 --headless

# Motion mode (robot walks while teleoperating arms)
python teleop_hand_and_arm.py --ee=dex3 --motion

# Controller tracking instead of hand tracking
python teleop_hand_and_arm.py --input-mode=controller --ee=dex1
```

Runtime keyboard controls: **r** = start tracking, **s** = toggle recording, **q** = quit.

## Architecture

### Data Flow

```
XR Device (WebXR/WebSocket)
    │
    ▼
televuer (TeleVuerWrapper)  ──  captures wrist poses, hand joint positions, controller inputs
    │
    ▼
teleop_hand_and_arm.py      ──  main loop: orchestrates all subsystems at --frequency Hz
    │
    ├──▶ robot_arm_ik        ──  solves IK (Pinocchio + CasADi) for target wrist poses
    ├──▶ robot_arm           ──  sends joint commands to robot via DDS (unitree_sdk2py)
    ├──▶ robot_hand_*        ──  runs dexterous hand control in separate processes (multiprocessing)
    └──▶ episode_writer      ──  records states/actions/images to disk
```

### Key Subsystems

- **`teleop/teleop_hand_and_arm.py`** — Single entry point. State machine (START/STOP/READY/RECORD_RUNNING) driven by keyboard or IPC. All robot types and end-effectors are handled here via CLI args.

- **`teleop/robot_control/robot_arm_ik.py`** — IK solvers per robot variant (`G1_29_ArmIK`, `G1_23_ArmIK`, `H1_2_ArmIK`, `H1_ArmIK`). Uses Pinocchio for kinematics and CasADi for nonlinear optimization. Loads URDF from `assets/`, caches compiled models as `.pkl` files for faster startup.

- **`teleop/robot_control/robot_arm.py`** — Arm controllers per robot variant. Publishes low-level motor commands via DDS topics (`rt/lowcmd` for debug, `rt/arm_sdk` for motion mode). Subscribes to `rt/lowstate` for current joint state. Handles smooth startup ramp and go-home on exit.

- **`teleop/robot_control/robot_hand_*.py`** — End-effector controllers (Dex3-1, Dex1-1, Inspire DFX/FTP, BrainCo). Each runs as a **separate process** via `multiprocessing`, communicating with the main loop through shared `Array`/`Value` objects.

- **`teleop/robot_control/hand_retargeting.py`** — Wraps the `dex-retargeting` submodule to map XR hand joint positions to robot hand joint angles.

### Git Submodules

Three submodules in `teleop/`:
- **`televuer`** — WebXR visualization and XR data capture (Vuer-based)
- **`teleimager`** — Camera image service (ZMQ + WebRTC streaming)
- **`dex-retargeting`** (in `robot_control/`) — Dexterous hand retargeting algorithms

### Communication

- **Robot ↔ Host**: CycloneDDS via `unitree_sdk2py`. Domain ID 0 for physical robot, 1 for simulation.
- **XR Device ↔ Host**: WebSocket/WebRTC over HTTPS (requires SSL certs in `televuer/` or `~/.config/xr_teleoperate/`).
- **Image Server (PC2) ↔ Host**: ZMQ for image frames, WebRTC for direct streaming to XR device.
- **IPC mode** (`--ipc`): ZMQ REQ/REP on `tcp://0.0.0.0:5556` for programmatic control (agent integration).

### Logging

Uses `logging_mp` (multiprocessing-safe logging) throughout. Get a logger via:
```python
import logging_mp
logger_mp = logging_mp.getLogger(__name__)
```

## Robot Variants

| `--arm` flag | Robot | Arm DoF | Motor count | URDF path |
|---|---|---|---|---|
| `G1_29` | G1 (29 DoF) | 7 per arm | 35 | `assets/g1/` |
| `G1_23` | G1 (23 DoF) | 4 per arm | 35 | `assets/g1/` |
| `H1_2` | H1_2 | 7 per arm | 35 | `assets/h1_2/` |
| `H1` | H1 | 4 per arm | 20 | `assets/h1/` |

## Conventions

- The main script must be run from `teleop/` because URDF paths in IK classes are relative (e.g., `../assets/g1/...`).
- Hand controllers use `multiprocessing` with shared memory (`Array`, `Value`, `Lock`) — not threads.
- IK solver results are filtered through `WeightedMovingFilter` for smooth joint trajectories.
- Recorded data goes to `teleop/utils/data/` by default (gitignored).

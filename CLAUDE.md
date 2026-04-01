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
pip install -e teleop/teleimager       # client-only; use pip install -e "teleop/teleimager[server]" on PC2
pip install -e teleop/televuer

# Install additional deps
pip install -r requirements.txt

# External dependency (not in this repo)
pip install -e /path/to/unitree_sdk2_python
```

**Unlisted dependencies** that must be available: `casadi` (IK solver), `torch` (dex-retargeting), `psutil` (only with `--affinity`). The `logging-mp` package is installed transitively by teleimager/televuer.

## Testing and Linting

There are **no automated tests or linting tools** configured in this repo. No pytest, CI pipeline, Makefile, or formatter config exists. The only test-like files are manual integration examples in `teleop/televuer/example/`.

## Running

All commands run from `teleop/` directory:

```bash
# Simulation mode (G1_29 + Dex3 hand)
python teleop_hand_and_arm.py --arm=G1_29 --ee=dex3 --sim

# Physical robot (default: G1_29, hand tracking, immersive display)
python teleop_hand_and_arm.py --arm=G1_29 --ee=dex3

# Arm-only (no end-effector) — --ee is optional
python teleop_hand_and_arm.py --arm=G1_29 --sim

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

- **`teleop/teleop_hand_and_arm.py`** — Single entry point. State machine (START/STOP/READY/RECORD_RUNNING) driven by keyboard or IPC. All robot types and end-effectors are handled here via CLI args. Main loop: waits for `r` key in pre-start loop, then runs at `1/--frequency` Hz reading images, solving IK, commanding arms, and optionally recording.

- **`teleop/robot_control/robot_arm_ik.py`** — IK solvers per robot variant (`G1_29_ArmIK`, `G1_23_ArmIK`, `H1_2_ArmIK`, `H1_ArmIK`). Uses Pinocchio for kinematics and CasADi for nonlinear optimization. Loads URDF from `assets/`, caches compiled models as `.pkl` files in the current working directory for faster startup.

- **`teleop/robot_control/robot_arm.py`** — Arm controllers per robot variant. Publishes low-level motor commands via DDS topics (`rt/lowcmd` for debug, `rt/arm_sdk` for motion mode). Subscribes to `rt/lowstate` for current joint state. Handles smooth startup ramp and go-home on exit.

- **`teleop/robot_control/robot_hand_*.py`** — End-effector controllers (Dex3-1, Dex1-1, Inspire DFX/FTP, BrainCo). Each runs as a **separate process** via `multiprocessing`, communicating with the main loop through shared `Array`/`Value` objects.

- **`teleop/robot_control/hand_retargeting.py`** — Wraps the `dex-retargeting` submodule to map XR hand joint positions to robot hand joint angles.

### End-Effector / Input Mode Compatibility

| `--ee` | Input mode | Shared memory | Notes |
|---|---|---|---|
| `dex3` | `hand` only | `Array('d', 75)` in, 14 out | 25 joints × 3D hand skeleton |
| `dex1` | `hand` or `controller` | `Value('d')` in, 2 out | Uses pinch (hand) or trigger (controller) |
| `inspire_dfx` | `hand` only | `Array('d', 75)` in, 12 out | Requires DFX service on PC2 |
| `inspire_ftp` | `hand` only | `Array('d', 75)` in, 12 out | Requires FTP service on PC2 |
| `brainco` | `hand` only | `Array('d', 75)` in, 12 out | Requires BrainCo service on PC2 |

When `--ee` is omitted, no end-effector process is spawned (arm-only teleoperation).

### Git Submodules

Three submodules in `teleop/`:
- **`televuer`** — WebXR visualization and XR data capture (Vuer-based)
- **`teleimager`** — Camera image service (ZMQ + WebRTC streaming). Runs on PC2; provides CLI entry points `teleimager-server` and `teleimager-client`.
- **`dex-retargeting`** (in `robot_control/`) — Dexterous hand retargeting algorithms (silencht fork)

### Communication

- **Robot ↔ Host**: CycloneDDS via `unitree_sdk2py`. Domain ID 0 for physical robot, 1 for simulation.
- **XR Device ↔ Host**: WebSocket/WebRTC over HTTPS (requires SSL certs in `televuer/` or `~/.config/xr_teleoperate/`).
- **Image Server (PC2) ↔ Host**: ZMQ for image frames, WebRTC for direct streaming to XR device. Camera config via `cam_config_server.yaml` in teleimager.
- **IPC mode** (`--ipc`): ZMQ REP/REQ on Linux abstract sockets (`ipc://@xr_teleoperate_data.ipc` for commands, `ipc://@xr_teleoperate_hb.ipc` for heartbeat). Same-host only. Commands: `CMD_START`, `CMD_STOP`, `CMD_RECORD_TOGGLE`. Run `python teleop/utils/ipc.py` for a client example.

### Recording Format

Episodes are saved to `<task-dir>/<task-name>/episode_XXXX/` with auto-incrementing IDs. Each episode contains:
- `colors/` — JPEG images per camera per frame (`NNNNNN_color_N.jpg`)
- `data.json` — incrementally-written JSON with `info`, `text` (goal/desc/steps), and `data` array (per-frame `states`/`actions` with `left_arm`, `right_arm`, `left_ee`, `right_ee` qpos/qvel/torque)

The writer uses a background thread with a queue to avoid blocking the main control loop. Live visualization via `rerun-sdk` unless `--headless`.

### Logging

Uses `logging_mp` (multiprocessing-safe logging, `pip install logging-mp`) throughout. Get a logger via:
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

- The main script must be run from `teleop/` because URDF paths in IK classes are relative (e.g., `../assets/g1/...`) and `.pkl` cache files are written to the current directory.
- Hand controllers use `multiprocessing` with shared memory (`Array`, `Value`, `Lock`) — not threads.
- IK solver results are filtered through `WeightedMovingFilter` for smooth joint trajectories.
- Recorded data goes to `teleop/utils/data/` by default (gitignored).
- See `CHANGELOG.md` for version history and migration notes.

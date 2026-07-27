# Thesis Project — Claude Code Context

## Project
ROS 2 Humble + Gazebo Classic mobile manipulation thesis.
Robot: Agilex LIMO PRO (Mecanum target) + myCobot 280 M5 arm on Jetson Orin Nano.
Goal: vertical box stacking with Sim-to-Real drop-rate/error analysis.

## Knowledge Base
A structured knowledge base lives at:
`/home/omar/Desktop/Thesisorg/docs/super_memory/`

Use it whenever answering questions about this project. Categories:
- `facts/architecture/` — FSM, TF frames, ROS graph, package map
- `facts/code_map/` — per-node summaries (orchestrator, perception, grasp, etc.)
- `facts/design_decisions/` — why choices were made (MoveGroup vs MTC, weld grasp, etc.)
- `facts/troubleshooting/` — known failure modes and fixes
- `facts/hardware/` — specs for LIMO, myCobot, LiDAR, depth camera, Jetson
- `facts/parameters/` — joint limits, nav timeouts, grasp geometry, physics tuning
- `facts/experiments/` — calibration loops, Gazebo metrics, sim-vs-real results
- `facts/evaluation/` — KPIs, sim-to-real reliability focus
- `facts/glossary/` — TF frames, acronyms, topics/services
- `facts/workflows_commands/` — build, launch, cleanup, LaTeX compile commands

Always check the relevant category before answering questions about the system.

## Key Files
- Orchestrator: `src/limo_ros2/limo_car/scripts/nav_pick_orchestrator.py`
- Perception: `src/limo_ros2/limo_car/scripts/box_pose_estimator.py`
- Gazebo world: `src/limo_ros2/limo_car/gazebo/ackermann.xacro`
- URDF base: `src/limo_ros2/limo_car/urdf/limo_ackerman_base.xacro`
- Launch: `src/limo_ros2/limo_car/launch/`

## Hardware Notes
- Dual-GPU laptop: Gazebo must run on NVIDIA dGPU via PRIME offload (see `facts/hardware/hardware-compute-gpu-prime-offload.md`)
- Drive mode target: Mecanum holonomic (not Ackermann)

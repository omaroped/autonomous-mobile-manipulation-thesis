# Thesis Project — Claude Code Context

## Project
ROS 2 Humble + Gazebo Classic mobile manipulation thesis.
Robot: Agilex LIMO PRO (**differential drive**) + myCobot 280 M5 arm on Jetson Orin Nano.
Goal: single-box pick → transport → place → stack at a fixed position, with Sim-to-Real
error analysis.

**NEW SESSION? Read `docs/HANDOFF_2026-07-29.md` first.** It covers where we are, the working
rules, every problem hit so far and how it was solved, and the exact next step. Then
`docs/MASTER_STATUS_AND_PLAN_2026-07-28.md` — the single source of truth for status, the open
regression, and the plan to submission.

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
- **Drive mode: differential.** Corrected 2026-07-28 — the registered exposé (2026-07-01) states
  "AgileX LIMO differential-drive base", the code agrees (`drive_mode` default `diff`,
  `nav2_limo_diff.yaml`), and the Mecanum claim appears nowhere in the verified 2026-06-15
  supervisor notes. Ackermann was the original design and is abandoned.

## Before every simulation run
`./kill_sim.sh` — orphaned nodes from a prior run cause symptoms indistinguishable from real
bugs (two `base_pin`s fight and oscillate the base; a stale `grasp_attacher` floats a box in
mid-air; a stale `gzserver` makes edits appear to have no effect).

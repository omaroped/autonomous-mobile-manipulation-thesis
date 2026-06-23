# Mobile Manipulation with the Agilex LIMO Cobot

Thesis project exploring autonomous mobile manipulation in the context of on-demand manufacturing, built around the [Embodied Autonomous Intelligence Workshop (EAI-WS)](https://github.com/robocup-at-work/sml-embodied-autonomous-intelligence) framework for the RoboCup Smart Manufacturing League.

## Platform

**Agilex LIMO Cobot** — a compact mobile manipulator combining:
- LIMO PRO differential-drive base (4 kg payload, 1 m/s)
- myCobot 280 M5 6-DoF robotic arm (250 g payload, 280 mm reach)
- NVIDIA Jetson Orin Nano (8 GB, 40 TOPS)
- Orbbec Dabai depth camera
- ROS 2 Humble compatible simulation & control environment

## Repository Structure

```
Thesis/ (Workspace Root)
├── src/                      # ROS 2 packages (limo_cobot_bringup, limo_cobot_moveit_config)
├── third_party/              # Dependency source repositories
├── data/                     # Experimental datasets (raw/processed)
├── simulation/               # Gazebo worlds & simulation launch files
├── docs/                     # Unified Documentation & Academic Hub
│   ├── thesis/               # Active LaTeX document workspace (main.tex, chapters)
│   ├── writing_resources/    # University thesis guidelines & templates
│   ├── references/           # Papers (including the Embodied PDF), datasheets
│   ├── hardware/             # Physical robot hardware manuals & guides
│   ├── media/                # Images & screenshots for README / thesis
│   ├── meetings/             # Meeting logs
│   ├── progress/             # Weekly updates
│   └── proposal/             # Research proposal
├── archive/                  # Unrelated or retired personal assets
├── .gitignore                # Git ignore patterns
├── README.md                 # Project introduction (this file)
├── CHANGELOG.md              # Version history
└── reset_simulation.sh       # Shell utility to kill hanging ROS processes
```

## Quick Links

| Document | Path |
|----------|------|
| Working rules | [RULES.md](RULES.md) |
| Change log | [CHANGELOG.md](CHANGELOG.md) |
| Timeline | [docs/timeline.md](docs/timeline.md) |
| Reading list | [docs/references/reading_list.md](docs/references/reading_list.md) |
| Hardware requirements | [docs/hardware/requirements.md](docs/hardware/requirements.md) |

---

## 🏆 Key Milestones Reached (Simulation Baseline)

We have successfully built and verified the baseline Gazebo & MoveIt 2 simulation environment for the LIMO Cobot. Key implementations include:

1. **Direct Flush Mount:** Mounted `joint1` of the myCobot directly to the top plate of the LIMO chassis (Z-offset lowered to `0.025m`), matching the physical assembly and eliminating the floating gap from the deprecated `g_base`.
2. **ODE Physics Stability:** Resolved physics crashes (NaN solver failures / model collapse) by correcting the moment of inertia properties for the arm's **`joint2`** to satisfy the physical triangle inequality constraint ($ixx + iyy \ge izz$).
3. **Synchronized Mimic Gripper:** Restored the parallel-jaw adaptive gripper as a set of `revolute` joints with `<mimic>` tags in URDF. Active mimic parameters were added to the `<ros2_control>` tags to ensure the Gazebo control plugin actively locks and drives the passive linkages symmetrically.
4. **Coordinated Controller Managers:** Spawned `joint_state_broadcaster`, `arm_controller`, and `mycobot_gripper_controller` cleanly with delay parameters in the launch files to prevent startup race conditions.
5. **MoveIt 2 Trajectory Execution:** Fully integrated path planning with the controller managers, enabling direct motion planning and execution via RViz/MoveIt 2.

## Thesis Status

**Phase:** Coordinated Mobile Manipulation (weeks 4–8)  
**Started:** March 2026  
**Status:** Simulation Baseline Fully Complete & Verified  

---

> This repository is maintained as a living workspace throughout the thesis period.  
> See [RULES.md](RULES.md) for conventions on commits, branches, experiments, and documentation.

# Mobile Manipulation with the Agilex LIMO Cobot

Thesis project exploring autonomous mobile manipulation in the context of on-demand manufacturing, built around the [Embodied Autonomous Intelligence Workshop (EAI-WS)](https://github.com/robocup-at-work/sml-embodied-autonomous-intelligence) framework for the RoboCup Smart Manufacturing League.

## Platform

**Agilex LIMO Cobot** — a compact mobile manipulator combining:
- LIMO PRO differential-drive base (4 kg payload, 1 m/s)
- Mycobot 280 M5 6-DoF robotic arm (250 g payload, 280 mm reach)
- NVIDIA Jetson Orin Nano (8 GB, 40 TOPS)
- Orbbec Dabai depth camera
- ROS 2 Foxy / Humble compatible

## What This Repository Contains

```
docs/          Proposal, thesis (LaTeX), meeting notes, timeline
references/    Papers, datasheets, annotated reading list
src/           ROS 2 workspace, utility scripts, notebooks
simulation/    Gazebo / Isaac Sim worlds, URDF models, launch files
experiments/   Structured experiment logs with results
data/          Raw and processed experiment data (git-ignored)
media/         Photos, videos, diagrams
hardware/      Setup guides, requirements, troubleshooting
```

## Quick Links

| Document | Path |
|----------|------|
| Working rules | [RULES.md](RULES.md) |
| Change log | [CHANGELOG.md](CHANGELOG.md) |
| Timeline | [docs/timeline.md](docs/timeline.md) |
| Reading list | [references/reading_list.md](references/reading_list.md) |
| Hardware requirements | [hardware/requirements.md](hardware/requirements.md) |

## Thesis Status

**Phase:** Foundation (weeks 1–3)
**Started:** March 2026

---

> This repository is maintained as a living workspace throughout the thesis period.
> See [RULES.md](RULES.md) for conventions on commits, branches, experiments, and documentation.

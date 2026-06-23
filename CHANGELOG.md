# Changelog

A running narrative of significant decisions, milestones, and progress throughout the thesis.
Updated at least once per week.

---

## 2026-03-02 — Project Kickoff

- Created the thesis workspace with full directory structure.
- Established working rules (see [RULES.md](RULES.md)).
- Placed the EAI-WS paper (Masannek et al., 2026) into `docs/resources/literature/papers/`.
- Created initial reading list, timeline, hardware requirements, and document templates.
- Initialized Git repository.

**Current status:** Foundation phase. Next steps are finalizing the thesis proposal with the professor, setting up the development environment, and beginning the literature review.

---

## 2026-05-21 — Simulation Baseline & Reorganization Complete

- Mounted `mycobot` arm directly to the LIMO chassis top plate (Z-height flush adjustment to `0.025m`), matching physical hardware.
- Fixed `mycobot` Gazebo physics collapse by correcting `joint2` principal moments of inertia to satisfy the triangle inequality.
- Synchronized parallel-jaw gripper's kinematic loop using active `<ros2_control>` mimic joint configurations in Gazebo.
- Configured and spawned coordinated ROS 2 Humble controllers (arm trajectory and gripper controller) cleanly in the simulation launch flow.
- Reorganized the workspace root to isolate active software packages and keep all documents/data strictly local (`.gitignore` update).
- Relocated and compiled the LaTeX thesis draft successfully in its new professional structure (`docs/thesis/latex/`).

**Current status:** Simulation baseline verified. Next steps are navigation/mapping tests and initial autonomous planning with MoveIt 2.

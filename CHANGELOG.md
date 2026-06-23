# Changelog

A running narrative of significant decisions, milestones, and progress throughout the thesis.
Updated at least once per week.

---

## 2026-04-05 — Critical Fixes & Honest Assessment

- Conducted a full directory audit of the thesis repository. Identified 10 concrete errors.
- **Fixed world file:** Removed baked-in robot model from `finalworld`, renamed to `final_map.world`. The robot is now spawned exclusively via `spawn_entity` from the xacro pipeline — no more duplicates.
- **Fixed LaTeX:** Corrected `\bibliography{references}` → `\bibliography{bibliography}`. Added Declaration of Independent Work chapter to `main.tex`.
- **Fixed conclusion chapter:** Removed pre-written conclusions that claimed results from work not yet done. Replaced with honest TODO markers.
- **Honest status:** 5 weeks into a 14-week plan. Scaffolding is excellent, but thesis chapters are empty templates, literature review is barely started (1/10 papers read), and no experiments have been conducted. Time to execute.

**Current status:** Transitioning from foundation to active development. Next steps: lock thesis topic with professor, read 3 papers, get Nav2 working in simulation.

---

## 2026-03-19 — MyCobot 280 Arm Integration

- Integrated the myCobot 280 M5 6-DoF arm into the LIMO base xacro model.
- Created `mycobot_ros2_control.xacro` with position command interfaces for all 6 joints + gripper.
- Created `mycobot_controllers.yaml` with JointTrajectoryController and JointGroupPositionController.
- Encountered physics-based segfaults with ros2_control plugin — temporarily disabled to stabilize simulation.

**Current status:** Arm is visual-only in Gazebo. Controller integration is blocked by segfault root cause.

---

## 2026-03-15 — Educational Course Website

- Designed course plan for LIMO PRO + VLM educational website.
- Built React/Vite website with 6 lesson pages covering ROS 2, VLMs, and system architecture.
- Created Home page, Layout component, and custom styling.

**Current status:** Website is functional but not part of the thesis deliverables.

---

## 2026-03-08 — Thesis Writing Infrastructure

- Set up LaTeX project structure in `docs/thesis/` with chapter files, bibliography, and cleanup script.
- Created 12 thesis writing resource guides (academic phrases, HSRW guidelines, citation styles, anti-AI writing guide).
- Initialized `smart-manufacturing-nav` as a public GitHub repository.
- Configured terminal AI tools (Gemini CLI, Codex) with strict anti-AI writing rules.
- Compiled first `main.pdf` from LaTeX template.

**Current status:** Writing infrastructure complete. No actual thesis content written.

---

## 2026-03-07 — Simulation Environment

- Refined Gazebo world layout based on hand-drawn diagrams.
- Positioned red divider at ⅓/⅔ split, placed stations and counters.
- Generated initial `ackermann_gazebo.launch.py` for spawning the LIMO in the world.
- Transitioned from Voxtype to SpeechNote for voice-to-text dictation.

**Current status:** Simulation world built and robot spawns. Manual driving works.

---

## 2026-03-02 — Project Kickoff

- Created the thesis workspace with full directory structure.
- Established working rules (see [RULES.md](RULES.md)).
- Placed the EAI-WS paper (Masannek et al., 2026) into `references/papers/`.
- Created initial reading list, timeline, hardware requirements, and document templates.
- Initialized Git repository.

**Current status:** Foundation phase. Next steps are finalizing the thesis proposal with the professor, setting up the development environment, and beginning the literature review.

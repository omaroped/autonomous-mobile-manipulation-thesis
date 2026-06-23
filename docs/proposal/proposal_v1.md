# Thesis Preparation — Working Document

> **Platform:** Agilex LIMO Cobot (Mycobot 280 + NVIDIA Orin Nano)
> **Reference paper:** Masannek et al., "Embodied Autonomous Intelligence for On-Demand Manufacturing" (EAI-WS, 2026)
> **Author:** Omar [Last Name]
> **Supervisor:** Prof. [Name]
> **Date:** March 2026

---

## Current Situation

The professor has shared the EAI-WS workshop paper as a starting point. The thesis will involve work in a similar direction — mobile manipulation using the LIMO Cobot — but the exact thesis topic, title, and research questions have not been finalized.

Before official thesis registration, the expectation is to demonstrate independent preparation:
- Understand the platform (hardware, software, capabilities, limitations)
- Understand the domain (mobile manipulation, on-demand manufacturing, ROS 2 ecosystem)
- Set up the development environment and get basic autonomy running
- Identify concrete challenges and potential research contributions

This document evolves as the thesis direction becomes clearer.

---

## What the EAI-WS Paper Tells Us

The paper defines a smart manufacturing scenario where robots must:
- **Navigate** shared warehouse spaces autonomously
- **Manipulate** small objects (LEGO Duplo-sized colored blocks) with precision
- **Assemble/disassemble** products at workbench stations
- **Plan** and coordinate task execution dynamically

The LIMO Cobot fits naturally into the warehouse and workbench tracks. It has the navigation base for logistics and the 6-DoF arm for manipulation.

---

## Preparation Goals (Before Registration)

### 1. Platform Readiness
- [ ] Unbox and physically inspect the LIMO Cobot
- [ ] Set up the Orin Nano (Ubuntu, ROS 2, drivers)
- [ ] Verify teleop (drive the robot with keyboard)
- [ ] Verify arm control (move Mycobot 280 to known poses)
- [ ] Verify depth camera stream (RGB + depth)

### 2. Navigation Baseline
- [ ] Run SLAM and build a map of the lab/test area
- [ ] Run Nav2 and achieve autonomous point-to-point navigation
- [ ] Measure reliability: how often does it reach the goal?

### 3. Manipulation Baseline
- [ ] Set up MoveIt 2 for the Mycobot 280
- [ ] Achieve basic pick-and-place in simulation (Gazebo)
- [ ] Attempt real-world pick-and-place with a simple object

### 4. Perception Baseline
- [ ] Detect a colored block from the depth camera feed
- [ ] Estimate its 3D position relative to the robot
- [ ] Feed the position to the arm for grasping

### 5. Domain Understanding
- [ ] Read and annotate 10+ papers from the reading list
- [ ] Understand Nav2, MoveIt 2, and their integration patterns
- [ ] Identify what makes this hard (the open challenges)

---

## Potential Thesis Directions (To Discuss with Professor)

These are rough ideas, not commitments. They emerge from reading the EAI-WS paper and understanding the LIMO Cobot's capabilities:

1. **End-to-end mobile manipulation pipeline** — Navigate to a shelf, detect an object, pick it, deliver it to a workbench. Benchmark reliability and speed.
2. **Perception for colored block manipulation** — How to reliably detect, classify, and estimate 6-DoF pose of small colored blocks using the Orbbec depth camera + Orin Nano.
3. **Navigation + manipulation coordination** — Studying the handoff between Nav2 (base) and MoveIt2 (arm) on a resource-constrained edge device.
4. **Simulation-to-real transfer** — Develop the pipeline in Gazebo, transfer to the real LIMO, and measure the gap.
5. **EAI-WS warehouse track participation** — Build a system that could actually compete in the RoboCup SML warehouse track.

---

> This is a living document. Update after every meeting with the professor.
> The goal is to have a finalized thesis topic and registration within the first 2-3 weeks.

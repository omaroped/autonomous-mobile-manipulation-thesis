# Reading List

Annotated bibliography tracking every paper, article, and resource reviewed during the thesis.
Status: `[ ]` To Read · `[/]` Reading · `[x]` Done

---

## Core References

### [x] Embodied Autonomous Intelligence for On-Demand Manufacturing
**Authors:** Masannek, Steup, Norouzi, Uemura (2026)
**File:** `papers/Embodied Autonomous Intelligence Workshop - V1.0.pdf`
**Relevance:** Foundation paper for the thesis. Defines the EAI-WS framework, the three tracks (warehouse, workbench, planning), and the on-demand manufacturing scenario. Directly specifies the competition environment the LIMO Cobot will operate in.
**Key takeaways:**
- Three complementary tracks: warehouse (logistics/navigation), workbench (manipulation/assembly), planning (coordination/optimization)
- Uses LEGO Duplo-sized polymer blocks as standardized objects — 2×2×2 and 4×2×2 in five colors
- Tiered competition (beginner/advanced/expert) with increasing complexity
- Explicitly supports small ground-based robots like the LIMO Cobot
- Human–robot interaction and inter-team collaboration are first-class features, not workarounds
- Scoring rewards balanced autonomy across tracks, not single-track dominance

### [ ] From Production Logistics to Smart Manufacturing: The Vision for a New RoboCup Industrial League
**Authors:** Dissanayaka, Ferrein, Hofmann, Nakajima, Sanz-Lopez, Savage, Swoboda, Tschesche, Uemura, Viehmann, Yasuda (2025)
**Relevance:** Predecessor vision paper for the SML. Referenced as [1] in the EAI-WS paper. Provides context for why the leagues were merged.

---

## Navigation & SLAM

### [ ] Navigation2: A Complete Navigation Framework for ROS 2
**Authors:** Macenski et al.
**Relevance:** Nav2 is the standard ROS 2 navigation stack. Will be used for autonomous navigation in the warehouse track.

### [ ] SLAM Toolbox: SLAM for the Dynamic World
**Authors:** Macenski, Jambrecic
**Relevance:** Primary SLAM option for the LIMO. Need to compare with Cartographer for our use case.

---

## Manipulation & Grasping

### [ ] MoveIt 2: Motion Planning Framework for ROS 2
**Authors:** MoveIt maintainers
**Relevance:** Will be used for Mycobot 280 motion planning. Need to understand integration with Nav2 for mobile manipulation.

### [ ] Elephant Robotics Mycobot 280 Documentation
**Source:** elephantrobotics.com
**Relevance:** Official docs for the arm. Includes ROS 2 driver information, URDF models, and pymycobot API.

---

## Perception

### [ ] YOLOv8 / YOLOv11 for Object Detection
**Authors:** Ultralytics
**Relevance:** Candidate for block detection. Need to evaluate performance on the Orin Nano (TensorRT).

### [ ] BlenderProc: Synthetic Data Generation
**Authors:** Denninger et al.
**Relevance:** Mentioned in EAI-WS paper for generating training data for perception. Useful if real data collection is insufficient.

---

## Mobile Manipulation

### [ ] Embodied Intelligence for Robot Manipulation: Development and Challenges
**Source:** ResearchGate survey
**Relevance:** Systematic review of the field. Good for literature review chapter context.

---

## Platform Documentation

### [ ] Agilex LIMO Cobot User Manual
**Source:** agilex.ai
**Relevance:** Official platform documentation. Setup, calibration, ROS driver information.

### [ ] NVIDIA Jetson Orin Nano Developer Guide
**Source:** developer.nvidia.com
**Relevance:** Compute platform docs. JetPack SDK, TensorRT optimization, power modes.

---

> Add new entries here as papers are discovered. Keep the status updated.
> When a paper is finished, write a 2–3 sentence summary of key takeaways before marking it `[x]`.

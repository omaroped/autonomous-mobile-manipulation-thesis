# Bachelor Thesis Exposé

Autonomous Mobile Manipulation: Design and Evaluation of a Pick-and-Place and Object Stacking Pipeline on the LIMO Cobot

Omar Alobaid
Supervisor: Prof. Ronny Hartanto
Rhine-Waal University of Applied Sciences (HSRW)
July 2026

---

## Research Question

How can a modular autonomous mobile-manipulation pipeline, integrating onboard navigation, perception, and collision-aware motion planning, be designed and evaluated in a ROS 2 / Gazebo simulation to perform reliable, repeatable pick-and-place and object-stacking tasks with a compact manipulator and gripper, and to what extent can it then be transferred to and validated on the physical robot?

### Sub-Questions

1. Navigation. How reliably and repeatably can an autonomous mobile base position itself such that a target object lies within the manipulator's reachable workspace, and to what extent does base-positioning accuracy determine subsequent grasp success?

2. Perception. How accurately and robustly can a target object's three-dimensional pose be estimated from onboard RGB-D sensing and expressed in the manipulator's planning frame, and how does the resulting estimation error propagate to grasp success?

3. Manipulation and reachability. To what extent do the arm-mounting configuration, base stop distance, and workspace geometry constrain the reachable workspace of a short-reach manipulator, and how can collision-aware motion planning ensure that grasp and placement trajectories remain safe and executable within those constraints?

4. Stacking precision. With what precision and repeatability can objects be placed to achieve stable sequential vertical stacking, and which sources of error most strongly limit stacking reliability?

5. Evaluation in simulation. What level of task performance (success rate, placement accuracy, planning time, and cycle time) does the integrated pipeline achieve under repeated trials, and which failure modes dominate?

6. Simulation-to-reality transfer. To what extent does a pipeline developed in simulation transfer to the physical robot, how does real-world performance compare with simulated performance, and what engineering effort does the transfer entail?

---

## Scope and Limitations

### Scope

The thesis designs and evaluates a complete mobile-manipulation pipeline covering navigation, perception, grasping, placement, and sequential stacking, for a single compact mobile manipulator (AgileX LIMO Cobot). The system is developed and primarily evaluated in physics-based simulation; deployment on the physical robot is pursued as a transfer-and-validation objective, contingent on hardware availability and the time available within the thesis period. The operating environment is static, planar, and structured: a known layout, known support surfaces, and a single known target object at a time. Perception is delimited to a single, geometrically simple, colour-distinct object resting on a known support surface. Manipulation is restricted to top-down grasps of light, regularly shaped objects lying within the arm's reachable workspace. Evaluation is quantitative, based on success rate, placement accuracy, planning time, and cycle time, assessed under repeated trials (N ≥ 20).

### Limitations

Results are established primarily in simulation; despite physics-based modelling, a simulation-to-reality gap may affect real-world performance, and characterising this gap is itself part of the study. The limited reach of the manipulator (approximately 0.28 m) constrains the feasible workspace and the admissible placement of objects and support surfaces. Classical colour-and-depth perception is sensitive to lighting, sensor noise, and object appearance; robustness under uncontrolled conditions is not claimed. Achievable stacking precision is bounded by the combined error of base positioning, perception, and end-effector alignment. Physical-hardware validation depends on platform availability, calibration, and the time remaining within the thesis period.

### Out of Scope

Dynamic or deformable environments and moving obstacles or targets are not addressed. The work does not cover cluttered or multi-object scenes, general object recognition, learning-based perception or manipulation policies, or multi-robot coordination. Online map-building during execution is also outside scope; a known map and layout are assumed throughout.

---

## Task Planning

Total thesis period: 3 months, July 1 to October 1, 2026 (13 weeks). Phases overlap deliberately to make the most of the available time. Continuous light drafting runs throughout Phases 1–4 so that the writing phase is consolidation rather than writing from scratch.

### Phase 1 — Consolidation (Weeks 1–2, Jul 1–14)

Finalise the thesis framing; complete the integrated navigate → dock → perceive → grasp → transport → place → stack cycle into a reliable, repeatable run. Freeze the simulation testbed used for all subsequent experiments. Begin literature review.

### Phase 2 — Simulation Experiments (Weeks 2–6, Jul 8 – Aug 4)

Execute the systematic trial campaign (N ≥ 20 per configuration), recording task success rate, placement accuracy, planning time, and cycle time. Conduct the reachability study (effect of mounting, base stop distance, and workspace layout) and the stacking-precision study. Complete literature review.

### Phase 3 — Simulation-to-Reality Transfer (Weeks 5–10, Jul 29 – Sep 8)

Set up the physical LIMO Cobot (onboard computer, drivers, arm and camera calibration), port the pipeline to hardware, and perform the real-robot trials. Compare real-world placement precision against simulation and record the engineering effort required for the transfer. This phase is contingent on hardware availability and lab access.

### Phase 4 — Analysis (Weeks 9–11, Sep 1–22)

Process and interpret collected data, produce figures and tables, identify dominant failure modes, and prepare results for write-up.

### Phase 5 — Writing and Submission (Weeks 10–13, Sep 15 – Oct 1)

A three-week period reserved for writing and final revision. Complete all chapters, the discussion and conclusion, and the abstract; finalise and submit.

---

## Schedule

Registration date: July 1, 2026. Target submission: October 1, 2026 (13 weeks).

| Activity | Jul 1 | Jul 8 | Jul 15 | Jul 22 | Jul 29 | Aug 5 | Aug 12 | Aug 19 | Aug 26 | Sep 2 | Sep 9 | Sep 16 | Sep 23 |
|---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| Phase 1: Consolidation | ██ | ██ | | | | | | | | | | | |
| Literature review | ██ | ██ | ██ | ██ | ██ | ██ | | | | | | | |
| Phase 2: Simulation experiments | | ██ | ██ | ██ | ██ | ██ | | | | | | | |
| Reachability and stacking study | | | | ██ | ██ | ██ | | | | | | | |
| Phase 3: Hardware setup | | | | | ██ | ██ | ██ | | | | | | |
| Phase 3: Real-robot trials | | | | | | | ██ | ██ | ██ | ██ | | | |
| Phase 4: Analysis and figures | | | | | | | | | ██ | ██ | ██ | | |
| Phase 5: Writing and revision | | | | | | | | | | ██ | ██ | ██ | ██ |

### Milestones

| Milestone | Date | Deliverable |
|---|---|---|
| M1 — Simulation data collected | Aug 4 | N ≥ 20 simulation runs complete; metrics logged |
| M2 — Hardware trials complete | Sep 8 | Real-robot stacking trials done; comparison data logged |
| M3 — Analysis complete | Sep 22 | All figures, tables, and failure-mode analysis ready |
| M4 — Thesis submitted | Oct 1 | Final corrected thesis submitted |

---

## Overview

Mobile manipulation brings together two abilities that are normally studied apart: moving through an environment and handling the objects in it. A mobile manipulator is a wheeled base with a robotic arm, so one robot can drive to an object, pick it up, carry it, and put it down. The idea is well established and has been demonstrated on many research platforms over the years. Making it work reliably on a small, low-cost robot is still demanding, because the base and the arm have to be coordinated precisely for each task to succeed.

This thesis designs a complete pipeline for such a robot and studies how well it picks objects up, places them, and stacks them. The robot drives to a target, finds it with an onboard camera, plans a safe grasp around the surfaces near it, picks the object up, carries it, and places it where it should go. By repeating this cycle and setting each object on top of the last, the robot builds a stack. Stacking is the most demanding goal of the three, because the objects have to be placed precisely enough to rest stably on one another, so it sets the standard the rest of the pipeline has to meet. The work is carried out first in simulation and then, as far as time and hardware allow, on the real robot. The methods are kept simple and transparent rather than learning-based, so the system is straightforward to follow and to transfer to hardware. Performance is judged by how often the task succeeds, how accurately objects are placed, and how long planning and each full cycle take. Comparing the simulated behaviour with the behaviour on the real robot is itself one of the primary things the thesis sets out to measure.

### Platform Architecture

The mobile manipulator platform combines an AgileX LIMO differential-drive base with an Elephant Robotics myCobot 280 six-degree-of-freedom arm in the LIMO Cobot configuration. The base provides autonomous navigation capabilities, while the arm extends approximately 280 mm with a payload capacity of 250 g. An adaptive parallel-jaw gripper is mounted at the end-effector, and an RGB-D camera provides visual perception. The entire system operates under ROS 2 Humble with Gazebo Classic as the simulation environment, enabling physics-based testing before hardware deployment. This compact configuration represents a low-cost platform for mobile manipulation research, with the arm's limited reach creating stringent requirements for base positioning accuracy.

---

## References

- Macenski, S. and Jambrecic, I., "SLAM Toolbox: SLAM for the Dynamic World," *Journal of Open Source Software*, 2021.
- Macenski, S. et al., "The Marathon 2: A Navigation System," *IROS*, 2020.
- Masannek, M. et al., "Embodied Autonomous Intelligence for On-Demand Manufacturing," EAI-WS Proposal, 2026.
- Chaumette, F. and Hutchinson, S., "Visual Servo Control, Part I: Basic Approaches," *IEEE Robotics & Automation Magazine*, 2006.

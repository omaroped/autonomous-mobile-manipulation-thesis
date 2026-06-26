# Bachelor Thesis Exposé

By Omar [Last Name], [Matriculation No.]

---

## Title

**Autonomous Mobile Manipulation: Design and Evaluation of a Pick-and-Place and Object Stacking Pipeline**

> "Design and Evaluation" is intentionally generic: it covers both evaluation *in simulation* and evaluation *on the real robot*, so the simulation-to-reality transfer is carried implicitly as an evaluation objective rather than being fixed in the title. "Mobile" carries the locomotion/navigation aspect; "Pick-and-Place and Object Stacking" names the tasks without overclaiming complex assembly.

*(Alternative titles for reference)*

- *Autonomous Mobile Manipulation: Design, Evaluation, and Simulation-to-Reality Transfer of a Pick-and-Place Pipeline* — use this if the supervisor later wants the sim-to-real transfer made explicit in the title
- *Autonomous Mobile Manipulation: Design and Evaluation of a Pick-and-Place Pipeline* — shorter form if object stacking is dropped from the headline

---

## Research Question

*Primary language of study, as well as primary usage in literature relevant to the topic*

**Autonomous Mobile Manipulation: Design and Evaluation of a Pick-and-Place and Object Stacking Pipeline**

How can a modular autonomous mobile-manipulation pipeline — integrating onboard navigation, RGB-D perception, and collision-aware motion planning — be designed and evaluated in a ROS 2 / Gazebo simulation to perform reliable and repeatable pick-and-place and object-stacking tasks with a compact mobile manipulator, and to what extent can the simulation-developed pipeline be transferred to and validated on the physical robot?

---

## Sub-Questions

1. **Navigation.** How can the mobile base be autonomously and repeatably positioned so that the target object falls within the manipulator's reachable workspace, by combining global navigation (Nav2 / AMCL) with reactive vision-based docking, without prior knowledge of the object's exact pose?

2. **Perception.** How can a target object's three-dimensional pose be estimated from onboard RGB-D data using lightweight, classical methods (colour segmentation and depth deprojection) and reliably transformed into the manipulator's planning frame?

3. **Manipulation and reachability.** How do the arm-mounting configuration, base stop distance, and workspace layout jointly constrain the reachable workspace of a short-reach (280 mm) manipulator, and how can collision-aware motion planning (MoveIt 2) generate safe, executable top-down grasp and place trajectories?

4. **Stacking precision.** With what precision and repeatability can objects be placed to enable sequential vertical stacking, and which factors — base-positioning error, perception error, and end-effector alignment — most limit stacking reliability?

5. **Evaluation in simulation.** What level of performance — task success rate, placement accuracy, planning time, and cycle time — does the integrated pipeline achieve across repeated simulation trials (N ≥ 20), and which failure modes dominate?

6. **Simulation-to-reality transfer.** To what extent can the simulation-developed pipeline be transferred to the physical platform, how does real-world placement precision compare with simulation, and what engineering effort does the transfer require?

---

## Scope and Limitations

- This thesis focuses on autonomous pick-and-place in a static, controlled, indoor warehouse-style environment (flat ground, known layout, static obstacles)
- Primary implementation and validation in simulation — ROS 2 Humble, Gazebo Classic 11
- Platform: AgileX LIMO PRO (differential-drive) mobile base + Elephant Robotics myCobot 280 M5 (6-DoF arm) + adaptive parallel-jaw gripper + Orbbec DaBai RGB-D camera + EAI T-mini Pro 2D LiDAR, driven by NVIDIA Jetson Orin Nano (8 GB)
- Classical RGB-D perception (HSV colour segmentation + depth deprojection) for a single, colour-distinct target object on a known support surface
- Top-down grasp planned with MoveIt 2 (OMPL/RRTConnect); gripper driven directly outside the planner
- Nav2 with AMCL for global navigation; reactive visual-servo docking controller for precision base positioning at short range
- AprilTag (36h11) fiducial-guided placement for the place stage
- The EAI-WS (Embodied Autonomous Intelligence Workshop) framework for the RoboCup Smart Manufacturing League is used as the target application context; standardised LEGO Duplo polymer blocks serve as manipulation targets

**Note:** This thesis shall **not** focus on dynamic terrain, moving obstacles, cluttered multi-object scenes, learning-based detection (e.g. YOLO), multi-robot coordination, nor dynamic obstacles. Global SLAM map-building during operation is excluded. Deployment on physical hardware is an optional extension and **susceptible to change**.

---

## Weekly Task Planning

**Week 1 — Thesis Framing & Foundations**
Thesis framing and requirements definition; Literature Review (Part 1: mobile manipulation, visual servoing, sampling-based motion planning); Experiment Design (Part 1: metrics and test cases); Documentation (Chapters 1–2).

**Week 2 — System Modelling & Simulation Setup**
Combined robot model (LIMO + myCobot + gripper + sensors, URDF/Xacro, `ros2_control` integration); Gazebo Classic warehouse world construction and physics calibration (inertia tensors, controller timing); Literature Review (Part 2); Experiment Design (Part 2); Documentation (Chapters 1–3).

**Week 3 — Navigation & Perception**
Nav2 + AMCL global navigation; reactive visual-servo docking controller (camera-driven precision approach); RGB-D target detection and 3-D grasp pose estimation (HSV + depth deprojection + TF transform into arm frame); first end-to-end integration tests; Experiment Design (Part 3); Documentation (Chapters 3–4).

**Week 4 — Manipulation & Full Pipeline Integration**
MoveIt 2 configuration (SRDF, KDL IK, OMPL); collision-aware planning scene (support surface insertion); top-down grasp trajectory execution; adaptive gripper control (`close_until_contact`); complete state-machine orchestration (navigate → dock → perceive → grasp → carry → place → return); Experiments (Simulation, Part 1); Documentation (Chapter 4).

**Week 5 — Systematic Experiments & Analysis**
Systematic experiment campaigns (N ≥ 20 trials per configuration): grasp success rate, final-pose accuracy, planning time, cycle time; analysis of how base stop distance, arm mounting, and workspace layout affect reachability; Documentation (Chapter 5).

**Week 6 — Consolidation & (Optional) Hardware Transfer**
Documentation (Chapters 4–6 and Abstract); (optional) preliminary real-hardware deployment and validation on physical LIMO Cobot (Jetson Orin Nano, `/dev/ttyTHS1` arm driver); final refinements and adjustments.

---

## Overview

Mobile manipulation integrates autonomous navigation with robotic grasping, enabling a robot to traverse its environment and physically interact with objects — a core capability for warehouse automation, intralogistics, and smart manufacturing. Unlike a fixed-base manipulator, a mobile manipulator must first position its base so that the target object lies within the arm's reachable workspace, then perceive the object and plan a collision-free grasp. This coupling introduces a challenge absent from fixed-arm systems: when the arm's reach is limited — as with the Elephant Robotics myCobot 280 M5 at 280 mm — the feasible base-stop window is narrow and jointly governed by the arm mounting position, the base stop distance, and the workspace geometry. Positioning error propagates directly into grasp failure, making precise navigation a prerequisite for reliable manipulation (Chaumette and Hutchinson, 2006).

This thesis designs, implements, and evaluates an autonomous pick-and-place pipeline for the **AgileX LIMO Cobot** — a compact mobile manipulator pairing the LIMO PRO differential-drive base with the myCobot 280 M5 six-degree-of-freedom arm, an adaptive parallel-jaw gripper, and an Orbbec DaBai RGB-D camera, driven by an NVIDIA Jetson Orin Nano — operating within the **Embodied Autonomous Intelligence Workshop (EAI-WS)** framework for the RoboCup Smart Manufacturing League (Masannek et al., 2026). The entire system is implemented and validated in **ROS 2 Humble** and **Gazebo Classic 11**. Three integrated layers are built: (1) a global–local navigation stack that combines Nav2/AMCL waypoint planning for coarse positioning with a reactive camera-based visual-servo docking controller that drives the base to a graspable configuration using onboard RGB-D feedback; (2) a lightweight perception module that segments the target by colour in the HSV space, deprojects its depth point using camera intrinsics to obtain a 3-D object pose, and publishes it into the manipulator's planning frame via TF; and (3) collision-aware motion planning with MoveIt 2 (OMPL/RRTConnect) that inserts the support surface into the planning scene and computes a safe top-down grasp trajectory (Şucan et al., 2012; Coleman et al., 2014), after which the adaptive gripper closes and the object is lifted, transported, and placed at a fiducial-guided destination table.

Standardised LEGO Duplo-sized polymer blocks in five colours — as specified by the EAI-WS framework — serve as manipulation targets. The pipeline is evaluated through grasp success rate, final-pose accuracy, planning time, path length, and cycle time, with a particular focus on how base stop distance, arm mounting position, and workspace layout jointly constrain reachability. The design deliberately favours transparency and robustness — classical perception and direct motion planning rather than a heavy task-planning pipeline — to keep the system debuggable and a realistic candidate for transfer to the physical platform. An optional comparison of how the system behaves under different base kinematics (differential-drive versus Ackermann steering) may be included in simulation, subject to time. Deployment on the physical LIMO Cobot is planned as a final validation stage, subject to hardware availability and time.

### References (preliminary)

- Masannek, M., Steup, C., Norouzi, M., & Uemura, M. (2026). Embodied Autonomous Intelligence for On-Demand Manufacturing. *EAI Workshop Proceedings*.
- Chaumette, F., & Hutchinson, S. (2006). Visual servo control. I. Basic approaches. *IEEE Robotics & Automation Magazine*, 13(4), 82–90.
- Şucan, I. A., Moll, M., & Kavraki, L. E. (2012). The Open Motion Planning Library. *IEEE Robotics & Automation Magazine*, 19(4), 72–82.
- Coleman, D., Şucan, I. A., Chitta, S., & Correll, N. (2014). Reducing the Barrier to Entry of Complex Robotic Software: a MoveIt! Case Study. *Journal of Software Engineering for Robotics*, 5(1).
- Macenski, S., et al. Navigation2: A Complete Navigation Framework for ROS 2. *ROS-Industrial Consortium*.

---

## Gantt Chart

```
TASK                                     W1    W2    W3    W4    W5    W6
─────────────────────────────────────────────────────────────────────────
Thesis Framing & Requirements            ████
Literature Review                        ████  ████  ▒▒▒▒
Experiment Design                        ████  ████  ████
─────────────────────────────────────────────────────────────────────────
System Model (URDF / Xacro / ros2ctrl)         ████
Simulation World & Physics Calibration         ████
─────────────────────────────────────────────────────────────────────────
Navigation (Nav2 + AMCL)                             ████
Visual-Servo Docking Controller                      ████
─────────────────────────────────────────────────────────────────────────
RGB-D Perception & TF Pose Pipeline                  ████
─────────────────────────────────────────────────────────────────────────
MoveIt 2 Config & Grasp Planning                           ████
Gripper Control & Adaptive Close                           ████
State-Machine Orchestrator (FSM)                           ████
─────────────────────────────────────────────────────────────────────────
Simulation Experiments (Part 1)                            ▒▒▒▒
Simulation Experiments (Part 2, N≥20)                            ████
Data Analysis & Result Write-Up                                  ████
─────────────────────────────────────────────────────────────────────────
Hardware Transfer (optional)                                           ▒▒▒▒
─────────────────────────────────────────────────────────────────────────
Documentation / Writing (ongoing)        ████  ████  ████  ████  ████  ████
Abstract & Final Review                                                ████
─────────────────────────────────────────────────────────────────────────
Legend: ████ Primary focus   ▒▒▒▒ Secondary / partial   (blank) Not active
```

---

*This exposé was prepared for submission to [Supervisor Name], [University/Department], [Date].*

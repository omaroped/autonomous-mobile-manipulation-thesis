# Master's Thesis Exposé

By Omar [Last Name], [Matriculation No.]
*(Change "Master's" → "Bachelor's" if needed.)*

---

## Title Options (ranked — offer the next if one isn't preferred)

Each title frames the **same project** from a slightly different angle, so the supervisor can steer the direction.

**1. (Recommended — balanced, integration + research angle)**
*Autonomous Vision-Guided Mobile Manipulation for Pick-and-Place: Integrating Visual-Servo Navigation and Collision-Aware MoveIt 2 Motion Planning on a LIMO–myCobot Platform*

**2. (Research-forward — emphasises the navigation→grasp / reachability contribution)**
*From Navigation to Grasp: Base Positioning and Reachability Constraints in Autonomous Mobile Manipulation*

**3. (Coupling / hand-off angle)**
*Coupling Reactive Visual-Servo Navigation with Manipulator Motion Planning for Autonomous Mobile Pick-and-Place*

**4. (Perception-driven angle)**
*Vision-Guided Pick-and-Place for a Mobile Manipulator: RGB-D Perception and Collision-Aware Motion Planning on the AgileX LIMO and myCobot 280*

**5. (System / engineering angle — simplest, safest)**
*Design and Evaluation of an Autonomous Pick-and-Place Pipeline for a Mobile Manipulator in Simulation (ROS 2 / Gazebo)*

**6. (Application / warehouse angle)**
*Autonomous Pick-and-Place for Warehouse Mobile Manipulators: A ROS 2 Simulation Study on a LIMO–myCobot System*

**7. (Motion-planning emphasis)**
*Collision-Aware Manipulation for Mobile Robots: Integrating MoveIt 2 with Visual-Servo Navigation for Autonomous Grasping*

---

## Research Question

Can an autonomous mobile manipulator that integrates camera-based visual-servo navigation, lightweight RGB-D perception, and collision-aware motion planning (MoveIt 2) reliably perform vision-guided pick-and-place of a target object in a controlled environment — and which base-positioning and workspace constraints govern grasp success when a mobile base is coupled with a limited-reach manipulator?

## Sub-Questions

1. **Navigation.** How can a mobile base be driven to a *graspable* configuration relative to a target object using only onboard RGB-D camera feedback (reactive visual servoing), without a pre-built global map?
2. **Perception.** How can a target object's 3-D grasp pose be estimated from RGB-D data (colour segmentation + depth deprojection) and transformed reliably into the manipulator's planning frame?
3. **Reachability / workspace.** How do the manipulator mounting position, the base stop distance, and the workspace layout (support-surface and object placement) affect reachability and grasp success when a short-reach arm (myCobot 280) is coupled with a mobile base?
4. **Planning safety.** How does collision-aware planning — adding the support surface to the MoveIt 2 planning scene — improve the safety and reliability of the grasp compared with open-loop arm motion?
5. **Evaluation.** How do these factors affect the quantitative outcomes of the task: grasp success rate, final-pose accuracy, planning time, and path length?
6. **Transfer (optional, susceptible to change).** To what extent can the simulation-validated pipeline be transferred to the physical LIMO + myCobot platform?

## Scope and Limitations

**In scope**
- Autonomous pick-and-place in a controlled, **static** indoor (warehouse-style) environment: flat ground, known layout, a single known target object on a support surface.
- Implementation and validation in **simulation** — ROS 2 Humble, Gazebo Classic.
- Platform: AgileX **LIMO** (Ackermann steering) mobile base + Elephant Robotics **myCobot 280** (6-DOF) manipulator + adaptive gripper.
- **Classical** RGB-D perception (HSV colour segmentation + depth deprojection) for a single, colour-distinct object.
- **Top-down grasp** planned with MoveIt 2 (OMPL); gripper position-controlled outside the planner.
- **Reactive** camera visual-servoing for navigation toward the visible target.

**Out of scope / limitations**

Note: this thesis shall **not** focus on dynamic or moving obstacles, moving targets, cluttered or multi-object scenes, learning-based object detection, multi-robot scenarios, nor dynamic terrain. Global SLAM / Nav2 path planning is excluded (the target lies within the local field of view). Deployment on **physical hardware** is an optional extension and **susceptible to change**.

## Weekly Task Planning

**Week 1 — Framing & foundations**
Thesis framing and requirements; Literature Review (Part 1: mobile manipulation, visual servoing, sampling-based motion planning); Experiment Design (Part 1: metrics & test cases); Documentation (Chapters 1–2).

**Week 2 — System design: model & environment**
Combined robot model (LIMO + myCobot + gripper, URDF/Xacro); simulation world and sensor configuration (RGB-D, LiDAR, IMU); Literature Review (Part 2); Experiment Design (Part 2); Documentation (Chapters 1–3).

**Week 3 — Perception & navigation**
RGB-D target detection and 3-D pose estimation; reactive visual-servo base controller (navigate-to-target); Experiment Design (Part 3); first integration tests; Documentation (Chapters 3–4).

**Week 4 — Manipulation & integration**
MoveIt 2 configuration; collision-aware top-down grasp planning; gripper control; full navigate → perceive → grasp → place state machine; Experiment (Simulation, Part 1); Documentation (Chapter 4).

**Week 5 — Experiments & analysis**
Systematic experiments (Simulation, Part 2): grasp success rate, final-pose accuracy, planning time, path length, and the effect of base stop distance / mounting / workspace layout on reachability; Analysis; Documentation (Chapter 5).

**Week 6 — Consolidation**
Documentation (Chapters 4–6 and Abstract); refinement and adjustments; (optional) preliminary real-hardware exploration on the physical LIMO + myCobot.

*(Scale the six phases proportionally if the thesis runs longer than six weeks.)*

## Overview

Mobile manipulation unites autonomous navigation with robotic manipulation, enabling a robot to move through its environment and physically interact with objects. It is a core capability for warehouse automation, intralogistics, and service robotics. Unlike a fixed-base manipulator, a mobile manipulator must first position its base so that the target lies within the arm's reachable workspace, then perceive the object and plan a collision-free grasp. This coupling introduces challenges absent from fixed manipulation: the precision of base positioning directly determines whether the object is graspable, and a manipulator with a limited reach imposes tight constraints on where the base must stop relative to the workspace.

This thesis designs, implements, and evaluates an autonomous pick-and-place pipeline on a simulated AgileX LIMO (Ackermann) mobile base equipped with a six-degree-of-freedom Elephant Robotics myCobot 280 arm and an adaptive gripper, using ROS 2 Humble and Gazebo Classic. The system integrates three layers. First, a reactive camera-based visual-servo controller drives the base toward the target using onboard RGB-D feedback, without relying on a global map (Chaumette and Hutchinson, 2006). Second, a lightweight perception module segments the target by colour and deprojects its depth to estimate a 3-D grasp pose, which is transformed into the manipulator's planning frame. Third, collision-aware motion planning with MoveIt 2 and the OMPL sampling-based planners (Şucan et al., 2012; Coleman et al., 2014) adds the support surface to the planning scene and computes a safe top-down grasp trajectory, after which the gripper closes and the object is lifted and placed.

A colour-distinct box on a pickup table serves as the target, and the task is evaluated through grasp success rate, final-pose accuracy, planning time, path length, and — as a particular focus — the relationship between base stop distance, manipulator mounting, workspace layout, and reachability. The work deliberately favours a simple, transparent, and robust design — classical perception and direct motion planning rather than heavyweight perception-and-task pipelines — to keep the system debuggable and a realistic candidate for transfer to the physical platform. A comparison of how a differential-drive versus an Ackermann base constrains the navigate-to-grasp hand-off may additionally be included in simulation, subject to time.

### References (preliminary)
- Chaumette, F., & Hutchinson, S. (2006). Visual servo control. I. Basic approaches. *IEEE Robotics & Automation Magazine*, 13(4), 82–90.
- Şucan, I. A., Moll, M., & Kavraki, L. E. (2012). The Open Motion Planning Library. *IEEE Robotics & Automation Magazine*, 19(4), 72–82.
- Coleman, D., Şucan, I. A., Chitta, S., & Correll, N. (2014). Reducing the Barrier to Entry of Complex Robotic Software: a MoveIt! Case Study. *Journal of Software Engineering for Robotics*, 5(1).

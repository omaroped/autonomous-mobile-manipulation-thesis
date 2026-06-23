# Supervisor Meeting — Direction & Notes

> **VERIFIED.** Updated and confirmed against the meeting audio recording. All key metrics, title wording, and scope boundaries have been verified.

---

## 1. Agreed thesis direction
- **Topic:** a mobile-manipulation pipeline on the LIMO COBOT (AgileX LIMO PRO + myCobot 280 M5)
  combining autonomous navigation with the 6-DOF arm for **pick-and-place and stacking**.
- **Core theme:** **simulation-to-reality transfer** — develop the pipeline in Gazebo, then move it
  to the physical robot. The majority of testing is done in simulation, with the physical hardware used for final validation of sim-to-real findings.
- **Refined scope:** a **single-block pick → place → stack cycle with a FIXED assembly position.**
  This reduces complexity while keeping academic merit, and can later be extended to variable positions.
- **Agreed Title:**
  *"Simulation-to-Reality Transfer of a Mobile Manipulation Pipeline: Development, Evaluation, and
  Analysis of a Pick-and-Place Task Using the LIMO COBOT."*

## 2. How the supervisor framed the *contribution* (important)
- The **exploratory process itself** — documenting design decisions, failures, and solutions — is a
  **core academic contribution**, to be written up as the methodology chapter.
- Contributions to document explicitly:
  1. pipeline architecture,
  2. collision-model optimisation methodology,
  3. navigation tuning,
  4. grasp parameter configuration,
  5. comparative evaluation of simulation fidelity vs. real-world performance.

> **Implication for us:** the long debugging we've done (collision matrix, contact tuning, perception
> bias, cmd_vel hand-off, GPU limits) is not wasted time — it is exactly the material the supervisor
> wants in the thesis.

## 3. Technical challenges he raised — mapped to what we already did
| Supervisor's point | What we already found / built | Where |
|---|---|---|
| **Collision mesh complexity** — manufacturer bounding boxes caused MoveIt planning failures; fixed by a decimated mesh from the real 3D model | We replaced the base collision box with a decimated chassis mesh and rebuilt the SRDF collision matrix; phantom collisions were also traced to a gripper mesh-scale bug | `limo_base_collision.stl`, `limo_cobot.srdf`, `COLLISION_DECISION.md` |
| **Grasp instability** — objects slide/displace; attributed to inertial/damping params, a known Gazebo/MoveIt issue | We used soft ODE contact (kp/kd/max_vel/min_depth) + a weld helper to stabilise the grasp | `grasp_attacher.py`, `PARAMETER_AUDIT.md §5` |
| **Navigation drift / localisation** — base drifts (worse with the arm's weight); symmetric lab gives poor SLAM features → add unique landmarks | We saw AMCL/odom drift and used a base-pin fixture; Ackermann + low wheel friction add to it | `base_pin.py`, nav notes |
| **Pipeline integration complexity** — cascading errors at integration points, esp. camera re-localisation between steps | Exactly our full-pipeline experience: cmd_vel contention, close-range depth bias, re-perceiving at the table | `nav_pick_orchestrator.py`, `NAV_PICK_PLAN.md` |

## 4. Scope decisions
- **In scope:** single block, **pick → transport → place → stack at a FIXED (hard-coded) assembly position.**
- **Why fixed placement helps:** it removes the need for dynamic placement perception (the hardest,
  least reliable part) — you command the arm to known stack coordinates. This de-risks stacking.
- **Out of scope / future work:**
  - reinforcement-learning grasping (scoped out for time),
  - variable / perception-driven assembly positions,
  - error-recovery behaviour (detect misplacement → re-grasp) — an extension once the baseline is stable.

## 5. Hardware / environment
- Simulation runs locally on an **OMEN laptop, Ryzen 5, NVIDIA GPU (6 GB VRAM)**, GPU-accelerated Gazebo.
- **Note for the methodology:** 6 GB VRAM is a real constraint — it is consistent with the GPU/render
  instabilities we hit (gzserver depth-camera segfaults under load). Worth documenting as a
  simulation-performance limitation, not a code defect.

## 6. Next steps (from the meeting)
1. Finalise and **register the thesis title** with the university.
2. Confirm the **~3-month work plan** with the supervisor.
3. Complete the **end-to-end pipeline in simulation**.
4. **Transfer to physical hardware** for final experimentation.
5. **Document all decisions, failures, and solutions** (methodology chapter).

## 7. My analysis — what to do with this
1. **Strong alignment.** The supervisor's hard problems = the problems we already characterised. Reframe
   our debugging logs as methodology content, not as "stuff that went wrong."
2. **The scope is now genuinely achievable.** Fixed-position, hard-coded stacking of one block sidesteps
   the placement-perception problem that was blocking us. The remaining work is reliability + the stack motion.
3. **Update the exposé** (`expose_v2.md`): make **stacking core** (not future work), make **placement
   fixed/hard-coded**, adopt the new title, keep sim-to-real central, and add an explicit
   "Documented Development Process" contribution (he values it).
4. **Document the 6 GB VRAM limit** and the staged-startup/shared-memory lessons as real
   simulation-engineering findings.

## 8. Verified from Recording

All verification points have been resolved against the meeting transcript:
- **Mesh Figures:** The base robot mesh was 7,500 faces (approx. 50 MB file size) and was simplified to a 300 KB file size, resolving load time issues in Gazebo.
- **Thesis Title:** Confirmed as *"Simulation-to-Reality Transfer of a Mobile Manipulation Pipeline: Development, Evaluation, and Analysis of a Pick-and-Place Task Using the LIMO COBOT."*
- **Stacking Scope:** Stacking is simplified to a single-block pick, transport, and stack cycle at a fixed, hard-coded target position. Stacking multiple blocks is out of scope/future work.
- **Sim-vs-Real Split:** Most experimental trials and systematic evaluations will run in simulation. Real hardware runs will serve as final validation and sim-to-real gap analysis rather than full comparative study.
- **Simulation Environment Landmarks:** The supervisor recommends adding asymmetrical features or distinct landmarks to the simulation environment to assist AMCL/localization and prevent drift, rather than keeping a bare/symmetric room.
- **Hardware Integration & Safety:** Emphasize systematic pre-grasp safety stages to prevent collision forces.
- **Timeline:** The student will register the thesis for a 3-month timeline.
- **Second Supervisor:** Still to be finalized/registered.

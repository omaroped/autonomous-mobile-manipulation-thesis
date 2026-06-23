# Professor Demo Scripts & Specifications

These are step-by-step command files and specifications to demonstrate and validate the autonomous pick-and-place pipeline.

**Before starting, make sure nothing is running from a previous session:**
```bash
pkill -9 -f "gzserver|gzclient|move_group|rviz2|robot_state_publisher|spawn_entity|box_pose|arm_grasp|base_pin|grasp_attacher" ; sleep 3
```

---

## 1. Demo Order (Recommended)

Run these demos in order — each builds on the previous one's understanding:

| # | File | What it shows | Time |
|---|------|---------------|------|
| 1 | [01_gazebo_world.sh](01_gazebo_world.sh) | Gazebo simulation + robot model + warehouse world | 2 min |
| 2 | [02_teleop.sh](02_teleop.sh) | Keyboard control of base + arm + gripper | 3 min |
| 3 | [03_navigation.sh](03_navigation.sh) | Nav2 autonomous drive to the workstation | 3 min |
| 4 | [04_perception.sh](04_perception.sh) | Camera sees the box, publishes 3D pose | 2 min |
| 5 | [05_arm_grasp.sh](05_arm_grasp.sh) | MoveIt picks up the box (fixed pose, no nav) | 3 min |
| 6 | [06_full_pipeline.sh](06_full_pipeline.sh) | Full orchestrator: navigate → dock → perceive → grasp → retract | 5 min |

**Total demo time: ~18 minutes** (plus discussion)

### Tips for the meeting
- Have **all terminals pre-opened** before the professor arrives.
- Run Demo 1 first so Gazebo is already loaded (it takes ~20s to start).
- If anything crashes, just run the cleanup line and restart that demo.
- The most impressive demo is **Demo 6** (full pipeline) — save it for last.

---

## 2. Scenario Coordinates & Spatial Setup

All coordinates are in meters, specified in either the world frame or the robot's local planning frame (`base_link`).

### World Coordinates
- **Robot Spawn Pose**: $(-2.0, 7.0, 0.15)$ facing $-Y$ direction (forward is $-Y$).
- **Support Table (`grasp_test_table`)**: Size $0.20 \times 0.06 \times 0.20$ m. Spawned at $(-2.0, 6.79, 0.10)$. The table top is at $z = 0.20$ m.
- **Target Object (`grasp_test_box`)**: A $2$ cm blue cube. Spawned at $(-2.0, 6.79, 0.21)$, resting on the top of the table.

### Robot Local Coordinates (`base_link`)
- **Box Centroid**: $(0.21, 0.0, 0.06)$
- **Grasp Target Pose**: $(0.21, 0.0, 0.08)$ (offsets TCP height slightly to avoid contact collisions).

---

## 3. Motion Sequence

The execution node runs the following sequence of commands to perform a top-down grasp:

1. **Initialize**: Arm moves to `ready` named joint state: `[0.0, -0.5, -0.6, 1.1, 0.0, 0.0]`.
2. **Open Gripper**: Publish open state `[0.15, 0.15, -0.15, -0.15, -0.15, 0.15]`.
3. **Pre-Grasp Hover**: Plan and move TCP to $(0.21, 0.0, 0.17)$ with top-down orientation `(-0.7071, 0.0, 0.0, 0.7071)`.
4. **Descend**: Move TCP down to grasp height $(0.21, 0.0, 0.11)$, straddling the upper part of the box.
5. **Attach & Close**:
   - Turn on weld simulation helper via `/grasp_attach` topic.
   - Gently close fingers to partial grip `[-0.20, -0.20, 0.20, 0.20, 0.20, -0.20]`.
6. **Lift**: Plan and execute lift back to $(0.21, 0.0, 0.17)$ carrying the box.
7. **Hold**: Pause for 3 seconds to verify grasp stability.
8. **Place**: Lower TCP back to $(0.21, 0.0, 0.11)$ to place the box.
9. **Release**:
   - Turn off weld helper via `/grasp_attach` topic.
   - Fully open gripper to `[0.15, 0.15, -0.15, -0.15, -0.15, 0.15]`.
10. **Retreat**: Retreat to pre-grasp hover $(0.21, 0.0, 0.17)$, then return to `ready`.

---

## 4. Success Criteria

A test run is considered successful if all the following conditions are met:
- **Object Lifted**: The box is lifted $\ge 5$ cm above the table top.
- **Hold Duration**: The box remains attached for $\ge 3$ seconds.
- **No Sim Explosion**: Neither the robot nor the box undergoes high-frequency joint oscillations or is launched into the sky by rigid-contact forces.
- **Target Success Rate**: 3/3 consecutive successful runs.

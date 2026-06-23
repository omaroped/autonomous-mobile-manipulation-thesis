# MoveIt integration — Stage 0/1/2 notes (written while you were out)

## What's done (code-complete, validated as far as possible without the sim)

| Item | State |
|---|---|
| `gazebo_ros2_control` EOL fix (patched plugin in overlay) | built; **awaiting your relaunch to confirm controllers load** |
| `limo_cobot_moveit_config` (MoveIt config) | built + config assembles |
| `box_pose_estimator.py` (Stage 2: camera → `/box_pose`) | written, deps present, syntax OK |
| `pick_orchestrator.py` (Stage 1: collision-aware pick) | written, **API-checked vs real moveit_msgs**, syntax OK |
| Launches: `moveit.launch.py`, `stage1_pick.launch.py` | written |

## Important API change
`moveit_py` is **not packaged for Humble** (only Iron+). So the orchestrator does
**not** use moveit_py — it drives the running `move_group` via the standard
`moveit_msgs` action/service interface over `rclpy`. Pure Python, no new packages.

## Test order (each terminal, all `source install/setup.bash` first)
1. `ros2 launch limo_car ackermann_gazebo.launch.py`   — sim + controllers (must work first)
2. `ros2 launch limo_cobot_moveit_config moveit.launch.py`   — move_group + RViz
3. `ros2 launch limo_cobot_moveit_config stage1_pick.launch.py`  — perception + pick

Test perception alone first: `ros2 run limo_car box_pose_estimator` and
`ros2 topic echo /box_pose` — confirm a sane box position in base_link.
Run the arm motion without the camera: `... stage1_pick.launch.py perception:=false`.

## TUNABLE constants in pick_orchestrator.py (calibrate once, visually)
- `TOPDOWN_QUAT` — gripper-down orientation; **a guess**. Verify in RViz, adjust.
- `GRASP_TCP_ABOVE` (0.12) / `PRE_GRASP_TCP_ABOVE` (0.22) — TCP height above box.
  `gripper_tcp` is currently coincident with `gripper_base`, so 0.12 ≈ finger length.
  When you set the real `gripper_tcp` offset (in ackermann_with_sensor.xacro), drop this.
- `BOX_FALLBACK` — used only if `/box_pose` isn't published.
- Named state `ready` is mirrored from the SRDF — keep them in sync.

## Known caveats / next
- KDL IK may struggle near singularities; if pose goals fail to plan, TRAC-IK is the fallback.
- Stage 3 TODO (marked in code): attach the box to `gripper_tcp` for collision-free carry,
  then wire the existing visual-servo nav in front, add place + return-home.

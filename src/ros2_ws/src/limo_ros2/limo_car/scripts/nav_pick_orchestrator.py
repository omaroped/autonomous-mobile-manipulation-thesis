#!/usr/bin/python3
"""
nav_pick_orchestrator.py — full autonomous navigate-and-pick sequence.

Sequence (one-shot, runs to completion):
  1. Navigate  — Nav2 NavigateToPose → approach goal in front of the table
  2. Pin base  — Gazebo /set_entity_state holds the base steady for grasping
  3. Perceive  — wait for a stable /box_pose (settle-then-capture)
  4. Grasp     — calibrated MoveIt top-down pick (arm_grasp_test logic):
                   ready → pre_grasp → grasp → weld → gentle close → lift
  5. Retract   — go_named('ready')
  6. Back up   — publish cmd_vel reverse for BACKUP_SEC seconds

Prerequisites (run first):
    ros2 launch limo_car nav_pick.launch.py
    ros2 launch limo_cobot_moveit_config moveit.launch.py

All the MoveIt / gripper logic is ported verbatim from arm_grasp_test.py (proven).
The only addition is the Nav2 action client at the front and the cmd_vel reverse
at the end.

Design notes:
  - base_pin logic is INLINE here (no separate node) — the orchestrator owns the
    50 Hz hold timer and cancels it before publishing cmd_vel reverse.
  - AMCL initial pose is set programmatically (spawn is always −2, 7).
  - If Nav2 is not running the node fails fast with a clear error.
"""

import csv
import math
import os
import statistics
import time

import numpy as np

import reach_lookup

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import (PoseStamped, Pose, Point, PointStamped,
                                Twist)
from std_msgs.msg import Float64MultiArray, Bool, Float64
from action_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry

from nav2_msgs.action import NavigateToPose

from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPositionIK
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, JointConstraint,
    PositionConstraint, OrientationConstraint, BoundingVolume,
    CollisionObject, PlanningScene,
)
from shape_msgs.msg import SolidPrimitive

from gazebo_msgs.srv import GetEntityState, SetEntityState
from gazebo_msgs.msg import EntityState

from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener


class GraspResult:
    GRASPED    = 'grasped'      # stall at BOX_CONTACT_ANGLE ± APERTURE_TOL → weld fires
    AIR        = 'air'          # swept to full close with no stall → nothing in fingers
    OBSTRUCTED = 'obstructed'   # stall before BOX_CONTACT_ANGLE → finger on top/rim


# ── Navigation goal ───────────────────────────────────────────────────────────
# Robot stops so the arm base (~0.22 m ahead of body centre) is ~0.20 m from box.
# Table at (−2, 4); approach from +Y → stop at y ≈ 4.22.
# Fine-tune NAV_GOAL_Y if the arm can't reach on first run.
# FALLBACK ONLY. The live values come from config/scene.yaml via nav_pick.launch.py
# (self._nav_goal_x etc). These literals are used only if the node is run bare,
# without the launch file. Do not tune them -- tune scene.yaml.
NAV_GOAL_X   = -2.0
NAV_GOAL_Y   =  4.75    # Nav2 stops here; visual_docking() handles the final 0.5 m.
                         # Must be > pickup_table inflated north edge (4.10+0.45=4.55) + margin.
NAV_GOAL_YAW = -1.5708  # −π/2 → robot faces −Y (toward the table)

NAV_TIMEOUT_SEC = 120.0  # generous — 3 m at 0.25 m/s = 12 s, but allow replanning

# ── Backup ────────────────────────────────────────────────────────────────────
BACKUP_VEL_X = -0.15   # m/s reverse
BACKUP_SEC   =  2.5    # back up ~0.38 m

# ── Gripper ───────────────────────────────────────────────────────────────────
# 6-element commands, mirror signs [1,1,-1,-1,-1,1] baked in — must match the
# joints: list in config/mycobot_controllers.yaml.
# The mimic-joint experiment (single-element commands) was REVERTED on 2026-07-29:
# it made gazebo_ros2_control register the followers as "<joint>_mimic", which broke
# their TF and left MoveIt without half the gripper's collision model. See
# VERIFICATION_REGISTER.md V2.
GRIPPER_OPEN  = [ 0.15,  0.15, -0.15, -0.15, -0.15,  0.15]
GRIPPER_GRASP = [-0.20, -0.20,  0.20,  0.20,  0.20, -0.20]   # gentle partial close
GRIPPER_TOPIC = '/mycobot_gripper_controller/commands'

# ── How the box is held: REAL JOINT vs LEGACY WELD ───────────────────────────
# Selected by the `use_physics_grasp` parameter (default True), which nav_pick.launch.py
# also uses to decide whether to start grasp_attacher/smart_grasp — one flag, so the
# node and the launch graph cannot disagree. Read into self._physics_grasp in __init__.
#
# PHYSICS GRASP (default, use_physics_grasp:=true)
#   libgazebo_grasp_plugin (declared in gazebo/mycobot_ros2_control.xacro) creates a real
#   ODE fixed joint between gripper_base and the box once enough finger contacts
#   accumulate, and destroys it when gripper_controller opens past release_position (0.0).
#   Nothing in this node commands the grasp: contact makes it, opening the fingers breaks
#   it. `attach()` is therefore a no-op and `_weld_active` becomes an OBSERVATION
#   ("ground truth says the box came off the table") rather than a command echo.
#
# LEGACY WELD (use_physics_grasp:=false)
#   grasp_attacher.py teleports the box onto the gripper via /set_entity_state, triggered
#   by this node publishing /grasp_attach. Kept as a working fallback — it produced the
#   two successful end-to-end runs on 2026-07-02. Two known defects it cannot fix:
#     1. it copies POSITION only (the box's orientation is frozen at pickup, so the box
#        never rotates with the wrist), and
#     2. it teleports at ~50 Hz against 1000 Hz physics, so the box lags and snaps.
#
# The two must NEVER run together — the teleport overwrites the pose the joint solver
# just computed, every step.
USE_WELD = True   # legacy default; overridden per-instance by use_physics_grasp

# ── Aperture-based contact detection ─────────────────────────────────────────
# BOX_CONTACT_ANGLE: run calibrate_contact_angle.py with the box at the grasp pose to
# measure the median stall angle, then paste the result here.  The old
# WELD_FALLBACK_ANGLE (-0.10) was an uncalibrated proxy for this same constant.
BOX_CONTACT_ANGLE = -0.11    # rad — placeholder; run calibrate_contact_angle.py
APERTURE_TOL      = 0.025    # ±rad band: stall within this of contact → GRASPED
# LAG_THRESH/STALL_STEPS were loosened to 0.08/10 on 2026-07-26 so that one finger
# brushing the box would not halt the close. That was tuned against a BROKEN close
# loop: spin_once() was being used as a sleep, so the whole 70-step close ran in
# ~65 ms and the joint could never track the command at all (see _spin_for).
#
# With the loop actually running at 0.08 s/step, the joint DOES follow, and 0.08/10
# is far too slack. Once a finger touches, the joint holds still while the command
# keeps advancing 0.005 rad/step, so reaching a 0.08 lag takes 16 steps and 10 more
# to confirm — the controller drives 0.13 rad PAST contact before stopping. Measured
# box contact is at actual ≈ -0.010 rad, so that is a hard squeeze against a light
# box, which then slides out from between the fingers.
#
# 0.03/4 stops ~0.05 rad past contact: firm enough to hold, gentle enough not to
# extrude the box. Retuned 2026-08-03 together with the _spin_for fix — these two
# changes belong together, do not revert one without the other.
LAG_THRESH        = 0.03     # rad: actual lags commanded by this → stall onset
STALL_STEPS       = 4        # consecutive stall detections to confirm (~0.32 s)
CLOSE_STEP_RAD    = 0.005    # rad per step (same as legacy)
CLOSE_STEP_SEC    = 0.08     # s per step (same as legacy)
MAX_GRASP_RETRIES = 2        # retry descents before aborting the pick

# ── MoveIt frames / group ─────────────────────────────────────────────────────
PLANNING_FRAME = 'base_link'
TCP_LINK       = 'gripper_tcp'
ARM_GROUP      = 'arm'
ARM_JOINTS = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
              'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
NAMED_STATES = {
    'home':   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'ready':  [0.0, -0.5, -0.6, 1.1, 0.0, 0.0],
    # Travel pose: joint2 tilts arm forward, joint3+4 fold it DOWN so the arm's
    # centre of mass is low and close to the body. This minimises the pendulum
    # torque on joint2 (horizontal-axis hinge) during navigation, stopping the
    # vibration that occurs when the arm is straight up (home = worst case).
    'travel': [0.0,  1.2, -0.6, -0.6, 0.0, 0.0],
}

# ── Grasp geometry (calibrated, from arm_grasp_test) ─────────────────────────
TOPDOWN_QUAT = (-0.7071, 0.0, 0.0, 0.7071)
GRASP_OFFSET = (0.0, 0.013, 0.0)   # perception → grasp target offset (calibrated: x/z pulled to box centre; y kept)
FIXED_BOX    = (0.28, 0.015, 0.08)     # fallback if perception unavailable
PRE_ABOVE    = 0.07                    # TCP above box centre at pre-grasp
GRASP_ABOVE  = 0.0    # TCP above box centre at grasp. 0 since 2026-08-01: gripper_tcp was
                       # moved to the real grasp point (ackermann_with_sensor.xacro),
                       # so the tool goes straight to the box centre. Was 0.06, which
                       # was silently correcting for the TCP sitting on the palm.

# Table collision in planning scene (add it so MoveIt routes around it)
TABLE_SIZE     = (0.10, 0.32, 0.10)   # metres (world: 0.30×0.08×0.10; pad Y/X slightly)
BOX_HALF_H     = 0.02                  # half of 4 cm box

# ── Stacking geometry (vertical box-stacking — thesis target task) ────────────
# The robot picks a box and places it on a HARDCODED foundation (per supervisor scope),
# then stacks each subsequent box on top (z grows by one box height per level).
# All three numbers are LIVE params (stack_x/stack_y/stack_surface_z) — calibrate them
# in sim with `ros2 param set` exactly like the grasp offset, NO relaunch needed.
BOX_HEIGHT       = 2 * BOX_HALF_H      # 0.04 m — full box height = one stack level
STACK_X_DEFAULT  =  0.26               # base_link x of the stack foundation (≈ pick reach)
STACK_Y_DEFAULT  = -0.12               # base_link y — offset to the side of the pick spot
STACK_SURFACE_Z  =  0.06               # base_link z of the foundation surface top (table top)
STACK_HOVER      =  0.10               # TCP hover above the current stack top before placing
# FALLBACK ONLY -- live value is self._stop_distance, from config/scene.yaml.
STOP_DISTANCE    =  0.24               # box distance from base_link at the pickup dock (arm reach limit)
# Where the FINAL camera reading is taken, before the last short blind hop to the dock.
# Chosen from geometry + observation: with the 0.14 m pickup table the WHOLE box is inside
# the camera's vertical field of view down to 0.266 m (camera is 0.065 m up, looking level,
# 22.8 deg of downward view). Below that the box slides off the bottom of the image.
# 0.27 is the last distance where a reading is trustworthy; 0.24 is the furthest the arm can
# actually reach. Measuring at 0.27 and driving the last 3 cm blind gives a 12x shorter blind
# drive than the old single reading at ~0.60 m, so odometry drift has 12x less room to act.
FINAL_READ_DIST  =  0.27
STACK_COUNT_DEF  =  0                  # 0 = legacy single pick+backup; N>0 = stack N boxes

# ── Place table — fiducial-guided docking ─────────────────────────────────────
# Nav2 pre-dock goal: get the robot onto the table's north-face normal line,
# close enough to see the AprilTag.  x=-4 keeps it on the normal line (centred);
# y=-1.28 is well within tag camera range (~0.72 m standoff) but clear of inflation.
NAV_PLACE_X    = -4.0        # on the table-centre's north normal line
NAV_PLACE_Y    = -0.50       # Nav2 goal well away from table inflation zone.
                             # Old -1.28 was too close → Nav2 "Failed to make progress".
                             # Dock creep covers the remaining 1.25 m from -0.50 to -1.75.
NAV_PLACE_YAW  = -1.5708    # -π/2: robot faces -Y (toward the table face)

# Closed-loop dock parameters (dock_to_tag)
#
# ALL of these are FALLBACKS. The live values are derived in
# nav_pick.launch.py:load_scene() from config/scene.yaml and delivered as ROS
# parameters (self._dock_range, self._place_map_y_dock, self._table_map_x/y).
#
# They used to be hand-computed constants, and every table change meant editing
# them by hand in step with the world file, the launch file and the estimator.
# On 2026-08-03 a table resize updated three of the four and the run failed:
# the estimator still used the old size, aimed the arm 4 cm past the table, IK
# failed, and the box was released in mid-air. Deriving them from one file
# removes that whole class of bug.
#
# DO NOT TUNE THESE. Tune config/scene.yaml.
PLACE_DOCK_RANGE   = 0.20   # fallback: stop when tag is this far from base_link (m)
PLACE_MAP_Y_DOCK   = -1.76  # fallback: map y target for the map-position dock
TABLE_MAP_X        = -4.0   # fallback: place table centre, map X
TABLE_MAP_Y        = -2.0   # fallback: place table centre, map Y
PLACE_DOCK_YAW_TOL = 0.04   # Phase A: heading tight enough to start Phase C (rad ≈ 2.3°)
PLACE_DOCK_BEAR_TOL= 0.04   # Phase A: tag bearing tolerance (rad)
PLACE_DOCK_K_ROT   = 2.0    # Phase A/C rotation gain (rad/s per rad error)

# ── Phase B: lateral centring on the tag's face-normal line ──────────────────
# Squaring the heading (Phase A) makes the chassis PARALLEL to the face normal. It
# does NOT put the chassis ON that normal line. Nav2's xy_goal_tolerance is 0.25 m,
# so the robot routinely stops several cm to one side; Phase A squares it there, and
# Phase C then drives straight forward and arrives just as far off to the side.
#
# The arm has been absorbing that error ever since: place_box() carries lateral
# fallbacks (py -> py*0.6 -> py*0.3) that trade placement accuracy for a reachable
# pose. That is compensating in the wrong subsystem — the base should arrive centred.
#
# Once squared, the tag's y in base_link IS the lateral offset, so the correction is
# a pure sideways shift. A differential base cannot strafe, so it is done as a crab:
# pivot 90 deg, drive the offset, pivot back. The drive leg runs PERPENDICULAR to the
# approach axis and so never moves the robot toward the table — which is how this
# differs from the 2026-07 "Phase B steering trim" that drove into it and was reverted.
PLACE_DOCK_LAT_TOL  = 0.02   # m — offset from the normal line that counts as centred
PLACE_DOCK_LAT_MAX  = 0.35   # m — refuse to crab further; a bigger reading is a bad tag
PLACE_DOCK_CRAB_ROT = 0.8    # rad/s for the 90 deg pivots
PLACE_DOCK_CRAB_FWD = 0.10   # m/s for the sideways leg — slow, it is open-loop on odom
PLACE_DOCK_LAT_ITERS = 3     # re-measure and repeat; each pass removes odometry error
PLACE_DOCK_K_FWD   = 0.8    # Phase C forward gain (m/s per m range error)
PLACE_DOCK_MAX_ROT = 0.80   # max rotation speed (rad/s)
# Minimum turn rate for Phase A (rotate in place). Proportional control alone
# commands 2.0*0.04 = 0.08 rad/s (4.6 deg/s) at the tolerance, so the last few
# degrees crawl. NOT applied in Phase C, where angular.z is a steering trim
# while driving — a floor there would make the robot weave.
PLACE_DOCK_MIN_ROT = 0.25
PLACE_DOCK_MIN_FWD = 0.05   # min forward speed during approach (m/s)
PLACE_DOCK_MAX_FWD = 0.12   # max forward speed during approach (m/s)

# Arm geometry for placing
BOX_HEIGHT     =  0.04      # full box height (one stack level)
PLACE_BOX_Z    =  0.08      # table-top z in base_link (same as pickup table)
PLACE_HOVER    =  0.12      # TCP height above place surface before descend
PLACE_ABOVE    =  0.04      # TCP height above box centre at release

# ── Planning ──────────────────────────────────────────────────────────────────
PLAN_ATTEMPTS = 10
PLAN_TIME_SEC =  5.0
VEL_SCALE     =  0.2
ACC_SCALE     =  0.2
# 30 s was too tight for some real executions (a straight-line retimed trajectory
# legitimately took 62 s once) -- the client gave up and reported FAILED while the
# goal kept running server-side, unmonitored, with the orchestrator already moving
# on to the next command. Raised, and both wait sites now cancel on timeout instead
# of abandoning the goal. Diagnosed 2026-08-12 after this exact bug (already fixed
# in real_grasp_test.py the night before) recurred here.
EXEC_TIMEOUT_SEC = 90.0
# Per-trajectory-point allowance used to scale the execution timeout (see
# _exec_timeout). The retimed pace is ~0.5 s/point at VEL_SCALE 0.2; 0.75 leaves
# headroom without letting a genuinely stuck move hang forever.
EXEC_SEC_PER_POINT = 0.75

# ── Cartesian (straight-line) motion ─────────────────────────────────────────
# go_pose() plans in JOINT space via OMPL/RRTConnect, which optimises for path
# VALIDITY, not for the shape the tool traces. A commanded vertical move therefore
# arrives along an arc. MEASURED 2026-07-28: a commanded 0.060 m lift produced only
# 0.031 m of actual rise — roughly half the motion went sideways (see
# _verify_grasp_truth's docstring). On the way DOWN that same lateral component is
# what clips the box before the fingers are around it.
#
# For the short approach/retreat segments either side of a grasp or a place, the
# straight line is the whole point, so those use /compute_cartesian_path instead.
CART_MAX_STEP     = 0.002   # interpolation step (m). 2 mm over a 60 mm move = 30 waypoints.
# Reduced from 5 mm on 2026-08-21. computeCartesianPath seeds each waypoint's IK from
# the PREVIOUS waypoint's solution; the closer the waypoints, the closer the seed, and
# the less often kdl_kinematics_plugin fails to converge and falls back to a RANDOM
# restart (kinematics_solver_attempts: 20). Every random restart is a chance to land on
# a different IK branch, and near the workspace boundary -- where this grasp lives, at
# 99.4% of measured max reach -- that happens often. Measured on the 2026-08-21 run at
# 5 mm: total joint path 3.240 rad for a 0.060 m lift (54 rad/m, against ~5-8 rad/m for
# a clean descend) with a 0.1404 rad single-step outlier against a 0.0546 median.
CART_MIN_FRACTION = 0.90    # accept the path only if the interpolator covered >= 90%
# Max joint travel per metre of Cartesian motion before a "straight-line" path is
# rejected as a zigzag. A clean single-branch descent on this arm measures ~5 rad/m;
# the pathological paths measured 170-403. See the gate in go_pose_straight().
CART_JOINT_TRAVEL_MAX = 15.0
MOVEIT_SUCCESS = 1

# ── Base pin ──────────────────────────────────────────────────────────────────
PIN_ROBOT_NAME = 'mbot'


class NavPickOrchestrator(Node):

    def __init__(self):
        super().__init__('nav_pick_orchestrator')

        # Nav2
        self._nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # MoveIt
        self._move  = ActionClient(self, MoveGroup, '/move_action')
        # Straight-line motion: plan with /compute_cartesian_path, then run the
        # returned trajectory through /execute_trajectory. MoveGroup's own action
        # cannot do this — it only takes goal CONSTRAINTS, never a precomputed path.
        self._cart  = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        # Single-solve IK, used by go_pose_branch() for the near-boundary descend/lift.
        self._ik    = self.create_client(GetPositionIK, '/compute_ik')
        self._exec  = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self._scene = self.create_client(ApplyPlanningScene, '/apply_planning_scene')

        # Gripper
        self._gripper      = self.create_publisher(Float64MultiArray, GRIPPER_TOPIC, 10)
        self._attach_pub   = self.create_publisher(Bool, '/grasp_attach', 10)
        self._last_gripper   = [0.0] * len(GRIPPER_OPEN)   # 1 joint (mimic followers)
        self._weld_active    = False
        self._gripper_actual = GRIPPER_OPEN[0]   # actual position from /joint_states
        self._arm_actual = None                  # live ARM joint positions (IK seed)
        self._bumper_contact = False              # bumper oracle from smart_grasp
        self.create_subscription(Bool, '/grasp_attach', self._weld_cb, 10)
        self.create_subscription(JointState, '/joint_states',
                                 self._gripper_joint_cb, 10)
        self.create_subscription(Bool, '/grasp_bumper_contact',
                                 self._bumper_cb, 10)

        # Perception
        self._latest_box = None
        self.create_subscription(PoseStamped, '/box_pose', self._box_cb, 10)
        # Age (s) since box_pose_estimator's last REAL detection — /box_pose itself
        # keeps replaying the last-known position even when nothing is visible right
        # now, so this is the only way to tell a live sighting from a stale memory.
        self._box_age = float('inf')
        self.create_subscription(Float64, '/box_detection_age', self._box_age_cb, 10)

        # Odometry (for the blind final approach — drive a measured distance when the
        # real-spec camera can no longer see the box closer than 0.30 m)
        self._odom = None
        self._odom_yaw = None
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        # Box (x,y,z) in base_link recorded at the dock: x from the odometry-measured
        # distance (TRUE), y/z from the accurate far perception. Used for the grasp so
        # we don't aim at the camera's inflated close-range distance.
        self._dock_box = None
        # Table top height in base_link derived from the first perception reading.
        # Eliminates the need to hand-tune stack_surface_z for each new deployment.
        self._table_top_base_z = None

        # Name of the box currently in the gripper — set by _claim_held_box(), which
        # also removes it from _remaining_boxes. Keeps ground-truth queries pointed at
        # the carried box. See _candidate_box_names().
        self._held_box_name = None
        self._target_box_name = None   # set by _verify_grasp_truth once a box is seen to rise

        # Which grasp mechanism is running — see the USE_WELD comment block at the top.
        # Must match what nav_pick.launch.py started; the launch file passes the same
        # value it used to gate grasp_attacher/smart_grasp.
        self.declare_parameter('use_physics_grasp', True)
        self._physics_grasp = bool(self.get_parameter('use_physics_grasp').value)
        self.get_logger().info(
            f'grasp mechanism = '
            f'{"PHYSICS (libgazebo_grasp_plugin real fixed joint)" if self._physics_grasp else "LEGACY kinematic weld (grasp_attacher)"}')

        # base_pin_enabled:=false — experiment: does the base actually creep during
        # the arm's motion without the 50 Hz teleport hold? pin_base() becomes a
        # no-op; every existing pin_base()/unpin_base() call site is untouched.
        self.declare_parameter('base_pin_enabled', True)
        self._base_pin_enabled = bool(self.get_parameter('base_pin_enabled').value)
        if not self._base_pin_enabled:
            self.get_logger().warn('base_pin_enabled=false — base will NOT be held during arm motion')

        # ── Calibration knobs (live, tune with `ros2 param set` — NO relaunch) ──
        self.declare_parameter('grasp_off_x', GRASP_OFFSET[0])
        self.declare_parameter('grasp_off_y', GRASP_OFFSET[1])
        self.declare_parameter('grasp_off_z', GRASP_OFFSET[2])
        self.declare_parameter('calib_loops', 0)     # 0 = normal single run; N = repeat the grasp N times
        self.declare_parameter('calib_pause', 6.0)   # seconds to observe between iterations
        # 'target_box' no longer exists — the world now has 3 distinct stack_box_N
        # boxes. stack_box_1 sits exactly at the box_reset_xyz default below, so
        # calib_loop()/run_stack()'s reset_box() has a real entity to respawn.
        # MUST match a model that actually exists in the world. Was 'stack_box_1',
        # which was deleted on 2026-07-28 during the switch to single-box testing —
        # so reset_box() teleported a non-existent model, logged success anyway, and
        # the real box was never put back on the table. Every calib iteration after
        # the first then grasped at an empty spot (observed: iters 2-10 aborted in
        # under a second each).
        self.declare_parameter('box_name', 'stack_box_0')
        # z = pickup table top 0.14 + half box height 0.02 = 0.16 (table raised 2026-08-02
        # so the box enters the camera's vertical field of view at grasp range).
        # Must match the box's spawn pose in worlds/final_map.world. Moved to y=4.02
        # on 2026-08-25 (pickup_table.box_offset = 0.02): 20 mm toward the robot, so
        # the grasp sits mid-band instead of at the arm's maximum reach.
        self.declare_parameter('box_reset_xyz', [-2.0, 4.02, 0.16])

        # ── Stacking knobs (live, tune with `ros2 param set` — NO relaunch) ──────
        # Straight-line approach/retreat either side of a grasp or a place.
        # Live param so the arcing joint-space behaviour can be A/B'd against the
        # Cartesian one WITHOUT a rebuild:
        #     ros2 param set /nav_pick_orchestrator use_cartesian_descent false
        # Only the SHORT vertical segments use it; long transfers stay joint-space,
        # where a straight tool path is neither wanted nor reliably solvable.
        self.declare_parameter('use_cartesian_descent', True)

        # How far the gripper is allowed to close (rad). 0 = just touching,
        # negative = squeezing further; -0.20 is fully closed. Live param —
        # tune without rebuilding:
        #     ros2 param set /nav_pick_orchestrator grasp_close_floor -0.15
        self.declare_parameter('grasp_close_floor', -0.13)

        # Phase B — put the base ON the tag's face-normal line before driving in.
        # Live param, same A/B reasoning as above:
        #     ros2 param set /nav_pick_orchestrator dock_lateral_align false
        self.declare_parameter('dock_lateral_align', True)

        self.declare_parameter('stack_count', STACK_COUNT_DEF)   # >0 → stack this many boxes
        self.declare_parameter('stack_x', STACK_X_DEFAULT)       # base_link x of foundation
        self.declare_parameter('stack_y', STACK_Y_DEFAULT)       # base_link y of foundation
        self.declare_parameter('stack_surface_z', STACK_SURFACE_Z)  # base_link z of surface top

        # ── Tag-dock knobs (live, tune with `ros2 param set` — NO relaunch) ────
        # ── Scene geometry, supplied by nav_pick.launch.py from config/scene.yaml.
        # The module constants are fallbacks for running this node bare.
        # Single source of truth: config/scene.yaml. Never tune these here.
        self.declare_parameter('dock_range',      PLACE_DOCK_RANGE)    # tag distance at stop
        self.declare_parameter('table_map_x',        TABLE_MAP_X)
        self.declare_parameter('table_map_y',        TABLE_MAP_Y)
        self.declare_parameter('place_map_y_dock',   PLACE_MAP_Y_DOCK)
        self.declare_parameter('nav_place_x',        NAV_PLACE_X)
        self.declare_parameter('nav_place_y',        NAV_PLACE_Y)
        self.declare_parameter('nav_place_yaw',      NAV_PLACE_YAW)
        self.declare_parameter('nav_goal_x',         NAV_GOAL_X)
        self.declare_parameter('nav_goal_y',         NAV_GOAL_Y)
        self.declare_parameter('nav_goal_yaw',       NAV_GOAL_YAW)
        self.declare_parameter('stop_distance',      STOP_DISTANCE)
        # base_link-frame table surface heights, derived in nav_pick.launch.py
        # from config/scene.yaml (world top_z - robot.base_link_ground_z).
        # place_surface_base_z is the SOURCE OF TRUTH for place_box(): there is
        # no perception of the place table at place time (unlike pickup, which
        # sees the box and derives its own surface height live), so this
        # scene-derived constant is what release height is computed from.
        #
        # Bug fixed 2026-08-04: place_box() used to fall back to
        # self._table_top_base_z, a value set ONLY during pickup docking from
        # the PICKUP table's perceived height. Reused unchanged for the place
        # table (4 cm shorter in world), it released the box ~6 cm above an
        # 8x8 cm table -- enough to bounce/roll it onto the floor. place_success
        # still read True because it was never checked against ground truth.
        # See metrics_20260803_195910.csv (err_xy 10 cm) and
        # metrics_20260803_195111.csv (err_xy 89 cm) -- both landed on the
        # floor (box world z=0.020), not the table.
        self.declare_parameter('place_surface_base_z',  -0.045)   # fallback: 0.10-0.145
        self.declare_parameter('pickup_surface_base_z', -0.005)   # fallback: 0.14-0.145
        self.declare_parameter('place_table_top_z_world', 0.10)   # world frame, for metrics ground truth

        g = lambda n: float(self.get_parameter(n).value)
        self._table_map_x       = g('table_map_x')
        self._table_map_y       = g('table_map_y')
        self._place_map_y_dock  = g('place_map_y_dock')
        self._nav_place_x       = g('nav_place_x')
        self._nav_place_y       = g('nav_place_y')
        self._nav_place_yaw     = g('nav_place_yaw')
        self._nav_goal_x        = g('nav_goal_x')
        self._nav_goal_y        = g('nav_goal_y')
        self._nav_goal_yaw      = g('nav_goal_yaw')
        self._stop_distance     = g('stop_distance')
        self._place_surface_base_z = g('place_surface_base_z')
        self._place_table_top_z_world = g('place_table_top_z_world')
        self.get_logger().info(
            f'[scene] place table centre map=({self._table_map_x:.2f},'
            f'{self._table_map_y:.2f}) dock_range={g("dock_range"):.3f} '
            f'stop_distance={self._stop_distance:.3f} '
            f'place_surface_base_z={self._place_surface_base_z:+.4f}')
        self.declare_parameter('place_stack_levels', 1)                 # boxes to stack at place table
        # How far to pull the place drop point toward the robot, from the tag-derived
        # table centre. Live-tunable so it can be matched to the measured band without
        # a rebuild:  ros2 param set /nav_pick_orchestrator place_near_edge_bias 0.04
        #
        # 0.03 since 2026-08-25. The place DESCENT pose sits at base_link z = -0.025
        # (place surface -0.045 + half box), near the bottom of the workspace, where
        # reach_map.csv shows only TWO reachable columns at y=0: x = 0.22 and x = 0.24.
        # A 0.02 bias landed the target at 0.233 -- between them -- and IK came back
        # 1.766 rad from the arm's pose, was refused, and the Cartesian fallback built
        # a 90-waypoint 13.8 rad path. 0.03 puts the target at ~0.223, on the 0.22
        # column. Contrast the RETRACT pose at z = +0.075, which has five reachable
        # columns and worked at 0.54 rad on the same run.
        self.declare_parameter('place_near_edge_bias', 0.03)
        self.declare_parameter('place_yaw_offset', 0.0)                 # calibration offset for arm yaw

        # Tag-based place dock (tag_dock_estimator topics)
        self._latest_tag_pose  = None   # /place_tag_pose  PoseStamped in base_link
        self._latest_drop_pt   = None   # /place_drop_point PointStamped in base_link
        self._latched_drop     = None   # (x, y) latched at Phase D
        self._latched_tag_yaw  = None   # tag yaw (rad) latched for arm orientation

        # TF buffer — used by the map-position fallback dock when tag detection fails
        self._tf_buf      = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)
        self.create_subscription(PoseStamped,  '/place_tag_pose',   self._tag_pose_cb,   10)
        self.create_subscription(PointStamped, '/place_drop_point', self._drop_point_cb, 10)

        # cmd_vel (for backup)
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # base_pin (Gazebo services)
        self._get_state = self.create_client(GetEntityState, '/get_entity_state')
        self._set_state = self.create_client(SetEntityState, '/set_entity_state')
        self._pin_pose  = None
        self._pin_timer = None

        # ── Metrics (Phase 4) ────────────────────────────────────────────────────
        self._metrics: list[dict] = []   # one dict per cycle, flushed to CSV at end
        self._cycle_start: float  = 0.0
        self.declare_parameter('metrics_csv', '')   # empty = auto-name under data/

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _box_cb(self, msg):
        self._latest_box = msg

    def _box_age_cb(self, msg):
        self._box_age = msg.data

    def _tag_pose_cb(self, msg: PoseStamped):
        self._latest_tag_pose = msg

    def _drop_point_cb(self, msg: PointStamped):
        self._latest_drop_pt = msg

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        self._odom = (p.x, p.y)
        # Heading too — the lateral centring maneuver (Phase B) turns by a commanded
        # angle, and closing that loop on odometry is what makes it a repeatable
        # 90 degrees instead of "however far it got in N seconds".
        q = msg.pose.pose.orientation
        self._odom_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _odom_dist_since(self, x0, y0):
        """Straight-line distance the base has driven since (x0, y0) in odom."""
        if self._odom is None:
            return None
        return math.hypot(self._odom[0] - x0, self._odom[1] - y0)

    # ── Metrics harness (Phase 4) ─────────────────────────────────────────────

    def _cycle_begin(self, cycle_idx: int):
        """Mark the start of cycle `cycle_idx` (0-indexed)."""
        self._cycle_start = time.time()
        self._current_cycle = {
            'cycle':              cycle_idx,
            'box_name':           None,
            'pick_success':       False,
            'transport_retained': False,
            'dock_via_tag':       False,
            'place_success':      False,
            'placement_err_x':    float('nan'),
            'placement_err_y':    float('nan'),
            'placement_err_z':    float('nan'),
            'cycle_time_s':       float('nan'),
        }

    def _claim_held_box(self):
        """Identify which box in self._remaining_boxes is currently held (elevated
        well above the pickup-table rest height, e.g. after go_named('home')), and
        remove it from the remaining list. Distinct boxes replace the old single
        recycled target_box, so we must track which physical box was actually
        grasped for correct per-box metrics. Falls back to popping the first
        remaining name if the Gazebo query is unavailable or inconclusive."""
        HELD_Z_THRESH = 0.16   # table-rest z=0.12; a held/retracted box reads much higher
        best_name, best_z = None, -1.0
        for name in self._remaining_boxes:
            if not self._get_state.service_is_ready():
                continue
            req = GetEntityState.Request()
            req.name = name
            req.reference_frame = 'world'
            fut = self._get_state.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
            if fut.done() and fut.result() is not None and fut.result().success:
                z = fut.result().state.pose.position.z
                if z > best_z:
                    best_name, best_z = name, z
        if best_name is None or best_z < HELD_Z_THRESH:
            held = self._remaining_boxes.pop(0)
            self.get_logger().warn(f'_claim_held_box: inconclusive query — assuming {held}')
            self._held_box_name = held
            return held
        self._remaining_boxes.remove(best_name)
        self.get_logger().info(f'_claim_held_box: identified held box = {best_name} (z={best_z:.3f})')
        # Remember it: this call REMOVES the box from _remaining_boxes, and every
        # ground-truth helper (_box_world_pose, _box_world_z, _box_is_held) searches
        # that list. Without this the carried box becomes invisible to them and they
        # silently fall back to a hardcoded name list — which happens to start with
        # 'stack_box_0' and so works by luck in the single-box case, but would query
        # the WRONG box as soon as there is more than one.
        self._held_box_name = best_name
        return best_name

    def _cycle_measure_placement(self, expected_x: float, expected_y: float,
                                 expected_z: float, box_name: str = None):
        """Query Gazebo for the actual box position and record XYZ error."""
        if not self._get_state.service_is_ready():
            return
        name = box_name if box_name is not None else self.get_parameter('box_name').value
        req = GetEntityState.Request()
        req.name = name
        req.reference_frame = 'world'
        fut = self._get_state.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        if not fut.done() or not fut.result().success:
            self.get_logger().warn('[metrics] get_entity_state failed — placement error not recorded')
            return
        p = fut.result().state.pose.position
        self._current_cycle['placement_err_x'] = round(p.x - expected_x, 4)
        self._current_cycle['placement_err_y'] = round(p.y - expected_y, 4)
        self._current_cycle['placement_err_z'] = round(p.z - expected_z, 4)
        err_xy = math.hypot(p.x - expected_x, p.y - expected_y)
        self.get_logger().info(
            f'[metrics] box world=({p.x:.3f},{p.y:.3f},{p.z:.3f}) '
            f'target=({expected_x:.3f},{expected_y:.3f},{expected_z:.3f}) '
            f'err_xy={err_xy:.4f} m')

    def _cycle_end(self):
        """Finalise the current cycle dict and append to the metrics list."""
        if not hasattr(self, '_current_cycle'):
            return
        self._current_cycle['cycle_time_s'] = round(time.time() - self._cycle_start, 2)
        self._metrics.append(self._current_cycle)
        self.get_logger().info(
            f'[metrics] cycle {self._current_cycle["cycle"]} done: '
            f'pick={self._current_cycle["pick_success"]} '
            f'retain={self._current_cycle["transport_retained"]} '
            f'place={self._current_cycle["place_success"]} '
            f't={self._current_cycle["cycle_time_s"]}s')

    def flush_metrics(self):
        """Write all collected metrics to a CSV file under data/."""
        if not self._metrics:
            return
        csv_path = self.get_parameter('metrics_csv').value
        if not csv_path:
            data_dir = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                '..', '..', '..', '..', '..', 'data')
            os.makedirs(data_dir, exist_ok=True)
            ts = time.strftime('%Y%m%d_%H%M%S')
            csv_path = os.path.join(data_dir, f'metrics_{ts}.csv')
        fieldnames = list(self._metrics[0].keys())
        with open(csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self._metrics)
        self.get_logger().info(f'[metrics] wrote {len(self._metrics)} rows → {csv_path}')

    def _grasp_attempts_csv_path(self):
        if not hasattr(self, '_grasp_csv_path'):
            data_dir = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                '..', '..', '..', '..', '..', 'data')
            os.makedirs(data_dir, exist_ok=True)
            ts = time.strftime('%Y%m%d_%H%M%S')
            self._grasp_csv_path = os.path.join(data_dir, f'grasp_attempts_{ts}.csv')
        return self._grasp_csv_path

    def _log_grasp_attempt(self, attempt, classifier_result, verified):
        """Phase 0 — append one row per grasp attempt immediately (not just at
        run end), so ground-truth vision-verified outcomes survive a crash."""
        path = self._grasp_attempts_csv_path()
        cycle = self._current_cycle['cycle'] if hasattr(self, '_current_cycle') else -1
        # stall_angle is the raw measurement the old classifier used to gate on.
        # It no longer decides anything, but logging it every attempt builds the
        # real dataset needed to calibrate BOX_CONTACT_ANGLE properly.
        # Centring, measured while the box is held aloft:
        #   jaw_mm   — between the jaws (0 = perfectly gripped)
        #   slide_mm — along the flat finger faces; |slide| > 18 mm means the box is
        #              past the edge of the 36 mm face. THIS is the component
        #              grasp_off_y corrects. A consistent sign across trials is a
        #              systematic offset; scattered signs are positioning noise.
        cen = getattr(self, '_last_centering', None)
        row = {
            'timestamp':         time.strftime('%Y-%m-%d %H:%M:%S'),
            'cycle':             cycle,
            'attempt':           attempt,
            'classifier_result': str(classifier_result),
            'stall_angle':       getattr(self, '_last_stall_angle', None),
            'verified_success':  verified,
            'centering_jaw_mm':   round(cen[0] * 1000, 2) if cen else None,
            'centering_slide_mm': round(cen[1] * 1000, 2) if cen else None,
        }
        self._last_centering = None   # don't carry a stale reading into the next row
        write_header = not os.path.exists(path)
        with open(path, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(row.keys()))
            if write_header:
                writer.writeheader()
            writer.writerow(row)
        self.get_logger().info(f'[verify] logged grasp attempt → {path}')

    def _offset(self):
        """Current grasp offset (live ROS params — tunable without relaunch)."""
        return (self.get_parameter('grasp_off_x').value,
                self.get_parameter('grasp_off_y').value,
                self.get_parameter('grasp_off_z').value)

    def reset_box(self):
        """Respawn the target box on the table (known world pose) for the next pick."""
        name = self.get_parameter('box_name').value
        x, y, z = self.get_parameter('box_reset_xyz').value
        st = EntityState()
        st.name = name
        st.pose.position = Point(x=float(x), y=float(y), z=float(z))
        st.pose.orientation.w = 1.0
        st.reference_frame = 'world'
        req = SetEntityState.Request(); req.state = st
        if self._set_state.wait_for_service(timeout_sec=3.0):
            fut = self._set_state.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
            self.get_logger().info(f'reset box "{name}" → ({x:.2f}, {y:.2f}, {z:.2f})')

    # ── Step 1: Navigate ──────────────────────────────────────────────────────

    def _activate_nav2_cmdvel(self):
        """Re-activate the Nav2 cmd_vel nodes (in case a previous mission
        deactivated them for docking) so navigation can drive again."""
        for node in ('controller_server', 'velocity_smoother', 'behavior_server',
                     'collision_monitor'):
            cli = self.create_client(ChangeState, f'/{node}/change_state')
            if not cli.wait_for_service(timeout_sec=3.0):
                continue
            req = ChangeState.Request()
            req.transition.id = Transition.TRANSITION_ACTIVATE
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)

    def navigate_to_table(self):
        self.get_logger().info('=== Step 1: Navigate to table ===')
        self._activate_nav2_cmdvel()
        # The orchestrator no longer seeds AMCL. It used to publish a HARDCODED
        # (-2.0, 7.0, -pi/2) to /initialpose here, which was wrong twice over: dead
        # under ground-truth localization (no AMCL exists to subscribe, so the 15 s
        # wait loop below it just stalled every run), and redundant under AMCL, which
        # is already seeded by set_initial_pose in nav2_limo_diff.yaml. Worse, the
        # constant did not track the spawn_y launch argument, so moving the robot left
        # this publishing the old pose. Seeding now happens in exactly one place --
        # nav2_limo.launch.py, from scene.yaml's robot.spawn_pose.

        if not self._nav.wait_for_server(timeout_sec=30.0):
            self.get_logger().error(
                'NavigateToPose action server not available — '
                'is nav2_limo.launch.py running?')
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self._nav_goal_x
        goal.pose.pose.position.y = self._nav_goal_y
        # Yaw NAV_GOAL_YAW → quaternion
        half = self._nav_goal_yaw / 2.0
        goal.pose.pose.orientation.z = math.sin(half)
        goal.pose.pose.orientation.w = math.cos(half)

        self.get_logger().info(
            f'sending Nav2 goal → ({self._nav_goal_x:.2f}, {self._nav_goal_y:.2f}, '
            f'yaw={self._nav_goal_yaw:.3f})')

        gh = None
        for attempt in range(15):
            send_fut = self._nav.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_fut, timeout_sec=10.0)
            if send_fut.done():
                gh = send_fut.result()
                if gh is not None and gh.accepted:
                    self.get_logger().info('Nav2 goal accepted — driving to table…')
                    break
            self.get_logger().warn(f'Nav2 goal rejected or timed out (attempt {attempt+1}/15) — waiting for bt_navigator to become active...')
            end_time = self.get_clock().now() + Duration(seconds=2.0)
            while self.get_clock().now() < end_time:
                rclpy.spin_once(self, timeout_sec=0.1)
        else:
            self.get_logger().error('Nav2 goal rejected after 15 attempts — aborting')
            return False
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=NAV_TIMEOUT_SEC)
        if not res_fut.done():
            self.get_logger().error(f'Nav2 did not complete within {NAV_TIMEOUT_SEC} s')
            return False

        res = res_fut.result()
        if res.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f'Nav2 navigation failed with status: {res.status}')
            return False

        self.get_logger().info('Nav2 navigation complete ✓')
        return True

    # ── Nav2 cmd_vel hand-off ─────────────────────────────────────────────────

    def _align_to_place_tag(self):
        """Slide sideways to face the place table's tag square-on, before docking.

        Runs while Nav2 still owns /cmd_vel -- i.e. BEFORE _silence_nav2_cmdvel() --
        because this uses a Nav2 goal, and the dock deactivates the controller server
        the moment it takes over.

        Same reasoning as the pickup side: Nav2's goal tolerance is 0.25 m, so the
        robot stops anywhere in a half-metre circle around the staging pose, and the
        dock's Phase A then SQUARES THE HEADING wherever that happens to be. Any
        lateral offset left at that point turns into a diagonal approach over the
        ~1.3 m Phase C run.

        This is the same job Phase B's crab does, done with a Nav2 goal instead of two
        in-place 90 degree pivots. The crab is the most slip-prone motion in the
        pipeline and has been observed timing out; it stays in place as a fallback,
        and with this running ahead of it there should be little left for it to correct.

        Non-fatal: if the tag is not visible from the standoff, docking proceeds
        exactly as before.
        """
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.1)
        t = self._read_tag_live()
        if t is None:
            self.get_logger().info(
                '[realign] tag not visible from the standoff — skipping alignment')
            return
        # _read_tag_live returns (x, y, range, bearing, yaw) in base_link; y is the
        # lateral offset, which is what the approach centre line needs corrected.
        self._nav_place_x = self.realign_perpendicular(
            t[1], self._nav_place_x, self._nav_place_y, self._nav_place_yaw,
            label='place tag')

    def _align_to_target_box(self):
        """Read the target box at staging range and slide sideways to face it square-on.

        Runs BEFORE visual_docking(), because Phase A's rotate-to-centre is exactly
        what turns a lateral offset into a diagonal approach. Reading here also costs
        nothing extra: the robot is stationary at the staging pose and the box is ~0.73 m
        away, comfortably inside the camera's usable range.

        Non-fatal by design. If the box is not visible yet, or Nav2 declines the
        alignment goal, the cycle proceeds exactly as it did before this step existed --
        diagonal, but working.
        """
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.1)
        b = self._read_box_live()
        if b is None:
            self.get_logger().info(
                '[realign] no box reading at the staging pose — skipping alignment')
            return
        self._nav_goal_x = self.realign_perpendicular(
            b[1], self._nav_goal_x, self._nav_goal_y, self._nav_goal_yaw,
            label='target box')

    def realign_perpendicular(self, target_base_y, goal_x, goal_y, goal_yaw,
                              label='target', tolerance=0.02, timeout=60.0):
        """Slide sideways via Nav2 so the approach stays PERPENDICULAR to the table.

        THE PROBLEM THIS SOLVES. visual_docking()'s Phase A rotates in place until the
        target is centred, then Phase B drives straight at it. For a target directly
        ahead that is correct. For one offset sideways it is not: the robot pivots to
        FACE the target and then drives along that new heading, so it reaches the box
        travelling DIAGONALLY across the table instead of square-on. With three boxes
        spread across the 0.30 m pickup table the outer ones sit 0.10 m off the staging
        centre line, which at the 0.73 m staging distance is a 7.8 degree approach
        angle. The gripper then closes on the box rotated by that angle, and the
        chassis ends up beside the table rather than in front of it.

        THE FIX. Translate instead of rotate. The robot cannot strafe -- it is
        differential drive -- but Nav2 can reposition it, so this re-issues the staging
        goal shifted sideways to line up with the target, keeping the approach yaw
        unchanged. Phase A then has almost nothing left to rotate and Phase B drives in
        square-on.

        Deliberately NOT a crab manoeuvre (rotate 90, drive, rotate back). That is what
        the place-side Phase B does, it is the most slip-prone motion a differential
        base can make, and it has been observed timing out. Nav2 already solves
        "get to this pose" properly.

        target_base_y is the target's lateral offset in base_link (positive = left).
        Both tables are approached at yaw = -pi/2, where base_link +Y maps to map +X,
        so the correction is applied directly to the goal's x.
        """
        if abs(target_base_y) <= tolerance:
            self.get_logger().info(
                f'[realign] {label} is {target_base_y*1000:+.0f} mm off centre — '
                f'within {tolerance*1000:.0f} mm, approach is already perpendicular')
            return goal_x

        new_x = goal_x + target_base_y
        self.get_logger().info(
            f'[realign] {label} is {target_base_y*1000:+.0f} mm off the approach centre '
            f'line. Sliding the staging pose {new_x - goal_x:+.3f} m in map x '
            f'({goal_x:.3f} -> {new_x:.3f}) so the approach stays perpendicular '
            f'instead of diagonal.')

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = new_x
        goal.pose.pose.position.y = goal_y
        half = goal_yaw / 2.0
        goal.pose.pose.orientation.z = math.sin(half)
        goal.pose.pose.orientation.w = math.cos(half)

        send_fut = self._nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut, timeout_sec=10.0)
        gh = send_fut.result() if send_fut.done() else None
        if gh is None or not gh.accepted:
            self.get_logger().warn(
                '[realign] Nav2 rejected the alignment goal — continuing with the '
                'diagonal approach rather than aborting the cycle')
            return goal_x

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=timeout)
        if not res_fut.done():
            self.get_logger().warn('[realign] alignment goal timed out — continuing')
            return goal_x

        self.get_logger().info('[realign] aligned — approach is now perpendicular')
        # Returned so the caller can keep ITS goal in sync: a later realign must
        # measure from where the robot now is, not the original staging pose.
        return new_x

    def _silence_nav2_cmdvel(self):
        """Deactivate the Nav2 nodes that publish /cmd_vel so the orchestrator can
        own the topic during docking. collision_monitor publishes safety-stop zeros
        when it receives no input on cmd_vel_smoothed (after smoother deactivates),
        which overrides direct dock-drive commands."""
        for node in ('collision_monitor', 'velocity_smoother', 'controller_server',
                     'behavior_server'):
            cli = self.create_client(ChangeState, f'/{node}/change_state')
            if not cli.wait_for_service(timeout_sec=3.0):
                self.get_logger().warn(f'{node}/change_state unavailable — skipping')
                continue
            req = ChangeState.Request()
            req.transition.id = Transition.TRANSITION_DEACTIVATE
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            self.get_logger().info(f'deactivated {node} (released /cmd_vel)')

    # ── Step 1.5: Visual docking ──────────────────────────────────────────────

    def _read_box_live(self):
        """One fresh /box_pose reading (x, y) in base_link, or None if missing /
        out of plausible range. Used by the docking control loops (needs to be
        responsive, so single readings — not the 8-sample median)."""
        rclpy.spin_once(self, timeout_sec=0.05)
        p = self._latest_box
        if p is None or p.header.frame_id != PLANNING_FRAME:
            return None
        x, y = p.pose.position.x, p.pose.position.y
        if x < 0.10 or x > 1.5:
            return None
        return x, y

    def _read_box_live_xyz(self):
        """Same as _read_box_live but keeps Z. Grasp verification MUST use 3D:
        the lift is straight up, so a correctly-grasped box keeps the same (x, y)
        and only Z changes. An XY-only check therefore reports a good grasp as
        'box still on the table' — which released the weld and dropped the box on
        every attempt (bug introduced and fixed 2026-07-28)."""
        rclpy.spin_once(self, timeout_sec=0.05)
        p = self._latest_box
        if p is None or p.header.frame_id != PLANNING_FRAME:
            return None
        x, y, z = p.pose.position.x, p.pose.position.y, p.pose.position.z
        if x < 0.10 or x > 1.5:
            return None
        return x, y, z

    def _all_box_world_z(self):
        """World z of EVERY candidate box, as {name: z}.

        Ground truth for the grasp check. Deliberately measures all of them rather
        than picking one in advance, because "which box are we holding" is exactly
        what the check is trying to establish, and every attempt to decide it up front
        has been wrong:

          - _candidate_box_names() returns the first name that resolves, which before
            a grasp is claimed is just _remaining_boxes in order -- always
            'stack_box_0'. With three boxes on the table and 'rightmost' targeting an
            outer one, the check measured a box sitting untouched on the table and
            reported "did not move" after a good lift, so the retry opened the fingers
            and dropped the box.
          - a second attempt identified it by proximity to gripper_tcp, but asked TF
            for a 'world' frame that does not exist in this tree (map / odom /
            base_footprint / base_link do). It failed on every call, logged a warning
            nobody read, and silently changed nothing.

        Comparing all of them sidesteps the question: whichever box actually rose is
        the one in the gripper. No frame assumptions, no proximity threshold.
        """
        if not self._get_state.service_is_ready():
            return {}
        out = {}
        for name in self._candidate_box_names():
            req = GetEntityState.Request()
            req.name = name
            req.reference_frame = 'world'      # Gazebo's own frame, not TF
            fut = self._get_state.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
            if fut.done() and fut.result() is not None and fut.result().success:
                out[name] = fut.result().state.pose.position.z
        return out

    def _box_world_z(self):
        """True box height in the Gazebo world frame — SIM GROUND TRUTH.

        Perception is not accurate enough to referee a grasp: measured 2026-07-28,
        a genuinely-grasped box that physically rose 0.06 m was reported by
        /box_pose as having moved only 0.033 m, while a box welded from 0.079 m
        away (fingers closed on air) read 0.063 m. The noise band overlaps the
        signal, so no tolerance can separate them. Gazebo's own state is exact.

        Sim only — /get_entity_state does not exist on real hardware, which is why
        _verify_grasp_vision is kept as the hardware path.
        """
        if not self._get_state.service_is_ready():
            return None
        names = self._candidate_box_names()
        for name in names:
            req = GetEntityState.Request()
            req.name = name
            req.reference_frame = 'world'
            fut = self._get_state.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
            if fut.done() and fut.result() is not None and fut.result().success:
                return fut.result().state.pose.position.z
        return None

    def _measure_grasp_centering(self):
        """How far off-centre did the box end up between the fingers?

        Measured WHILE THE BOX IS HELD UP, which is the only moment the answer is
        unambiguous: the box is rigidly located relative to the gripper, so the
        difference between where the box actually is and where the tool centre is
        IS the grasp error. On the table you cannot tell a good grasp from a lucky
        one; in the air you can.

        Uses Gazebo ground truth for the box and TF for gripper_tcp, so the number
        is exact and free of the perception noise that makes /box_pose unusable as
        a referee (see _box_world_z).

        The lateral component is the calibration signal for grasp_off_y: a
        consistent sign across trials is a systematic offset to correct, not noise.
        Logged per attempt so N trials produce a mean automatically.

        Returns (lat, ax, along) in the GRIPPER's own frame, or None:
          lat   — left/right within the jaw gap  (the "is it centred" number)
          ax    — across the jaw faces
          along — down the finger axis (depth into the gap)
        """
        box_p = self._box_world_pose()
        if box_p is None:
            return None
        try:
            tf = self._tf_buf.lookup_transform('odom', TCP_LINK, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'[centering] TF odom→{TCP_LINK} unavailable: {e}')
            return None

        t, q = tf.transform.translation, tf.transform.rotation
        d = np.array([box_p.x - t.x, box_p.y - t.y, box_p.z - t.z])

        # Rotate the world-frame offset into the gripper frame (R^T · d) so the
        # components mean something mechanical instead of depending on robot yaw.
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
            [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
        ])
        local = R.T @ d

        # Gripper-frame axes, verified against gripper.xacro (2026-07-28, register B6):
        #   x — the JAW-OPENING direction. Finger pivots sit at x = ∓0.012 and
        #       gripper_tcp at x = 0, so the tool frame IS centred between the jaws.
        #       This component answers "is the box gripped centrally?".
        #   y — along the fingers (the approach axis; points down in a top-down grasp).
        #   z — the hinge axis: the box slides freely along the flat finger faces in
        #       this direction, so an offset here means the box is near a face edge
        #       rather than badly gripped.
        # These were previously mislabelled (z was reported as "lateral"), which made
        # a well-centred grasp look 23 mm off.
        jaw, along, slide = float(local[0]), float(local[1]), float(local[2])
        FACE_HALF = 0.018   # finger face is 36 mm wide (gripper.xacro collision box)
        self.get_logger().info(
            f'[centering] box vs tool centre (gripper frame): '
            f'between-jaws={jaw * 1000:+.1f} mm  '
            f'along-faces={slide * 1000:+.1f} mm  '
            f'depth-into-gap={along * 1000:+.1f} mm  → '
            f'{"CENTRED" if abs(jaw) < 0.005 else "OFF-CENTRE"} between jaws; '
            f'{"within" if abs(slide) < FACE_HALF else "PAST THE EDGE OF"} the finger faces')
        self._last_centering = (jaw, slide, along)
        return jaw, slide, along

    def _box_world_pose(self):
        """Full box position in the Gazebo world frame (sim ground truth), or None."""
        if not self._get_state.service_is_ready():
            return None
        names = self._candidate_box_names()
        for name in names:
            req = GetEntityState.Request()
            req.name = name
            req.reference_frame = 'world'
            fut = self._get_state.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
            if fut.done() and fut.result() is not None and fut.result().success:
                return fut.result().state.pose.position
        return None

    def _candidate_box_names(self):
        """Which Gazebo models the ground-truth helpers should query, best guess first.

        Order matters — the helpers return the FIRST name that resolves:
          1. the box we are currently carrying, if known. _claim_held_box() removes it
             from _remaining_boxes, so without this entry the carried box is invisible
             to every ground-truth query the moment it is claimed.
          2. the boxes still waiting on the pickup table.
          3. a hardcoded fallback, for paths that never populate either list
             (calibration loops, isolated grasp tests).
        """
        names = []
        held = getattr(self, '_held_box_name', None)
        if held:
            names.append(held)
        # The box _verify_grasp_truth() saw rise, i.e. the one actually in the
        # gripper. Only known AFTER a lift, so it does not help the lift check itself
        # -- that measures every box (see _all_box_world_z) -- but it keeps the
        # centering measurement and the place step pointed at the right object.
        target = getattr(self, '_target_box_name', None)
        if target and target not in names:
            names.append(target)
        names += [n for n in (getattr(self, '_remaining_boxes', None) or [])
                  if n not in names]
        if not names:
            names = ['stack_box_0', 'stack_box_1', 'stack_box_2',
                     'target_box', 'grasp_test_box']
        return names

    def _box_is_held(self, max_dist=0.06):
        """Is the box STILL in the hand right now? Ground-truth distance from the box
        to gripper_tcp.

        Needed because _weld_active is a latched flag. Under the legacy weld that was
        acceptable — the teleport could not fail, so "we welded it" implied "we still
        have it". A real physics joint CAN break (contacts lost on a hard turn, the box
        knocked out on the table edge), and a latched flag would happily report
        transport_retained=True for a box lying on the floor 3 m back. Re-measuring at
        arrival is the honest number, and drop-during-transport is a headline result of
        the thesis, so it must not be faked.

        Threshold: since gripper_tcp was calibrated to the real grasp point
        (2026-08-01), a correctly held box now reads ~0.00 m from the TCP rather than
        the old ~0.06 m, so the threshold could be tightened from 0.12 to 0.06 and still
        leave 6 cm of margin. A dropped box (on the table or floor, arm in the travel
        pose) is several times further, so the separation stays unambiguous. The measured
        distance is logged every call so this can be replaced by a calibrated value from
        N runs rather than staying a guess.

        Returns True/False, or None if ground truth is unavailable (caller decides).
        """
        box_p = self._box_world_pose()
        if box_p is None:
            return None
        try:
            tf = self._tf_buf.lookup_transform('odom', TCP_LINK, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'[held] TF odom→{TCP_LINK} unavailable: {e}')
            return None
        t = tf.transform.translation
        dist = float(np.linalg.norm(
            [box_p.x - t.x, box_p.y - t.y, box_p.z - t.z]))
        held = dist <= max_dist
        self.get_logger().info(
            f'[held] box is {dist:.3f} m from {TCP_LINK} (limit {max_dist:.3f}) → '
            f'{"STILL HELD" if held else "NOT HELD — box was lost"}')
        return held

    def _check_transport_retained(self):
        """Did the box survive the drive to the place table? Sampled at ARRIVAL.

        Also re-syncs _weld_active, so a box lost in transit correctly skips the place
        step instead of the arm solemnly putting down nothing. Falls back to the latched
        flag if ground truth is unavailable (real hardware), which is the old behaviour.
        """
        if self._physics_grasp:
            held = self._box_is_held()
            if held is not None:
                if self._weld_active and not held:
                    self.get_logger().error(
                        '[transport] box was LOST in transit — the physics joint broke '
                        'somewhere between the pickup table and here')
                self._weld_active = bool(held)
                return bool(held)
            self.get_logger().warn(
                '[transport] ground truth unavailable — reporting the latched grasp flag')
        return self._weld_active

    def _verify_grasp_truth(self, z_before, min_rise=0.02):
        """Did the box actually leave the table? Compares true world Z before the
        close against after the lift.

        In theory the lift raises the TCP by HOVER - GRASP_Z = 0.06 m, so a held
        box should rise 0.06 m. MEASURED 2026-07-28: it rose only **0.031 m** on a
        good grasp — barely over the old 0.030 threshold, a 1 mm margin. The
        shortfall is because the lift is planned in JOINT space (RRTConnect), so
        the tool does not travel straight up; it arcs and rotates, and a box held
        at a fixed offset in the gripper frame follows that arc rather than rising
        the full commanded amount. Threshold lowered to 0.02 m so a genuine grasp
        is not rejected by that shortfall. A box left on the table moves 0.000 m,
        so the separation is still unambiguous.

        Returns True / False, or None if Gazebo state is unavailable (then the
        caller falls back to the vision check).
        """
        if not z_before:
            return None
        z_after = self._all_box_world_z()
        if not z_after:
            return None

        # Which box rose the most? That is the one in the gripper.
        risen = {n: z_after[n] - z_before[n] for n in z_after if n in z_before}
        if not risen:
            return None
        name = max(risen, key=risen.get)
        rise = risen[name]
        ok = rise >= min_rise

        others = ', '.join(f'{n}{risen[n]:+.3f}' for n in sorted(risen) if n != name)
        self.get_logger().info(
            f'[verify] GROUND TRUTH: {name} z {z_before[name]:.3f} → {z_after[name]:.3f} '
            f'(rise {rise:+.3f} m, need ≥{min_rise:.3f}) → '
            f'{"LIFTED (grasped)" if ok else "DID NOT MOVE (failed)"}'
            + (f'  [others: {others}]' if others else ''))

        # Latch the winner so the centering measurement and the place step follow the
        # box we are actually carrying.
        if ok:
            self._target_box_name = name
        return ok

    def _verify_grasp_vision(self, px, py, pz, tol=0.05, fresh_age=0.3, timeout=4.0):
        """Ground-truth grasp check (Phase 0): after the lift, wait for a FRESH
        /box_pose detection (age < fresh_age — a real sighting just now, not
        box_pose_estimator's 20 Hz replay of an old latch) and see if a box is
        still at the ORIGINAL 3D grasp point.

        MUST be 3D. The lift moves straight up, so a correctly-grasped box keeps
        the same (x, y) and only its Z changes. An XY-only comparison reported
        every good grasp as 'box still on the table', which released the weld and
        dropped the box — the repeating lift-a-bit-then-drop loop (fixed 2026-07-28).

        Returns:
          True  — grasp succeeded (nothing left at the original grasp point)
          False — grasp failed (a box is still sitting there)
          None  — inconclusive (no fresh detection within timeout — e.g. the arm
                  is occluding the camera's view of the table)
        """
        deadline = time.time() + timeout
        saw_fresh = False
        hit = False
        dist = None
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._box_age <= fresh_age:
                saw_fresh = True
                reading = self._read_box_live_xyz()
                if reading is not None:
                    x, y, z = reading
                    dist = math.sqrt((x - px) ** 2 + (y - py) ** 2 + (z - pz) ** 2)
                    if dist <= tol:
                        hit = True
                break   # one fresh look is enough — no need to wait out the timeout
            time.sleep(0.1)

        if not saw_fresh:
            self.get_logger().warn(
                f'[verify] no fresh /box_pose detection within {timeout}s '
                f'(last age={self._box_age:.2f}s) — grasp outcome INCONCLUSIVE, '
                f'not assumed successful')
            return None

        d_str = f'{dist:.3f} m' if dist is not None else 'no box detected'
        self.get_logger().info(
            f'[verify] fresh look at grasp point ({px:.3f},{py:.3f},{pz:.3f}): '
            f'nearest box {d_str} (tol {tol:.3f}) → '
            f'{"STILL THERE (failed)" if hit else "GONE (grasped)"}')
        return not hit

    def _drive_forward(self, drive, d_start, timeout=25.0):
        """Drive straight forward `drive` metres, measured by ODOMETRY, then hard stop.

        Odometry rather than the camera, deliberately: a camera-feedback stop only fires
        when forward AND lateral error settle at the same instant, which often never
        happened — the robot crept into the table and climbed it. Distance driven is pure
        geometry and stops every time.

        STOP_MARGIN: the pickup dock's STOP_DISTANCE already leaves only ~1 mm of bumper
        clearance above load_scene()'s own hard-coded minimum (verified 2026-08-13, same
        razor-thin-margin pattern already found and fixed for the place table's
        dock_range). The velocity law below never drops the commanded speed below
        0.06 m/s, even in the final millimetre, and one more control cycle passes
        between the stop decision and Twist() actually taking effect -- real overshoot,
        not just a tight target. Stopping a controlled 1 cm short of the literal target
        absorbs that overshoot; the arm's reach was measured usable up to 0.24-0.25 m,
        so 1 cm of slack here costs nothing.
        """
        STOP_MARGIN = 0.005
        drive = max(0.0, drive - STOP_MARGIN)
        if drive <= 0.001:
            return 0.0
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._odom is not None:
                break
        ox, oy = self._odom if self._odom is not None else (0.0, 0.0)

        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            disp = self._odom_dist_since(ox, oy)
            disp = 0.0 if disp is None else disp
            remaining = drive - disp
            self.get_logger().info(
                f'approach: driven {disp:.3f}/{drive:.3f} m  (box ≈ {d_start - disp:.3f} m)',
                throttle_duration_sec=0.5)
            if remaining <= 0.0:
                self.get_logger().info(f'stop — {d_start - drive:.3f} m from the box')
                break
            t = Twist()
            t.linear.x = max(0.06, min(0.14, 0.6 * remaining))   # ease off near the stop
            self._cmd_vel_pub.publish(t)
            time.sleep(0.05)
        else:
            self.get_logger().warn('approach timed out — stopping')

        self._cmd_vel_pub.publish(Twist())
        time.sleep(0.5)
        # Return what was ACTUALLY driven, not what was asked for. The two differ by
        # STOP_MARGIN, by any timeout, and by odometry error. The caller uses this to
        # aim the arm at where the box now is, instead of assuming it reached the
        # commanded stop distance -- see the grasp-target derivation in visual_docking().
        final = self._odom_dist_since(ox, oy)
        return 0.0 if final is None else float(final)

    def visual_docking(self):
        """MECHANISM 1 — land, align, then approach (differential drive).

        Because the base is now diff-drive (can rotate in place), we do this in
        two clean phases instead of the old Ackermann steer-while-rolling:
          Phase A  rotate in place until the box is centred straight ahead (|y|→0)
                   — i.e. the robot FACES the box squarely.
          Phase B  drive straight in to the grasp distance, with small rotation
                   trims to stay centred.
        Entering Phase B already aligned means the box ends up at (~TARGET_X, ~0)
        in base_link — square and centred — which is what makes the top-down grasp
        reliable.
        """
        self.get_logger().info('=== Step 1.5: Land → align → approach (diff) ===')
        # Hand /cmd_vel over from Nav2 to us, or the dock crawls/stalls.
        self._silence_nav2_cmdvel()

        TARGET_X = 0.28     # final grasp distance from base_link
        X_TOL    = 0.02     # forward tolerance
        Y_TOL    = 0.03     # lateral-centre tolerance
        K_ROT    = 1.8      # rotation gain  (rad/s per m of lateral error)
        K_FWD    = 0.8      # forward gain   (m/s per m of distance error)
        MAX_ROT  = 0.5
        # Minimum turn rate while OUTSIDE tolerance. Pure proportional control
        # slows down as it converges: at the 0.03 m tolerance the gain alone
        # commands 1.8*0.03 = 0.054 rad/s (3 deg/s), so the last few degrees
        # crawl and often time out. Floor it so the robot turns decisively and
        # then stops dead once centred.
        MIN_ROT  = 0.25
        MIN_FWD, MAX_FWD = 0.06, 0.16

        def clamp(v, lo, hi):
            return max(lo, min(hi, v))

        # ── Phase A: rotate in place to centre the box ──────────────────────
        self.get_logger().info('Phase A: rotating to face the box…')
        ok = 0
        deadline = time.time() + 15.0
        while time.time() < deadline:
            b = self._read_box_live()
            if b is None:
                self._cmd_vel_pub.publish(Twist())   # stop; never coast on a lost box
                self.get_logger().info('waiting for /box_pose…', throttle_duration_sec=1.0)
                time.sleep(0.1); continue
            x, y = b
            self.get_logger().info(f'align: lateral={y:+.3f} (x={x:.3f})',
                                   throttle_duration_sec=0.5)
            if abs(y) < Y_TOL:
                ok += 1
                if ok >= 3:
                    self.get_logger().info('aligned — box centred ahead'); break
            else:
                ok = 0
            w = clamp(K_ROT * y, -MAX_ROT, MAX_ROT)
            if abs(w) < MIN_ROT:            # never crawl; see MIN_ROT
                w = MIN_ROT if w >= 0 else -MIN_ROT
            t = Twist(); t.angular.z = w
            self._cmd_vel_pub.publish(t); time.sleep(0.05)
        self._cmd_vel_pub.publish(Twist()); time.sleep(0.3)

        # ── Phase B: distance-measured HARD STOP (reliable, can't climb the table)
        # The old camera-feedback stop was fragile: it only stopped when forward AND
        # lateral error settled *at the same time*, so it often never fired and the
        # robot crept into the table and climbed it. Instead: take ONE solid box
        # reading while the camera is still accurate (robot is ~0.6 m back, far beyond
        # the near-clip), then drive a MEASURED distance by ODOMETRY and STOP HARD at
        # STOP_DISTANCE. This is pure geometry — it stops in front of the table every
        # time, regardless of the camera at close range.
        # STOP_DISTANCE = module constant (0.24 m). Lower = closer/more reachable but risks climbing
        # the table; higher = safer but arm can't reach. 0.24 ≈ front ~3-4 cm from table.

        box0 = self.get_box_xyz(samples=6, timeout=8.0)
        if box0 is not None:
            d0 = box0[0]
            # After driving straight to STOP_DISTANCE, the box sits at base_link
            # x≈STOP_DISTANCE (TRUE), with the accurate far-range y/z unchanged.
            self._dock_box = (self._stop_distance, box0[1], box0[2])
            # Derive pickup table top height from the box centre (box top − half-height).
            # This eliminates the need to hand-tune stack_surface_z.
            self._table_top_base_z = box0[2] - BOX_HALF_H
            self.get_logger().info(
                f'table_top_base_z derived from perception: {self._table_top_base_z:.4f} m '
                f'(box_centre_z={box0[2]:.4f})')
        else:
            b = self._read_box_live()
            d0 = b[0] if b else FIXED_BOX[0]
            self._dock_box = None
            self.get_logger().warn(f'Phase B: no median reading — using d0={d0:.3f} m')

        # ── TWO-STAGE APPROACH (2026-08-02) ─────────────────────────────────
        # STAGE 1: drive to FINAL_READ_DIST, where the box is still fully inside the
        #          camera's vertical field of view.
        # STAGE 2: take the FINAL reading there — the most accurate one available, from
        #          the closest distance the camera can be trusted.
        # STAGE 3: drive the last few centimetres blind to STOP_DISTANCE, the arm's reach.
        #
        # The old code took ONE reading at ~0.60 m and drove ~0.36 m blind, so every bit of
        # odometry drift over that whole distance landed in the grasp target. Splitting it
        # cuts the blind stretch to ~0.03 m.
        stage1 = max(0.0, d0 - FINAL_READ_DIST)
        self.get_logger().info(
            f'Phase B stage 1: box at {d0:.3f} m → driving {stage1:.3f} m to the final '
            f'reading point at {FINAL_READ_DIST:.2f} m')
        self._drive_forward(stage1, d0)

        # ── STAGE 2: the final, best reading ────────────────────────────────
        final = self.get_box_xyz(samples=6, timeout=6.0)
        if final is not None:
            self._dock_box = (self._stop_distance, final[1], final[2])
            self._table_top_base_z = final[2] - BOX_HALF_H
            self.get_logger().info(
                f'FINAL READING at {FINAL_READ_DIST:.2f} m: box at '
                f'({final[0]:.4f}, {final[1]:+.4f}, {final[2]:.4f}) — this is the closest '
                f'trustworthy measurement, and it sets the grasp target.')
        else:
            self.get_logger().warn(
                'no final reading at the close point — keeping the earlier estimate. '
                'The box may already be out of the camera frame; check the table height.')

        # ── STAGE 3: the last short blind hop ───────────────────────────────
        stage2 = max(0.0, FINAL_READ_DIST - self._stop_distance)
        self.get_logger().info(
            f'Phase B stage 3: driving the last {stage2:.3f} m blind to '
            f'{self._stop_distance:.2f} m (the arm reach limit)')
        driven = self._drive_forward(stage2, FINAL_READ_DIST)

        # F3: aim at where the box ACTUALLY is, not where the dock was told to stop.
        #
        # _dock_box's x was previously hard-set to self._stop_distance, i.e. the code
        # ASSUMED the blind hop landed exactly on its commanded distance. It does not:
        # _drive_forward subtracts STOP_MARGIN (5 mm) from every commanded drive, and
        # odometry adds its own error on top. Ground-truth pin poses across runs put
        # the box at 0.229-0.239 m while the arm was aimed at a fixed 0.240 -- so the
        # target was wrong by up to ~11 mm, in a workspace band only ~40 mm wide, and
        # wrong in a DIFFERENT direction each run. That is enough on its own to explain
        # grasps landing 13-21 mm off-centre along the finger faces.
        #
        # Using the measured travel makes the aim track the dock instead of assuming
        # it, which also makes STOP_MARGIN harmless rather than a systematic bias.
        measured_x = FINAL_READ_DIST - driven
        if self._dock_box is not None:
            self.get_logger().info(
                f'grasp x from measured dock: {measured_x:.4f} m '
                f'(drove {driven:.4f} of {stage2:.4f} commanded; '
                f'assumed value would have been {self._stop_distance:.4f})')
            self._dock_box = (measured_x, self._dock_box[1], self._dock_box[2])

        # ── Lateral re-fix AFTER the drive (added 2026-07-28) ─────────────────
        # WHY: _dock_box was frozen BEFORE this drive, so its y came from a
        # measurement taken ~1 m away, expressed in the base_link frame as it was
        # back then. The robot then drove ~0.76 m forward. Any residual yaw error
        # during that drive rotates the frame and slides the target sideways —
        # Phase A's rotate-to-align is known to oscillate ±5-6° and time out
        # without converging, and at 0.24 m a 5° residual is ~2 cm of lateral
        # error, consistently to one side. That is the observed "box ends up a bit
        # to the right, but still between the fingers" bias.
        #
        # THE FIX: box_pose_estimator latches each detection as a point in the MAP
        # frame and republishes it at 20 Hz transformed into the CURRENT base_link.
        # So reading /box_pose now yields the same physical world point expressed
        # in the frame the arm will actually plan in — the drive and any yaw change
        # are already accounted for. No new detection is required, which matters on
        # real hardware where the camera is blind closer than 0.30 m.
        #
        # x is deliberately NOT taken from perception: close-range depth is biased
        # (~0.27 m read vs 0.18 m true) and trusting it pushes the goal past the
        # arm's reach and fails IK. Range stays odometry-derived (STOP_DISTANCE);
        # only the lateral/height components are refreshed.
        # Y ONLY — deliberately NOT z. At close range the base-mounted camera looks
        # down onto the box and sees mostly its TOP face, so the HSV blob centroid
        # rides up toward the top surface and z reads high (confirmed by watching
        # /box_detection_debug at the dock). The estimator re-latches on every
        # successful detection, so that biased value overwrites the good far-range
        # one. Height therefore stays with the FAR measurement, where the front face
        # is visible and the vertical centroid is much closer to the true centre.
        # Lateral y is unaffected by which face dominates — the top and front faces
        # share the same left-right centre — so y is safe to refresh, and y is
        # exactly the component the drive's yaw error corrupts.
        # SUPERSEDED 2026-08-02 — kept behind a flag, not deleted.
        # This whole re-fix existed to patch up a target that had been frozen ~0.76 m from
        # the box. With the two-stage approach the target is now set from a reading taken at
        # FINAL_READ_DIST (0.27 m), only ~3 cm before the dock, so there is almost no drive
        # left for yaw error to corrupt.
        #
        # Worse, re-running it here would now do HARM. With the 0.14 m table the box centre
        # leaves the camera's vertical field of view below 0.254 m — and this code runs at
        # 0.24 m. Any reading it gets is of a box that is half out of frame, and it would
        # overwrite the good 0.27 m measurement with a worse one.
        USE_LEGACY_LATERAL_REFIX = False
        if USE_LEGACY_LATERAL_REFIX and self._dock_box is not None:
            fresh = self._read_box_live_xyz()
            if fresh is not None:
                y_old, z_keep = self._dock_box[1], self._dock_box[2]
                y_new = fresh[1]
                self._dock_box = (self._stop_distance, y_new, z_keep)
                self.get_logger().info(
                    f'[dock] lateral re-fix after drive: y {y_old:+.4f} → {y_new:+.4f} '
                    f'(Δ{y_new - y_old:+.4f} m)')
            else:
                self.get_logger().warn(
                    '[dock] no /box_pose for the lateral re-fix — keeping the '
                    'pre-drive estimate (expect a small sideways bias)')

    # ── Step 2: Pin base ──────────────────────────────────────────────────────

    def pin_base(self):
        """Capture current robot world pose and hold it at 50 Hz."""
        self.get_logger().info('=== Step 2: Pinning base ===')
        if not self._base_pin_enabled:
            self.get_logger().info('base_pin_enabled=false — skipping pin')
            return False
        if not self._get_state.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn('/get_entity_state unavailable — base will not be pinned')
            return False
        req = GetEntityState.Request()
        req.name = PIN_ROBOT_NAME
        req.reference_frame = 'world'
        fut = self._get_state.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        res = fut.result()
        if res is None or not res.success:
            self.get_logger().warn('Could not read robot pose — continuing without pin')
            return False
        self._pin_pose = res.state.pose
        p = self._pin_pose.position
        self.get_logger().info(f'pinning base at world ({p.x:.3f}, {p.y:.3f})')
        self._pin_timer = self.create_timer(0.02, self._hold_base)   # 50 Hz
        return True

    def unpin_base(self):
        """Cancel the base-pin timer so cmd_vel can drive freely."""
        if self._pin_timer is not None:
            self._pin_timer.cancel()
            self._pin_timer = None
            self.get_logger().info('base pin released')

    def _hold_base(self):
        if self._pin_pose is None:
            return
        st = EntityState()
        st.name = PIN_ROBOT_NAME
        st.pose = self._pin_pose
        st.reference_frame = 'world'
        req = SetEntityState.Request()
        req.state = st
        self._set_state.call_async(req)

    # ── Step 3: Perceive box ──────────────────────────────────────────────────

    def get_box_xyz(self, samples=8, timeout=20.0):
        self.get_logger().info('=== Step 3: Perceiving box ===')
        readings = []
        deadline = time.time() + timeout
        while time.time() < deadline and len(readings) < samples:
            self._latest_box = None
            t0 = time.time()
            while self._latest_box is None and time.time() < t0 + 1.0:
                rclpy.spin_once(self, timeout_sec=0.1)
            p = self._latest_box
            if p is not None and p.header.frame_id == PLANNING_FRAME:
                readings.append((p.pose.position.x,
                                 p.pose.position.y,
                                 p.pose.position.z))
        if len(readings) < 3:
            self.get_logger().warn(
                'Not enough /box_pose readings — using FIXED_BOX fallback')
            return None
        box = (statistics.median(r[0] for r in readings),
               statistics.median(r[1] for r in readings),
               statistics.median(r[2] for r in readings))
        self.get_logger().info(
            f'box @ base_link {tuple(round(v, 3) for v in box)} '
            f'(median of {len(readings)} readings)')
        return box

    # ── Gripper helpers ───────────────────────────────────────────────────────

    def _weld_cb(self, msg: Bool):
        """/grasp_attach echo — only meaningful in legacy weld mode.

        Under physics grasp, _weld_active is set from GROUND TRUTH after the lift
        (see grasp_and_retract), not from a topic. Ignoring the topic here also
        makes the node immune to an orphaned grasp_attacher/smart_grasp left over
        from an earlier run, which would otherwise flip the flag True and fake a
        successful pick.
        """
        if getattr(self, '_physics_grasp', False):
            return
        self._weld_active = msg.data

    def _gripper_joint_cb(self, msg: JointState):
        try:
            idx = msg.name.index('gripper_controller')
            self._gripper_actual = msg.position[idx]
        except ValueError:
            pass
        # Also latch the ARM joints. solve_ik_seeded() needs the arm's ACTUAL
        # configuration as the IK seed: seeding from the real current pose is what
        # keeps the returned solution on the SAME branch the arm is already in.
        try:
            self._arm_actual = [msg.position[msg.name.index(j)] for j in ARM_JOINTS]
        except ValueError:
            pass

    def _bumper_cb(self, msg: Bool):
        self._bumper_contact = msg.data

    def set_gripper(self, values, label, duration=0.8, steps=20):
        start = self._last_gripper
        for i in range(1, steps + 1):
            a = i / steps
            interp = [s + a * (v - s) for s, v in zip(start, values)]
            msg = Float64MultiArray()
            msg.data = [float(x) for x in interp]
            self._gripper.publish(msg)
            rclpy.spin_once(self, timeout_sec=duration / steps)
        self._last_gripper = list(values)
        self.get_logger().info(f'gripper → {label}')

        # Under physics grasp, opening the fingers past the plugin's release_position
        # IS the release — there is no "detach" call to make. Clearing the flag here
        # catches every release path in this file (place, retry, abort, error) without
        # having to touch each call site. RELEASE_POSITION mirrors <release_position>
        # in gazebo/mycobot_ros2_control.xacro; keep the two in sync.
        RELEASE_POSITION = 0.0
        if self._physics_grasp and self._weld_active and values[0] > RELEASE_POSITION:
            self._weld_active = False
            self.get_logger().info(
                f'[grasp] fingers opened to {values[0]:+.3f} > release_position '
                f'{RELEASE_POSITION:+.3f} — physics joint released')

        time.sleep(0.3)

    def _spin_for(self, seconds: float):
        """Spin callbacks for a REAL wall-clock duration.

        rclpy.spin_once(timeout_sec=X) is NOT a sleep — it returns as soon as it
        executes one callback, and only waits up to X when the queue is empty.
        With /joint_states at 50 Hz there is always work pending, so it returns
        essentially instantly.

        Using it as a sleep made close_until_contact() dump the gripper's entire
        closing trajectory in ~65 ms instead of the intended 5.6 s (70 steps ×
        0.08 s). ODE never had time to develop finger contact, the joint could
        not follow the command, and stall detection became a race: the same box
        in the same place would grasp on one attempt and report AIR on the next.
        Observed 2026-08-03 in /tmp/run_191027.log — attempt 1 swept to the close
        limit in 65 ms and reported AIR, attempt 2 stalled correctly and grasped.

        This spins for the full duration while still servicing callbacks, so
        _gripper_actual stays fresh.
        """
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=max(0.001, end - time.time()))

    def close_until_contact(self):
        """Step-close the gripper and weld. Success is decided by VISION
        (_verify_grasp_vision, Phase 0) — no longer by an aperture threshold.

        WHY THE CLASSIFIER WAS REMOVED (2026-07-28): the stall angle used to be
        compared against BOX_CONTACT_ANGLE ± APERTURE_TOL, and anything outside
        that narrow band was discarded as OBSTRUCTED/AIR — which re-opened the
        gripper and re-descended. But BOX_CONTACT_ANGLE (-0.11) was never
        calibrated (its own comment said "placeholder; run
        calibrate_contact_angle.py"), so a physically fine grasp was being
        thrown away whenever the stall missed a made-up ±0.025 rad window.
        Observed symptom: close → open → close → open, exactly
        MAX_GRASP_RETRIES+1 times, then abort.

        The close still STOPS early on a genuine stall — that is physically
        correct (stop squeezing once blocked) and avoids over-compression — it
        just no longer decides whether the grasp counted. The stall angle is
        recorded in self._last_stall_angle as diagnostic data so real
        calibration numbers accumulate in the CSV for the thesis.

        Returns GraspResult.GRASPED once the close completes and the weld is
        fired; the real verdict comes from the post-lift vision check.
        """
        self._weld_active = False
        cmd     = self._last_gripper[0]
        # Close floor: how far the fingers are allowed to travel if nothing ever
        # reports contact. Live-tunable — see the declare_parameter comment above.
        floor   = float(self.get_parameter('grasp_close_floor').value)
        lag_run = 0
        stall_angle = None

        # Capture actual BEFORE the loop — needed to decide which path we're on.
        rclpy.spin_once(self, timeout_sec=0.05)
        start_actual    = self._gripper_actual
        gripper_tracking = False              # True once joint moves ≥ TRACKING_RAD
        TRACKING_RAD     = 3 * CLOSE_STEP_RAD  # 0.015 rad

        while cmd > floor:
            cmd = max(floor, cmd - CLOSE_STEP_RAD)
            t   = (GRIPPER_OPEN[0] - cmd) / (GRIPPER_OPEN[0] - GRIPPER_GRASP[0])
            msg = Float64MultiArray()
            msg.data = [float(o + t * (c - o))
                        for o, c in zip(GRIPPER_OPEN, GRIPPER_GRASP)]
            self._gripper.publish(msg)
            self._last_gripper = msg.data[:]

            self._spin_for(CLOSE_STEP_SEC)

            # smart_grasp (bumper or joint-error) fired the weld
            if self._weld_active:
                stall_angle = self._gripper_actual
                self.get_logger().info(
                    f'[close] smart_grasp weld  actual={stall_angle:.3f}')
                break

            actual = self._gripper_actual

            # Motion must be in the CLOSING direction (decreasing angle). abs()
            # also accepted the tail of the preceding OPEN move, so "tracking"
            # could be confirmed by the gripper still finishing its opening
            # sweep — after which the lag test compares against a joint moving
            # the wrong way.
            if not gripper_tracking and (start_actual - actual) >= TRACKING_RAD:
                gripper_tracking = True
                self.get_logger().info(
                    f'[close] joint tracking confirmed  actual={actual:.3f}')

            if gripper_tracking:
                # PATH A — aperture detection
                lag = actual - cmd
                if lag > LAG_THRESH:
                    lag_run += 1
                    if lag_run >= STALL_STEPS:
                        stall_angle = actual
                        self.get_logger().info(
                            f'[close] aperture stall  actual={stall_angle:.3f} '
                            f'cmd={cmd:.3f}  lag={lag:.3f}')
                        break
                else:
                    lag_run = 0
            else:
                # PATH B — joint frozen; cmd-based trigger at BOX_CONTACT_ANGLE
                if cmd <= BOX_CONTACT_ANGLE:
                    stall_angle = BOX_CONTACT_ANGLE
                    self.get_logger().info(
                        f'[close] cmd-reach weld  cmd={cmd:.3f} '
                        f'actual={actual:.3f} (joint not tracking — sim fallback)')
                    break
        else:
            self.get_logger().warn('[close] swept to -0.20 rad — no stall detected')

        # Diagnostic only — recorded for calibration, NOT used to gate the grasp.
        self._last_stall_angle = stall_angle
        would_have_been = self._classify_stall(stall_angle)
        self.get_logger().info(
            f'[close] stall={stall_angle}  bumper={self._bumper_contact}  '
            f'(old calibrated-band classifier would have said {would_have_been} — '
            f'ignored; that band was never calibrated)')

        # AIR check — the ONE part of the old classifier worth keeping.
        # This is not a calibrated threshold, it is a binary physical fact: if the
        # gripper swept all the way to its hard close limit and NOTHING ever
        # resisted, there is nothing between the fingers. Welding here fakes a
        # successful grasp — observed 2026-07-28, attempt 3: swept to -0.20 with no
        # stall, weld grabbed the box from 0.079 m away, and vision then reported
        # "GONE (grasped)" because the welded box dutifully followed the gripper.
        if stall_angle is None:
            self.get_logger().warn(
                '[close] swept to the close limit with NO resistance at any point — '
                'nothing is between the fingers. Refusing to weld; reporting AIR.')
            time.sleep(0.3)
            return GraspResult.AIR

        result = GraspResult.GRASPED
        # Legacy weld only: fire the teleport now that the fingers are closed. Under
        # physics grasp the joint was already created by the finger contacts during
        # the close loop above — there is nothing to trigger here.
        if not self._weld_active and USE_WELD and not self._physics_grasp:
            self.attach(True)

        # Back off 1 step to remove contact force — prevents ODE "explosion"
        if self._weld_active or result == GraspResult.GRASPED:
            j_back = self._last_gripper[0] + CLOSE_STEP_RAD
            t_back = max(0.0, (GRIPPER_OPEN[0] - j_back) /
                              (GRIPPER_OPEN[0] - GRIPPER_GRASP[0]))
            back = Float64MultiArray()
            back.data = [float(o + t_back * (c - o))
                         for o, c in zip(GRIPPER_OPEN, GRIPPER_GRASP)]
            self._gripper.publish(back)
            self._last_gripper = back.data[:]
            rclpy.spin_once(self, timeout_sec=0.15)
            self.get_logger().info('[close] fingers backed off — contact force removed')

        time.sleep(0.3)
        return result

    def _classify_stall(self, stall_angle):
        """Classify a stall angle against the calibrated contact band."""
        if stall_angle is None:
            return GraspResult.AIR
        delta = stall_angle - BOX_CONTACT_ANGLE
        if abs(delta) <= APERTURE_TOL:
            return GraspResult.GRASPED
        if delta > 0:          # stalled wider/earlier than expected
            return GraspResult.OBSTRUCTED
        return GraspResult.AIR  # stalled narrower than expected

    def attach(self, on: bool):
        """Command the LEGACY kinematic weld on/off.

        Under physics grasp this is a deliberate NO-OP. The plugin owns the joint:
        it is created by finger contact and destroyed when gripper_controller opens
        past release_position — neither is commandable over a topic. Every
        `attach(False)` call site in this file is already followed by
        `set_gripper(GRIPPER_OPEN, ...)`, which IS the release under physics, so the
        call sites stay correct in both modes and did not need rewriting.

        Publishing anyway would be worse than useless: if a stale grasp_attacher from
        a previous run is still alive (a documented failure mode — see kill_sim.sh),
        the message would restart the teleport and it would fight the joint.
        """
        if self._physics_grasp:
            self.get_logger().debug(
                f'attach({on}) ignored — physics grasp: contact makes the joint, '
                f'opening the fingers breaks it')
            return
        msg = Bool()
        msg.data = bool(on)
        self._attach_pub.publish(msg)
        self.get_logger().info(f'/grasp_attach → {on}')
        end = time.time() + 0.7
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    # ── MoveIt helpers ────────────────────────────────────────────────────────

    @staticmethod
    def _joint_constraints(values, tol=0.01):
        c = Constraints()
        for name, val in zip(ARM_JOINTS, values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(val)
            jc.tolerance_above = tol
            jc.tolerance_below = tol
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        return c

    @staticmethod
    def _pose_constraints(x, y, z, quat, pos_tol=0.01, ori_tol=0.1):
        """pos_tol is the RADIUS of the sphere MoveIt must land the TCP inside.

        Was 0.025 (2.5 cm) — larger than the box is wide (3.5 cm). MoveIt could
        legitimately stop 2.5 cm below the commanded grasp point and still report
        SUCCESS, which is enough to drive the fingers into the tabletop or to miss
        the box sideways. Tightened to 0.01 (1 cm) on 2026-07-31.
        Trade-off: a tighter goal is harder to solve, so watch for IK/planning
        failures. If planning starts failing, loosen it rather than accept a miss.
        """
        c = Constraints()
        target = Pose()
        target.position = Point(x=float(x), y=float(y), z=float(z))
        target.orientation.x, target.orientation.y, \
            target.orientation.z, target.orientation.w = (float(q) for q in quat)

        pc = PositionConstraint()
        pc.header.frame_id = PLANNING_FRAME
        pc.link_name = TCP_LINK
        region = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [pos_tol]
        region.primitives.append(sphere)
        region.primitive_poses.append(target)
        pc.constraint_region = region
        pc.weight = 1.0
        c.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.header.frame_id = PLANNING_FRAME
        oc.link_name = TCP_LINK
        oc.orientation = target.orientation
        oc.absolute_x_axis_tolerance = ori_tol
        oc.absolute_y_axis_tolerance = ori_tol
        oc.absolute_z_axis_tolerance = ori_tol
        oc.weight = 1.0
        c.orientation_constraints.append(oc)
        return c

    def _send(self, constraints, label, plan_only=False, plan_time=PLAN_TIME_SEC):
        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = ARM_GROUP
        req.goal_constraints = [constraints]
        req.num_planning_attempts = PLAN_ATTEMPTS
        req.allowed_planning_time = plan_time
        req.max_velocity_scaling_factor = VEL_SCALE
        req.max_acceleration_scaling_factor = ACC_SCALE
        goal.request = req
        goal.planning_options.plan_only = plan_only

        send_fut = self._move.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut, timeout_sec=15.0)
        if not send_fut.done():
            self.get_logger().error(f'[{label}] goal not accepted within 15 s')
            return False
        gh = send_fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error(f'[{label}] goal rejected')
            return False
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=EXEC_TIMEOUT_SEC)
        if not res_fut.done():
            # 2026-08-12: giving up here without cancelling left the goal running
            # server-side while the orchestrator moved on and issued the NEXT command —
            # an orphaned trajectory finishing late while a new one starts. Diagnosed
            # after this exact class of bug (found in real_grasp_test.py the night
            # before) recurred here, aborting a pick outright.
            self.get_logger().error(
                f'[{label}] no result within {EXEC_TIMEOUT_SEC:.0f} s — cancelling')
            cancel_fut = gh.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_fut, timeout_sec=5.0)
            return False
        ok = res_fut.result().result.error_code.val == MOVEIT_SUCCESS
        self.get_logger().info(f'[{label}] {"OK" if ok else "FAILED"}')
        return ok

    def _wait_for_move_group(self, timeout=60.0):
        self.get_logger().info('waiting for move_group…')
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._move.wait_for_server(timeout_sec=2.0):
                self.get_logger().info('move_group connected')
                return True
        return False

    def go_named(self, name):
        return self._send(self._joint_constraints(NAMED_STATES[name]), name)

    def go_pose(self, x, y, z, q, label):
        self.get_logger().info(f'[{label}] planning…')
        return self._send(self._pose_constraints(x, y, z, q), label)

    @staticmethod
    def _retime(traj, scale=VEL_SCALE):
        """Slow a Cartesian trajectory to the same speed every other motion uses.

        GetCartesianPath on Humble has no max_velocity_scaling_factor field, so the
        service hands back a trajectory timed at FULL speed, while every go_pose()
        move runs at VEL_SCALE (0.2). Left alone, the descent onto the box would be
        the fastest motion in the pipeline — the exact opposite of what a delicate
        approach wants. Stretching time by 1/scale leaves the geometry untouched and
        changes only how fast it is traversed.
        """
        k = 1.0 / max(float(scale), 1e-3)
        for pt in traj.joint_trajectory.points:
            t = (pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9) * k
            pt.time_from_start.sec     = int(t)
            pt.time_from_start.nanosec = int(round((t - int(t)) * 1e9))
            pt.velocities    = [v / k for v in pt.velocities]
            pt.accelerations = [a / (k * k) for a in pt.accelerations]
        return traj

    @staticmethod
    def _exec_timeout(traj):
        """Execution timeout scaled to the trajectory's own length.

        A FIXED 90 s was cancelling legitimate moves mid-flight. Measured 2026-08-25:
        a 273-point retract, retimed to ~0.5 s per point, needs ~136 s of motion; the
        timeout fired at 90 s and cancelled it halfway, which then cascaded into a
        rejected retry (the controller was still settling the cancelled goal), two
        failed fallback plans, and a failed homing move. That whole sequence is what
        an observer sees as the arm "moving up and down weirdly" after a place.

        Scaling with the point count means a long path is allowed to finish, while a
        pathological one is caught BEFORE execution by the rad/m gate in
        go_pose_straight() rather than by a timeout during it.
        """
        pts = getattr(getattr(traj, 'joint_trajectory', None), 'points', None)
        n = len(pts) if pts else 0
        return max(EXEC_TIMEOUT_SEC, 2.0 + EXEC_SEC_PER_POINT * n)

    def solve_ik_seeded(self, x, y, z, q, label, timeout=1.0, quiet=False):
        """One IK solve at (x, y, z, q), SEEDED from the arm's actual configuration.

        Returns a joint list, or None. The seed is the point: kdl_kinematics_plugin
        runs a seeded Newton iteration, so starting from where the arm already is
        keeps the answer on the SAME IK branch instead of some other valid one.
        """
        if self._arm_actual is None:
            self.get_logger().warn(f'[{label}] no /joint_states yet — cannot seed IK')
            return None
        if not self._ik.service_is_ready():
            self._ik.wait_for_service(timeout_sec=2.0)
        if not self._ik.service_is_ready():
            self.get_logger().warn(f'[{label}] /compute_ik unavailable')
            return None

        req = GetPositionIK.Request()
        req.ik_request.group_name    = ARM_GROUP
        req.ik_request.ik_link_name  = TCP_LINK
        req.ik_request.avoid_collisions = True
        # ik_request.timeout is a builtin_interfaces/Duration MESSAGE; the `Duration`
        # imported here is rclpy's class, so convert rather than constructing it raw.
        req.ik_request.timeout = Duration(seconds=timeout).to_msg()
        req.ik_request.robot_state.joint_state.name     = list(ARM_JOINTS)
        req.ik_request.robot_state.joint_state.position = list(self._arm_actual)
        # is_diff: this joint_state names only the 6 ARM joints, not the whole robot
        # (gripper, wheels, ...). Without is_diff MoveIt treats the message as a
        # COMPLETE state and every unnamed joint defaults to 0, which is both a wrong
        # collision world and a wrong seed. With is_diff it is applied as a delta on
        # top of the live scene state, which is what "seed from where the arm is"
        # actually requires.
        req.ik_request.robot_state.is_diff = True
        ps = req.ik_request.pose_stamped
        ps.header.frame_id = PLANNING_FRAME
        ps.pose.position = Point(x=float(x), y=float(y), z=float(z))
        (ps.pose.orientation.x, ps.pose.orientation.y,
         ps.pose.orientation.z, ps.pose.orientation.w) = (float(v) for v in q)

        fut = self._ik.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None:
            self.get_logger().warn(f'[{label}] IK service gave no answer')
            return None
        res = fut.result()
        if res.error_code.val != MOVEIT_SUCCESS:
            self.get_logger().warn(
                f'[{label}] IK failed (error_code {res.error_code.val})')
            return None
        try:
            names = list(res.solution.joint_state.name)
            sol = [res.solution.joint_state.position[names.index(j)]
                   for j in ARM_JOINTS]
        except (ValueError, IndexError):
            self.get_logger().warn(f'[{label}] IK answer missing arm joints')
            return None

        if not quiet:
            delta = max(abs(a - b) for a, b in zip(sol, self._arm_actual))
            self.get_logger().info(
                f'[{label}] IK solved, largest joint change from current: {delta:.4f} rad')
        return sol

    # tries=3 since 2026-08-25: with pick_ik (local optimisation from the seed) the
    # answer is seed-consistent by construction, so repeated sampling is nearly
    # redundant. Kept at a small number rather than removed so a solver regression --
    # or a fall back to KDL -- still gets caught by the spread in the log line below.
    def solve_ik_nearest(self, x, y, z, q, label, tries=3):
        """Call IK several times and keep the solution CLOSEST to the current pose.

        Necessary because kdl_kinematics_plugin does not honour the seed once the
        seeded Newton iteration fails to converge -- it falls back to a RANDOM restart
        (kinematics_solver_attempts: 20). Near the workspace boundary that failure is
        the common case, so a single seeded call returns an essentially arbitrary
        branch: measured 2026-08-25, one call came back 1.636 rad from the arm's
        actual configuration despite being seeded with it.

        Since the restarts are random, sampling repeatedly explores the branches and
        the minimum-distance answer is the same-branch one when it exists. This is a
        workaround for the solver, not a fix -- pick_ik/TRAC-IK use local optimisation
        and return a seed-consistent answer directly (see kinematics.yaml).
        """
        best, best_d = None, float('inf')
        seen = []
        for _ in range(tries):
            sol = self.solve_ik_seeded(x, y, z, q, label, quiet=True)
            if sol is None:
                continue
            d = max(abs(a - b) for a, b in zip(sol, self._arm_actual))
            seen.append(d)
            if d < best_d:
                best, best_d = sol, d
        if best is None:
            self.get_logger().warn(f'[{label}] IK found no solution in {tries} tries')
            return None
        self.get_logger().info(
            f'[{label}] IK {len(seen)}/{tries} solved; closest branch {best_d:.4f} rad '
            f'(worst {max(seen):.4f}) — using closest')
        return best

    def go_pose_branch(self, x, y, z, q, label, max_jump=1.0):
        """Move to a pose by solving IK ONCE and driving there as a JOINT goal.

        Why this exists (2026-08-21). The descend/lift sit at ~96-99% of this arm's
        measured max radial reach. computeCartesianPath samples the line and calls IK
        per waypoint, seeded from the previous solution; near the boundary those
        seeded solves keep failing, and kdl_kinematics_plugin then does a RANDOM
        restart (kinematics_solver_attempts: 20). Consecutive waypoints therefore land
        on DIFFERENT IK branches. Every waypoint pose is exactly on the line, so
        `fraction` reports 100% and nothing looks wrong -- but JointTrajectoryController
        interpolates between those waypoints in JOINT space, so the tool sweeps out
        horizontally and back between samples. Measured: 3.240 rad of joint travel for
        a 0.060 m lift (54 rad/m, vs ~5-8 rad/m for a clean move).

        Solving IK ONCE removes the mechanism entirely: one solve, one branch, and the
        controller interpolates between two configurations that are already close
        together. The straight-line guarantee is given up -- but at this reach a
        "straight line" was being executed as a joint-space swing anyway, so the
        guarantee was nominal. Over the short hover->grasp move the deviation from
        straight is second-order and far smaller than the excursions it replaces.

        max_jump rejects a solution that is a branch flip relative to where the arm
        already is; the caller falls back to the Cartesian path in that case.
        """
        sol = self.solve_ik_nearest(x, y, z, q, label)
        if sol is None:
            return None
        delta = max(abs(a - b) for a, b in zip(sol, self._arm_actual))
        if delta > max_jump:
            self.get_logger().warn(
                f'[{label}] IK returned a different branch ({delta:.3f} rad > '
                f'{max_jump} rad) — refusing it')
            return None
        return self._send(self._joint_constraints(sol), label)

    def go_pose_straight(self, x, y, z, q, label, min_fraction=CART_MIN_FRACTION):
        """Move the TCP in a STRAIGHT LINE to (x, y, z), holding orientation `q`.

        go_pose() hands MoveIt a goal CONSTRAINT and lets OMPL/RRTConnect find any
        valid joint path to it. RRTConnect optimises for validity, not for the shape
        the tool traces, so a commanded straight-down descent arrives along an arc —
        measured at roughly half the commanded travel going sideways on a 60 mm
        vertical move. Approaching a 35 mm box that way clips it before the fingers
        are around it. This plans in TASK space instead, so the tool goes where it
        was told to go.

        Falls back to go_pose() — loudly — whenever a full Cartesian path cannot be
        produced. A PARTIAL interpolation is worse than a joint-space plan: it would
        stop the tool at some undefined fraction of the way instead of at the goal.
        """
        if not self.get_parameter('use_cartesian_descent').value:
            return self.go_pose(x, y, z, q, label)

        if not self._cart.service_is_ready():
            self._cart.wait_for_service(timeout_sec=2.0)
        if not self._cart.service_is_ready():
            self.get_logger().warn(
                f'[{label}] /compute_cartesian_path unavailable — joint-space fallback')
            return self.go_pose(x, y, z, q, label)

        target = Pose()
        target.position = Point(x=float(x), y=float(y), z=float(z))
        (target.orientation.x, target.orientation.y,
         target.orientation.z, target.orientation.w) = (float(v) for v in q)

        req = GetCartesianPath.Request()
        req.header.frame_id  = PLANNING_FRAME
        req.header.stamp     = self.get_clock().now().to_msg()
        req.group_name       = ARM_GROUP
        req.link_name        = TCP_LINK
        req.waypoints        = [target]
        req.max_step         = CART_MAX_STEP
        # jump_threshold stays 0 (the RELATIVE, scaling-factor form -- unreliable on
        # short paths because it compares each step against the mean step). The
        # ABSOLUTE per-joint form below is the one that actually guards this failure.
        req.jump_threshold   = 0.0
        # 2026-08-21: this was the whole bug. With every jump threshold at 0, MoveIt
        # does NOT check joint-space continuity between consecutive Cartesian
        # waypoints -- `fraction` only counts waypoints_achieved/waypoints_requested,
        # so it reports 100% for a path that is geometrically straight in CARTESIAN
        # space but discontinuous in JOINT space.
        #
        # That matters here because the hover sits at r = sqrt(0.240^2 + 0.075^2)
        # = 0.2514 m against a measured max radial reach of 0.253 m -- 99.4% of the
        # envelope. In that near-singular band the IK solution manifold is nearly
        # flat, and kdl_kinematics_plugin (kinematics_solver_attempts: 20) reseeds
        # RANDOMLY whenever a seeded solve fails to converge. Consecutive waypoints
        # therefore land on different IK branches (elbow-up vs elbow-down). Each is
        # individually valid, so fraction stays 100%, and JointTrajectoryController
        # then splines between them IN JOINT SPACE -- the end effector traces
        # whatever curve that produces: out and back, repeatedly.
        #
        # joint2_to_joint1 + joint3_to_joint2 form the planar 2R sub-chain in the
        # vertical reach plane, so radial extension is a function of exactly those
        # two -- which is why the symptom is specifically a J2/J3 event, and why it
        # appears on the descend AND the lift (same path, reversed).
        #
        # 0.15 rad (~8.6 deg) between 5 mm waypoints is far above any legitimate
        # step and far below a branch flip. Exceeding it truncates the path, the
        # fraction drops below CART_MIN_FRACTION, and the existing joint-space
        # fallback fires -- converting a SILENT wrong-path into a LOUD, visible one.
        req.revolute_jump_threshold  = 0.15
        req.prismatic_jump_threshold = 0.0   # no prismatic joints on this arm
        req.avoid_collisions = True

        self.get_logger().info(f'[{label}] planning STRAIGHT line…')
        fut = self._cart.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        if not fut.done() or fut.result() is None:
            self.get_logger().warn(
                f'[{label}] cartesian service gave no answer — joint-space fallback')
            return self.go_pose(x, y, z, q, label)

        frac = float(fut.result().fraction)

        # Joint-path-to-Cartesian-path ratio: sum |q[k+1]-q[k]| over the returned
        # waypoints, divided by the straight-line distance actually travelled. A
        # genuinely straight descend in this workspace costs roughly 5-8 rad/m; an
        # IK branch flip costs an order of magnitude more. This is the number that
        # distinguishes "straight line" from "straight line with a hidden
        # discontinuity", which `fraction` alone cannot -- fraction counts achieved
        # waypoints, not continuity between them.
        pts = fut.result().solution.joint_trajectory.points
        if len(pts) > 1:
            steps = [max(abs(b - a) for a, b in zip(p0.positions, p1.positions))
                     for p0, p1 in zip(pts, pts[1:])]
            jpath = sum(
                sum(abs(b - a) for a, b in zip(p0.positions, p1.positions))
                for p0, p1 in zip(pts, pts[1:]))
            worst = max(steps)
            # A 5 mm Cartesian step should cost a small, roughly uniform joint step.
            # A single step far above the rest is an IK BRANCH FLIP: the solver
            # reseeded and returned a different arm configuration for essentially the
            # same pose. Individually valid, so `fraction` never notices -- but the
            # controller splines through it in joint space and the tool swings.
            # Cartesian length of this segment, measured from the arm's ACTUAL current
            # TCP via TF rather than assumed, so the ratio below is honest even when a
            # previous move ended somewhere unexpected.
            seg_len = None
            try:
                tf_now = self._tf_buf.lookup_transform(
                    PLANNING_FRAME, TCP_LINK, rclpy.time.Time())
                t = tf_now.transform.translation
                seg_len = math.dist((t.x, t.y, t.z), (x, y, z))
            except Exception:
                pass

            ratio = (jpath / seg_len) if (seg_len and seg_len > 1e-4) else float('nan')
            self.get_logger().info(
                f'[{label}] path diag: {len(pts)} waypoints, total joint path '
                f'{jpath:.3f} rad, largest single step {worst:.4f} rad '
                f'(median {sorted(steps)[len(steps)//2]:.4f})'
                + (f', {ratio:.0f} rad/m over {seg_len*1000:.0f} mm'
                   if ratio == ratio else ''))

            # GATE (2026-08-25). This replaces a `worst > 0.15` warning that could
            # never fire. The Cartesian service time-parameterizes its response at full
            # speed, resampling onto a fixed 0.1 s grid, so EVERY returned path -- clean
            # or pathological -- has its per-step joint motion capped at
            # max_velocity * resample_dt = 1.5 rad/s * 0.1 s = 0.15 rad. That the
            # observed maxima (0.110-0.150) sat just under the configured
            # revolute_jump_threshold was a numerical coincidence, not the threshold
            # doing work. A per-step check therefore cannot distinguish a straight line
            # from a zigzag, by construction.
            #
            # Joint travel per metre of Cartesian motion CAN: resampling redistributes
            # points along a path, it cannot add joint travel. Measured on this arm, a
            # clean single-branch 45 mm descent costs ~0.2 rad of max-joint change,
            # while the pathological "straight" paths logged 170-403 rad/m. 15 rad/m is
            # roughly 3x the clean cost -- comfortably above anything legitimate and far
            # below anything broken.
            #
            # With the task points inside the reachable band this should never fire.
            # Its job is to make a recurrence LOUD and non-executable rather than
            # silently played back under a "STRAIGHT OK (100% interpolated)" message.
            if ratio == ratio and ratio > CART_JOINT_TRAVEL_MAX:
                self.get_logger().error(
                    f'[{label}] REJECTED: {ratio:.0f} rad/m of joint travel over '
                    f'{seg_len*1000:.0f} mm — that is not a straight line '
                    f'(limit {CART_JOINT_TRAVEL_MAX:.0f} rad/m). Falling back to a '
                    f'joint-space plan rather than executing it.')
                return self.go_pose(x, y, z, q, label)

        if frac < min_fraction:
            self.get_logger().warn(
                f'[{label}] straight path only {frac * 100:.0f}% solvable '
                f'(need {min_fraction * 100:.0f}%) — joint-space fallback, '
                f'expect an arced approach on this move')
            return self.go_pose(x, y, z, q, label)

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = self._retime(fut.result().solution)

        if not self._exec.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(
                f'[{label}] /execute_trajectory unavailable — joint-space fallback')
            return self.go_pose(x, y, z, q, label)

        send_fut = self._exec.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut, timeout_sec=15.0)
        gh = send_fut.result() if send_fut.done() else None
        if gh is None or not gh.accepted:
            self.get_logger().warn(
                f'[{label}] straight-line goal rejected — joint-space fallback')
            return self.go_pose(x, y, z, q, label)

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=self._exec_timeout(goal.trajectory))
        if not res_fut.done():
            self.get_logger().error(
                f'[{label}] straight-line move did not finish in '
                f'{EXEC_TIMEOUT_SEC:.0f} s — cancelling')
            cancel_fut = gh.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_fut, timeout_sec=5.0)
            return False

        ok = res_fut.result().result.error_code.val == MOVEIT_SUCCESS
        self.get_logger().info(
            f'[{label}] STRAIGHT {"OK" if ok else "FAILED"} ({frac * 100:.0f}% interpolated)')
        return ok

    def add_table(self, bx, by, bz):
        """Register the pickup table as a MoveIt collision object."""
        table_top = bz - BOX_HALF_H
        co = CollisionObject()
        co.header.frame_id = PLANNING_FRAME
        co.id = 'pickup_table'
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = list(TABLE_SIZE)
        pose = Pose()
        pose.position = Point(x=float(bx), y=float(by),
                              z=float(table_top - TABLE_SIZE[2] / 2.0))
        pose.orientation.w = 1.0
        co.primitives.append(prim)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects.append(co)
        if self._scene.wait_for_service(timeout_sec=5.0):
            fut = self._scene.call_async(ApplyPlanningScene.Request(scene=scene))
            rclpy.spin_until_future_complete(self, fut)
            self.get_logger().info(f'added table collision @ z={pose.position.z:.3f}')

    def remove_table(self):
        co = CollisionObject()
        co.header.frame_id = PLANNING_FRAME
        co.id = 'pickup_table'
        co.operation = CollisionObject.REMOVE
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects.append(co)
        if self._scene.wait_for_service(timeout_sec=5.0):
            fut = self._scene.call_async(ApplyPlanningScene.Request(scene=scene))
            rclpy.spin_until_future_complete(self, fut)
            self.get_logger().info('removed table collision → retreat can plan freely')

    # ── Step 4–5: Grasp + Retract ─────────────────────────────────────────────

    def grasp_and_retract(self, bx, by, bz):
        """MECHANISM 2 — iterative top-down grasp.

        Instead of a single 1 cm descent, we:
          1. go to a HOVER point well above the box (top-down),
          2. re-perceive (median → kills the frame-to-frame drift), re-centre the
             target, and only descend once the estimate is STABLE,
          3. descend straight down to the calibrated grasp height, weld, close,
             lift, retract.

        Note: the depth camera is base-mounted (not on the wrist). Hovering the arm
        can occlude the box, so re-perception may return nothing — in that case we
        gracefully keep the (median-filtered) estimate from Step 3 and descend,
        i.e. at worst this equals the old behaviour, at best it's more precise.
        A wrist camera would make this a true eye-in-hand loop (sim-to-real note).
        """
        self.get_logger().info('=== Steps 4–5: Iterative top-down grasp ===')
        q = TOPDOWN_QUAT
        # Clearance above the box before descending, and the height the lift returns to.
        # REDUCED 0.12 -> 0.06 on 2026-08-02. The wrist+gripper stack is 213 mm long and
        # must hang straight down from the elbow, so every centimetre of clearance forces
        # the elbow higher — and with the pickup table raised to 0.14 m the elbow runs out
        # of room. Measured: 0.12 makes the hover pose unplannable at the 0.24 m dock
        # ("[hover] FAILED"), while 0.06 plans fine and still clears the table.
        # Reduced 0.06 -> 0.03 on 2026-08-21. Hover radius is what puts the START of the
        # descend on the workspace boundary: r = sqrt(x^2 + z^2), and with x pinned at
        # 0.240 m by the dock geometry, the hover height is the ONLY term still free.
        #   0.060 m hover -> r = sqrt(0.240^2 + 0.075^2) = 0.2514 m = 99.4% of max reach
        #   0.030 m hover -> r = sqrt(0.240^2 + 0.045^2) = 0.2442 m = 96.5% of max reach
        # Also shortens the path through the ill-conditioned band. This is a mitigation,
        # not the fix -- x = 0.240 dominates the radius and is set by the table geometry.
        HOVER      = 0.03
        GRASP_Z    = GRASP_ABOVE   # 0.0 — gripper_tcp IS the grasp point (calibrated)
        STABLE_TOL = 0.012   # estimate "settled" when it shifts < 1.2 cm
        MAX_ITERS  = 3

        if not self._wait_for_move_group():
            return False
        if not self.go_named('ready'):
            return False
        self.set_gripper(GRIPPER_OPEN, 'open')
        self.add_table(bx, by, bz)

        tx, ty, tz = bx, by, bz   # dock-geometry target (already incl. GRASP_OFFSET)

        # ── hover above the box (single shot, no close-range re-perception) ──
        # At the dock the base camera OVER-READS the distance, so re-perceiving here
        # would push the goal past the arm's reach and make IK fail (the bug we hit).
        # The odometry-measured dock distance is the reliable target, so we trust it.
        # Try decreasing clearances rather than aborting on the first failure: "unreachable"
        # here usually means "too high", not "too far", and a lower hover still works.
        #
        # Order is informed by reach_map.csv (a prior offline reachability sweep) rather
        # than always trying HOVER first: if a nearby pose is already known to fail, don't
        # waste a live OMPL planning cycle finding that out again. This does NOT skip
        # planning — go_pose() still calls MoveIt for whichever clearance is tried, and
        # every clearance is still attempted in order if the reordered ones fail — it only
        # changes which one goes first. See reach_lookup.py for why "outside the swept
        # envelope" deliberately falls back to the original order instead of guessing.
        # De-duplicate: HOVER became 0.03 on 2026-08-21, which is already in this list,
        # so the candidates were (0.03, 0.045, 0.03) and 0.03 got planned TWICE -- two
        # identical 5 s OMPL failures back to back, visible in the log as
        # "[hover 0.030 m] FAILED" appearing twice. dict.fromkeys preserves order.
        _clearances = tuple(dict.fromkeys((HOVER, 0.045, 0.03)))
        clearance_order = reach_lookup.rank_clearances(tx, ty, tz, _clearances)
        if clearance_order != list(_clearances):
            self.get_logger().info(
                f'reach map reorders hover attempt to {clearance_order} '
                f'(originally {list(_clearances)})')
        hovered = False
        for clearance in clearance_order:
            if self.go_pose(tx, ty, tz + clearance, q, f'hover {clearance:.3f} m'):
                hovered = True
                if clearance < HOVER:
                    self.get_logger().warn(
                        f'hover reduced to {clearance:.3f} m — headroom is tight at this '
                        f'table height and dock distance')
                break
        if not hovered:
            hit = reach_lookup.nearest(tx, ty, tz + HOVER)
            if hit is None or hit[0] > reach_lookup.MAX_TRUST_DIST:
                self.get_logger().error(
                    f'no reachable hover above the box — aborting. Target ({tx:.3f}, '
                    f'{ty:.3f}) is outside the characterized reach envelope '
                    f'(reach_map.csv has no sample within {reach_lookup.MAX_TRUST_DIST} m) '
                    f'— this looks like a bad target, not a genuine reach edge case.')
            else:
                self.get_logger().error(
                    f'no reachable hover above the box — aborting. Nearest characterized '
                    f'sample is {hit[0]:.3f} m away and was itself a failure — this is a '
                    f'genuine reach-envelope edge, not a perception error.')
            return False

        # ── descend straight down to the grasp point ────────────────────────
        # STRAIGHT is load-bearing here, not decorative. A joint-space plan to this
        # same pose arrives along an arc, and the lateral component of that arc is
        # what knocks the 35 mm box over before the fingers reach it.
        # Single-branch IK first (see go_pose_branch): at this reach the Cartesian
        # sampler reconfigures between waypoints and the "straight" path executes as a
        # joint-space swing. Fall back to the Cartesian path only if IK can't produce a
        # same-branch solution, so nothing is lost when the arm is well-conditioned.
        if self.go_pose_branch(tx, ty, tz + GRASP_Z, q, 'grasp') is not True:
            self.get_logger().info('[grasp] single-branch IK unavailable — '
                                   'falling back to the Cartesian path')
            if not self.go_pose_straight(tx, ty, tz + GRASP_Z, q, 'grasp'):
                self.get_logger().error('grasp pose failed — aborting')
                return False

        # Remove table collision so the lift is not blocked by a phantom
        # table-gripper collision. Done once, before the attempt loop.
        self.remove_table()

        # ── close → lift → VERIFY loop ──────────────────────────────────────
        # Retries are now driven by the post-lift VISION check, not by the
        # aperture classifier (which used the uncalibrated BOX_CONTACT_ANGLE and
        # rejected physically-fine grasps — see close_until_contact's docstring).
        # A retry therefore means "we lifted and the box was still on the table",
        # which is real evidence, instead of "a guessed angle missed a window".
        verified = False
        result = None
        attempt = 0
        for attempt in range(1, MAX_GRASP_RETRIES + 2):
            if attempt > 1:
                self.get_logger().warn(
                    f'[grasp] not verified — retry {attempt - 1}/{MAX_GRASP_RETRIES}')
                self.attach(False)                      # drop any stale weld
                self.set_gripper(GRIPPER_OPEN, 'open')
                if (self.go_pose_branch(tx, ty, tz + HOVER, q, 'hover for retry')
                        is not True
                        and not self.go_pose_straight(tx, ty, tz + HOVER, q,
                                                      'hover for retry')):
                    break
                if (self.go_pose_branch(tx, ty, tz + GRASP_Z, q,
                                        f'grasp retry {attempt - 1}') is not True
                        and not self.go_pose_straight(tx, ty, tz + GRASP_Z, q,
                                                      f'grasp retry {attempt - 1}')):
                    break

            # Heights of ALL candidate boxes before the close. The check after the
            # lift compares every one of them and takes whichever rose -- that is the
            # box in the gripper. Measuring all of them is what makes the check
            # immune to guessing the target wrong, which is how a good lift was
            # previously reported as a failure (see _all_box_world_z's docstring).
            z_before = self._all_box_world_z()

            result = self.close_until_contact()

            # Settle before the next MoveGroup plan (the lift, below). Ported from the
            # 2026-08-11 grip_bench.py fix: close_until_contact() ends the moment the
            # gripper stalls/finishes, but the ARM may still be micro-settling
            # (residual servo motion, /joint_states lag). The lift plans from a
            # "current state" snapshot; if that snapshot is stale by even a fraction of
            # a degree, MoveIt's execution-time check (0.01 rad tolerance) rejects the
            # whole trajectory and silently retries at a different clearance -- which
            # looks like the arm "deciding" to move (e.g. the base joint rotating) for
            # no reason right after the grasp closes, box already between the fingers.
            for _ in range(5):
                rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.3)

            # AIR means the APERTURE never stalled. Whether that means "nothing is
            # between the fingers" depends on which grasp mechanism is running.
            #
            # LEGACY WELD: the weld is a teleport we command, so it will happily
            # grab a box 8 cm away and vision then reports a perfect grasp. The
            # aperture is the only guard against faking it, so AIR must abort.
            #
            # PHYSICS GRASP: the joint is created by gazebo_grasp_plugin from REAL
            # finger contacts. It cannot fire on nothing, so a false GRASPED cannot
            # fake anything — and the post-lift ground-truth check is a strictly
            # better test than the aperture anyway. Short-circuiting here just
            # throws away grasps the plugin already made: with a light box the
            # fingers can close around it without ever producing a sustained
            # aperture stall. Let the lift decide.
            if result == GraspResult.AIR:
                if not self._physics_grasp:
                    self.get_logger().warn(
                        '[grasp] AIR — gripper closed on nothing, skipping lift/verify')
                    verified = False
                    self._log_grasp_attempt(attempt, result, verified)
                    continue
                self.get_logger().warn(
                    '[grasp] no aperture stall, but physics grasp is active — '
                    'lifting anyway and letting ground truth decide')

            # Lift — retry 3x AT THE SAME HEIGHT (OMPL/execution can be flaky on the
            # first attempt). STRAIGHT up: this is the motion whose shortfall was
            # measured at 0.031 m against a commanded 0.060 m, because the joint-space
            # plan arced and rotated instead of rising. With a task-space path the box
            # should now actually rise what it was told to, which also restores the
            # meaning of _verify_grasp_truth's min_rise threshold.
            #
            # BUG FIXED 2026-08-19: this used to retry at (HOVER, 0.045, 0.03) -- a
            # DESCENDING ladder copy-pasted from the pre-grasp hover-clearance loop
            # above, where trying lower makes sense (getting closer to an
            # unreachable target). For a LIFT that logic is backwards: a "failed"
            # attempt still executes the full trajectory before go_pose_straight
            # reports false (see its docstring), so what actually happened on
            # hardware was the arm lifting to 6 cm, then being commanded back DOWN
            # to 4.5 cm, then DOWN again to 3 cm -- visible as the box bobbing
            # up/down/up/down three times right after the grasp, for no reason
            # visible from outside. Retrying the SAME target is the correct fix: a
            # failure here means "that attempt didn't execute cleanly", not "try a
            # smaller lift."
            lifted = False
            for i in range(3):
                lbl = f'lift {HOVER:.3f} m (attempt {i + 1}/3)'
                # Same reasoning as the descend: the lift STARTS at the boundary pose,
                # so it re-traverses the same ill-conditioned region in reverse.
                if self.go_pose_branch(tx, ty, tz + HOVER, q, lbl) is True:
                    lifted = True
                    break
                if self.go_pose_straight(tx, ty, tz + HOVER, q, lbl):
                    lifted = True
                    break
                time.sleep(0.3)
            if not lifted:
                self.get_logger().warn('lift attempts failed — continuing anyway')

            # Phase 0 — did the box actually leave the table?
            # Preference order:
            #   1. Gazebo ground truth (exact, sim only)
            #   2. Vision (the only option on real hardware, but measured 45% error
            #      on the lift displacement — see _box_world_z's docstring)
            # Checked BEFORE retracting home, while the table is still in view.
            if not lifted:
                verified = False
            else:
                verified = self._verify_grasp_truth(z_before)
                if verified is None:
                    self.get_logger().info(
                        '[verify] Gazebo state unavailable — falling back to vision')
                    verified = self._verify_grasp_vision(tx, ty, tz, tol=0.04)

            # ── PHYSICS GRASP: this is where _weld_active gets its value ────────
            # In legacy weld mode the flag echoed our own /grasp_attach command, so it
            # only ever meant "we asked for a weld" — it was True even when the fingers
            # had closed on air. With a real joint there is no command to echo, so the
            # flag is redefined as an OBSERVATION: the box measurably left the table.
            # That is strictly more honest, and it is what gates the place step and
            # feeds pick_success in the metrics CSV.
            if self._physics_grasp:
                self._weld_active = bool(verified)
            # Measure centring WHILE the box is still held up — the only moment the
            # box is rigidly located relative to the gripper, so the offset is
            # meaningful. Done before _log_grasp_attempt so the row carries it.
            if verified:
                self._measure_grasp_centering()

            self._log_grasp_attempt(attempt, result, verified)
            if verified:
                self.get_logger().info(f'[grasp] VERIFIED on attempt {attempt}')
                break

        if hasattr(self, '_current_cycle'):
            # pick_success only trusts an explicit True — None (inconclusive) or
            # False both leave it False, since neither is confirmed success.
            self._current_cycle['pick_success'] = bool(verified)

        if not verified:
            self.get_logger().error(
                f'[grasp] failed after {attempt} attempts (vision never confirmed '
                f'the box left the table) — aborting')
            self.attach(False)
            self.set_gripper(GRIPPER_OPEN, 'open')
            self.go_named('ready')
            return False

        # Retract to home
        self.go_named('home')
        self.get_logger().info('=== Grasp + Retract complete ===')
        return True

    # ── Step 5b: Place onto the stack ───────────────────────────────────────────

    def _stack_xy(self):
        """Live foundation (x, y) in base_link — tune with `ros2 param set`."""
        return (float(self.get_parameter('stack_x').value),
                float(self.get_parameter('stack_y').value))

    def place_on_stack(self, level):
        """Place the currently-held box onto the stack at `level` (0 = foundation).

        Top-down placement at the hardcoded foundation (stack_x, stack_y). The resting
        surface for this box is the foundation top plus `level` full box heights, so each
        box lands on the one below. We mirror the grasp geometry: the TCP is the point
        the box is held at, so to rest the box bottom on the surface the TCP target is
        surface + BOX_HALF_H + GRASP_ABOVE — and GRASP_ABOVE is now 0, i.e. the tool goes
        to where the box centre must end up. Hover → descend → release → lift.

        Assumes the box is still held (welded or gripped) and the arm is at 'home'.
        """
        self.get_logger().info(f'=== Step 5b: Place on stack — level {level} ===')
        q = TOPDOWN_QUAT
        sx, sy = self._stack_xy()
        surface_z = float(self.get_parameter('stack_surface_z').value)

        rest_surface = surface_z + level * BOX_HEIGHT          # top of the stack so far
        place_z      = rest_surface + BOX_HALF_H + GRASP_ABOVE  # TCP height to release at
        hover_z      = rest_surface + BOX_HALF_H + STACK_HOVER

        if not self.go_named('ready'):
            return False
        # Approach from above so we descend straight onto the stack (avoids clipping it).
        if not self.go_pose(sx, sy, hover_z, q, f'stack hover L{level}'):
            self.get_logger().error('stack hover failed — aborting place')
            return False
        if (self.go_pose_branch(sx, sy, place_z, q, f'stack place L{level}') is not True
                and not self.go_pose_straight(sx, sy, place_z, q, f'stack place L{level}')):
            self.get_logger().error('stack place failed — aborting place')
            return False

        # Release. Legacy weld: drop the teleport first so physics takes over, then open.
        # Physics grasp: opening the fingers past release_position destroys the joint, so
        # set_gripper alone is the release (attach() is a no-op and set_gripper clears
        # _weld_active).
        if USE_WELD:
            self.attach(False)
        self.set_gripper(GRIPPER_OPEN, 'release')
        time.sleep(0.4)   # let the box settle before retracting

        # Lift straight up off the placed box, then retract home.
        for attempt in range(3):
            lbl = f'stack retract L{level} (try {attempt + 1})'
            if self.go_pose_branch(sx, sy, hover_z, q, lbl) is True:
                break
            if self.go_pose_straight(sx, sy, hover_z, q, lbl):
                break
            time.sleep(0.5)
        self.go_named('home')
        self.get_logger().info(f'=== Box placed at stack level {level} ✓ ===')
        return True

    # ── Step 6: Back up ───────────────────────────────────────────────────────

    def back_up(self):
        self.get_logger().info('=== Step 6: Backing up ===')
        self.unpin_base()
        twist = Twist()
        twist.linear.x = BACKUP_VEL_X
        end = time.time() + BACKUP_SEC
        while time.time() < end:
            self._cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
        # Stop
        self._cmd_vel_pub.publish(Twist())
        self.get_logger().info(f'backed up ~{abs(BACKUP_VEL_X) * BACKUP_SEC:.2f} m ✓')

    # ── Tag-dock helpers ──────────────────────────────────────────────────────

    def _read_tag_live(self):
        """One fresh /place_tag_pose reading: (x, y, range, bearing, yaw) or None."""
        rclpy.spin_once(self, timeout_sec=0.05)
        p = self._latest_tag_pose
        if p is None or p.header.frame_id != PLANNING_FRAME:
            return None
        x, y = p.pose.position.x, p.pose.position.y
        tag_range = math.hypot(x, y)
        if tag_range < 0.05 or tag_range > 3.0:
            return None
        bearing = math.atan2(y, x)
        q = p.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return x, y, tag_range, bearing, yaw

    @staticmethod
    def _topdown_yaw_quat(tag_yaw: float):
        """Compute topdown ⊗ yaw(tag_yaw): vertical gripper spun by tag_yaw.

        TOPDOWN_QUAT = (-0.7071, 0, 0, 0.7071)  → gripper points straight down.
        Post-multiplied by yaw rotation around Z so fingers align with box edges.

        Hamilton product:  q1=(x1=-0.7071, y1=0, z1=0, w1=0.7071), q2=(0, 0, sz, cz)
            w =  0.7071*cz,  x = -0.7071*cz,  y = 0.7071*sz,  z = 0.7071*sz
        """
        sz = math.sin(tag_yaw / 2.0)
        cz = math.cos(tag_yaw / 2.0)
        return (-0.7071 * cz, 0.7071 * sz, 0.7071 * sz, 0.7071 * cz)  # qx,qy,qz,qw

    # ── Tag-based dock controller ─────────────────────────────────────────────

    @staticmethod
    def _wrap(a):
        return (a + math.pi) % (2 * math.pi) - math.pi

    def _rotate_by(self, dyaw, timeout=8.0):
        """Turn in place by `dyaw` radians (signed), closed-loop on odometry yaw.

        The two pivots of a crab maneuver must cancel: whatever the first one gets
        wrong shows up as heading error after the second, and Phase A has to take it
        out again. Closing on odometry instead of timing the turn keeps that residue
        small enough for a single re-square to absorb.
        """
        if self._odom_yaw is None:
            rclpy.spin_once(self, timeout_sec=0.5)
        if self._odom_yaw is None:
            self.get_logger().warn('[dock B] no odometry yaw — cannot pivot precisely')
            return False

        target   = self._wrap(self._odom_yaw + dyaw)
        deadline = time.time() + timeout
        TOL      = 0.02   # rad ≈ 1.1°
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._odom_yaw is None:
                continue
            err = self._wrap(target - self._odom_yaw)
            if abs(err) < TOL:
                self._cmd_vel_pub.publish(Twist())
                time.sleep(0.25)
                return True
            cmd = Twist()
            w = max(-PLACE_DOCK_CRAB_ROT, min(PLACE_DOCK_CRAB_ROT, 2.0 * err))
            if abs(w) < 0.25:          # same no-crawl rule as Phase A
                w = math.copysign(0.25, w)
            cmd.angular.z = w
            self._cmd_vel_pub.publish(cmd)
        self._cmd_vel_pub.publish(Twist())
        time.sleep(0.25)
        self.get_logger().warn(f'[dock B] pivot of {math.degrees(dyaw):+.0f}° timed out')
        return False

    def _drive_straight(self, dist, speed=PLACE_DOCK_CRAB_FWD, timeout=15.0):
        """Drive forward `dist` metres (positive only), closed-loop on odom distance."""
        if self._odom is None:
            rclpy.spin_once(self, timeout_sec=0.5)
        if self._odom is None:
            self.get_logger().warn('[dock B] no odometry — cannot drive a measured leg')
            return False

        x0, y0   = self._odom
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            gone = self._odom_dist_since(x0, y0)
            if gone is None:
                continue
            if gone >= dist:
                self._cmd_vel_pub.publish(Twist())
                time.sleep(0.25)
                return True
            cmd = Twist()
            cmd.linear.x = speed
            self._cmd_vel_pub.publish(cmd)
        self._cmd_vel_pub.publish(Twist())
        time.sleep(0.25)
        self.get_logger().warn(f'[dock B] straight leg of {dist:.3f} m timed out')
        return False

    def _centre_on_tag_normal(self):
        """Phase B — shift the base sideways onto the tag's face-normal line.

        Assumes Phase A has already squared the heading, which is what makes the
        tag's y in base_link equal to the signed lateral offset. Corrects it with a
        crab: pivot 90° toward the offset, drive it, pivot back. The drive leg is
        perpendicular to the approach axis, so this never carries the robot toward
        the table.

        Iterates: each pass re-measures from the tag, so odometry error in one leg is
        removed by the next rather than accumulating. Returns True if the base ended
        inside PLACE_DOCK_LAT_TOL of the line.
        """
        if not self.get_parameter('dock_lateral_align').value:
            self.get_logger().info('Phase B: disabled by parameter — skipping')
            return True

        for i in range(PLACE_DOCK_LAT_ITERS):
            t = self._read_tag_live()
            if t is None:
                self.get_logger().warn('Phase B: tag not visible — skipping lateral centring')
                return False
            _, ty, tag_range, _, _ = t

            if abs(ty) <= PLACE_DOCK_LAT_TOL:
                self.get_logger().info(
                    f'Phase B: centred ✓ lateral offset {ty * 1000:+.0f} mm '
                    f'(tol ±{PLACE_DOCK_LAT_TOL * 1000:.0f} mm)')
                return True

            if abs(ty) > PLACE_DOCK_LAT_MAX:
                self.get_logger().error(
                    f'Phase B: lateral offset reads {ty:+.3f} m, beyond the '
                    f'{PLACE_DOCK_LAT_MAX:.2f} m sanity limit — refusing to crab on '
                    f'what is probably a bad tag reading. Continuing uncentred.')
                return False

            self.get_logger().info(
                f'Phase B (pass {i + 1}/{PLACE_DOCK_LAT_ITERS}): off the normal line by '
                f'{ty * 1000:+.0f} mm at range {tag_range:.2f} m — crabbing across')

            pivot = math.copysign(math.pi / 2.0, ty)   # +y is left, so +ty ⇒ turn left
            if not self._rotate_by(pivot):
                return False
            if not self._drive_straight(abs(ty)):
                return False
            if not self._rotate_by(-pivot):
                return False

            # The pivots leave a little heading error; take it out before the next
            # measurement, or the ty we read will not mean "lateral offset" any more.
            self._square_to_tag(timeout=5.0)

        t = self._read_tag_live()
        resid = abs(t[1]) if t else None
        if resid is not None and resid <= PLACE_DOCK_LAT_TOL:
            self.get_logger().info(f'Phase B: centred ✓ residual {resid * 1000:.0f} mm')
            return True
        self.get_logger().warn(
            f'Phase B: still {"%.0f mm" % (resid * 1000) if resid is not None else "unknown"} '
            f'off after {PLACE_DOCK_LAT_ITERS} passes — approaching anyway')
        return False

    def _square_to_tag(self, timeout=8.0):
        """Phase A — rotate in place until the chassis is squared to the tag FACE.

        Squaring on yaw (not bearing) is the load-bearing alignment: the
        tag_dock_estimator convention has the tag's Z axis point out of the face
        toward the robot, so a squarely-facing robot sees tag yaw ≈ π in base_link
        (heading_err = wrap(yaw − π) → 0). Bearing alone only centres the tag ahead —
        if the robot is not exactly on the face-normal line, that still leaves the
        chassis angled to the face.

        Returns (aligned, tag_seen).
        """
        self.get_logger().info('Phase A: squaring to tag face…')
        ok_count = 0
        tag_seen = False
        aligned  = False
        deadline = time.time() + timeout
        while time.time() < deadline:
            t = self._read_tag_live()
            if t is None:
                self._cmd_vel_pub.publish(Twist())
                self.get_logger().info('Phase A: no tag — stopping', throttle_duration_sec=1.0)
                time.sleep(0.1)
                continue
            tag_seen = True
            _, _, tag_range, bearing, yaw = t
            err = self._wrap(yaw - math.pi)
            self.get_logger().info(
                f'Phase A: bearing={math.degrees(bearing):+.1f}° '
                f'heading_err={math.degrees(err):+.1f}° range={tag_range:.3f}m',
                throttle_duration_sec=0.5)
            if abs(err) < PLACE_DOCK_YAW_TOL:
                ok_count += 1
                if ok_count >= 4:
                    self.get_logger().info('Phase A: aligned ✓')
                    aligned = True
                    break
            else:
                ok_count = 0
            cmd = Twist()
            w = max(-PLACE_DOCK_MAX_ROT,
                    min(PLACE_DOCK_MAX_ROT, PLACE_DOCK_K_ROT * err))
            if abs(w) < PLACE_DOCK_MIN_ROT:      # never crawl; see PLACE_DOCK_MIN_ROT
                w = math.copysign(PLACE_DOCK_MIN_ROT, w)
            cmd.angular.z = w
            self._cmd_vel_pub.publish(cmd)
            time.sleep(0.05)
        self._cmd_vel_pub.publish(Twist())
        time.sleep(0.3)
        return aligned, tag_seen

    def dock_to_tag(self):
        """Closed-loop dock to the AprilTag on the place table face.

        Phase A — face: rotate in place until the chassis is squared to the tag face.
        Phase B — centre: crab sideways until the chassis sits ON the face-normal
                  line. Without this the robot squares up wherever Nav2 left it and
                  drives in parallel to the correct line, arriving offset by however
                  far off-centre it started.
        Phase A' — re-square: the crab's two pivots leave a little heading error, and
                  Phase C amplifies heading into lateral error over a ~1.3 m run.
        Phase C — straight approach: drive forward with yaw trims keeping the tag
                  centred, until tag range ≤ dock_range.
        Phase D — latch: record drop (x,y) and tag_yaw for the arm.

        Returns True on success, False on timeout/no-tag.
        """
        self.get_logger().info('=== Place Step 1b: dock_to_tag ===')
        dock_range = float(self.get_parameter('dock_range').value)

        def clamp(v, lo, hi):
            return max(lo, min(hi, v))

        # ── Phase A: square to the tag face ──────────────────────────────────────
        # tag_seen tracks whether ANY reading arrived — if the tag was never visible
        # from the Nav2 stop (too far / wrong angle), skip the rest instead of burning
        # 30 s printing "tag lost".
        _, tag_seen = self._square_to_tag(timeout=8.0)

        # ── Phase B + A': centre on the normal line, then re-square ──────────────
        # Done HERE, at the ~1.3 m Nav2 standoff, not close in: there is room for the
        # crab maneuver, the tag is comfortably in frame, and any residual gets one
        # more correction from Phase C's trim on the way in.
        if tag_seen:
            self._centre_on_tag_normal()
            self._square_to_tag(timeout=5.0)

        if not tag_seen:
            # Tag never appeared — skip Phase C (would just print "tag lost" for 30 s)
            # and fall straight to Phase D, which will return False → map-pos fallback.
            self.get_logger().warn(
                'Phase A: tag never visible — skipping Phase C, going direct to Phase D')

        # ── Phase C: straight approach until dock_range (skipped if tag never seen) ──
        if tag_seen:
            self.get_logger().info(f'Phase C: approaching to {dock_range:.2f} m…')
            deadline = time.time() + 30.0
            # Stall guard. If dock_range is set below what the chassis can
            # physically reach (bumper is 0.189 m ahead of base_link), `remaining`
            # never hits zero and this loop drives the wheels into the table for
            # the whole 30 s — which is exactly what a dock_range of 0.15 did,
            # slipping the wheels and corrupting the odometry the drop point is
            # computed from. Bail out as soon as the range stops improving.
            STALL_EPS  = 0.005   # m of progress that counts as "still moving"
            # Distance-aware, not a single constant. 2026-08-11 tightened this to 0.5 s
            # everywhere to stop the wheels grinding into the table after real contact
            # (only ~1 cm of designed bumper clearance) -- but applied to the WHOLE
            # approach, it also fires on ordinary AprilTag reading noise during the long
            # cruise-in from ~1.7 m out, aborting almost immediately ("no progress for
            # 0 s") and latching a drop point from a meter+ away with a garbage yaw.
            # Diagnosed 2026-08-12 from exactly that log line. Fix: patient while far
            # away (cruising, noise is expected and harmless), tight only once close
            # enough that a real physical blockage is the actual risk.
            STALL_SEC_FAR   = 2.0
            STALL_SEC_CLOSE = 0.5
            STALL_CLOSE_RANGE = 0.15   # m of `remaining` below which "tight" applies
            best_range = float('inf')
            last_gain  = time.time()
            while time.time() < deadline:
                t = self._read_tag_live()
                if t is None:
                    self._cmd_vel_pub.publish(Twist())
                    self.get_logger().info('Phase C: tag lost — stopping', throttle_duration_sec=1.0)
                    time.sleep(0.1)
                    continue
                _, _, tag_range, bearing, _ = t
                remaining = tag_range - dock_range
                self.get_logger().info(
                    f'Phase C: range={tag_range:.3f} remaining={remaining:.3f}',
                    throttle_duration_sec=0.5)
                if remaining <= 0.0:
                    self.get_logger().info(f'Phase C: docked at range={tag_range:.3f} m ✓')
                    break
                # ── stall guard (see STALL_EPS above) ────────────────────────
                stall_sec = (STALL_SEC_CLOSE if remaining < STALL_CLOSE_RANGE
                             else STALL_SEC_FAR)
                if tag_range < best_range - STALL_EPS:
                    best_range = tag_range
                    last_gain  = time.time()
                elif time.time() - last_gain > stall_sec:
                    self.get_logger().warn(
                        f'Phase C: STALLED at range={tag_range:.3f} m (target '
                        f'{dock_range:.3f} m) — no progress for {stall_sec:.1f} s. '
                        f'The chassis cannot get closer; stopping here rather than '
                        f'grinding the wheels.')
                    break
                cmd = Twist()
                cmd.linear.x  = clamp(PLACE_DOCK_K_FWD * remaining,
                                       PLACE_DOCK_MIN_FWD, PLACE_DOCK_MAX_FWD)
                cmd.angular.z = clamp(PLACE_DOCK_K_ROT * bearing,
                                       -PLACE_DOCK_MAX_ROT, PLACE_DOCK_MAX_ROT)
                self._cmd_vel_pub.publish(cmd)
                time.sleep(0.05)
            self._cmd_vel_pub.publish(Twist())
            time.sleep(0.5)

        # ── Phase D: latch drop point and tag yaw ────────────────────────────
        # Take a fresh reading after settling to get the most accurate latch.
        rclpy.spin_once(self, timeout_sec=0.3)
        t = self._read_tag_live()
        drop = self._latest_drop_pt
        if t is None or drop is None:
            self.get_logger().error('Phase D: no tag or drop point after dock — cannot place')
            return False

        yaw_offset = float(self.get_parameter('place_yaw_offset').value)

        # F2 (2026-08-25): bias the drop toward the robot and clamp its lateral offset.
        #
        # The tag-derived drop point aims at the place table's CENTRE, which sits at
        # the far end of the arm's reach -- and Phase B's lateral centring times out
        # regularly, so the latched y has been observed as large as +0.048 m. Against
        # reach_map.csv at place height (z = -0.025) that combination is outside the
        # measured workspace: no near IK branch exists, the move is refused, and the
        # Cartesian fallback builds a multi-thousand-degree zigzag. That is the
        # "arm struggles to lift after placing" motion.
        #
        # Biasing x inward by 20 mm puts the drop inside the reachable column while
        # still landing well within an 80 mm table (the box is 40 mm, so a 20 mm
        # inward bias leaves 20 mm of table beyond the box's near face).
        #
        # The y clamp is deliberately a CLAMP, not a rejection: a large lateral offset
        # means the dock was poor, and placing 20 mm off-centre on the table beats
        # aiming at a pose the arm cannot reach. It is logged loudly when it bites.
        PLACE_NEAR_EDGE_BIAS = float(self.get_parameter('place_near_edge_bias').value)
        PLACE_MAX_LATERAL    = 0.02   # m, |y| cap on the latched drop
        raw_x, raw_y = float(drop.point.x), float(drop.point.y)
        clamped_y = max(-PLACE_MAX_LATERAL, min(PLACE_MAX_LATERAL, raw_y))
        if abs(raw_y - clamped_y) > 1e-6:
            self.get_logger().warn(
                f'Phase D: lateral drop offset {raw_y:+.3f} m exceeds '
                f'±{PLACE_MAX_LATERAL:.3f} m — clamped to {clamped_y:+.3f}. The dock '
                f'is off the table centre line (Phase B centring likely timed out); '
                f'the box will land off-centre but within reach.')
        self._latched_drop    = (raw_x - PLACE_NEAR_EDGE_BIAS, clamped_y)
        self.get_logger().info(
            f'Phase D drop biased {PLACE_NEAR_EDGE_BIAS*1000:.0f} mm inward: '
            f'({raw_x:.3f}, {raw_y:+.3f}) -> '
            f'({self._latched_drop[0]:.3f}, {self._latched_drop[1]:+.3f})')
        self._latched_tag_yaw = t[4] + yaw_offset    # tag yaw + calibration offset
        self.get_logger().info(
            f'Phase D latched: drop=({self._latched_drop[0]:.3f},{self._latched_drop[1]:.3f}) '
            f'tag_yaw={math.degrees(self._latched_tag_yaw):.1f}°')
        return True

    # ── Place: map-position fallback dock (when tag not visible) ────────────

    def _dock_by_map_pos(self):
        """Drive south until robot map-y ≤ PLACE_MAP_Y_DOCK, then compute the
        exact table-centre position in base_link from TF + known map coordinates.

        This replaces the old hardcoded (0.28, 0.0) with a computation that
        accounts for (a) where the robot actually stopped and (b) any lateral
        offset between the robot centre-line and the table centre.
        """
        self.get_logger().info(
            f'map-pos dock: driving south to map_y ≤ {self._place_map_y_dock}')

        deadline = time.time() + 25.0
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tf = self._tf_buf.lookup_transform(
                    'map', 'base_link',
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.3))
            except Exception as e:
                self.get_logger().info(
                    f'TF map→base_link wait: {e}', throttle_duration_sec=2.0)
                time.sleep(0.1)
                continue

            robot_y   = tf.transform.translation.y
            remaining = robot_y - self._place_map_y_dock   # positive → still heading south
            self.get_logger().info(
                f'map-dock: map_y={robot_y:.3f}  remaining={remaining:.3f}',
                throttle_duration_sec=0.8)

            if remaining <= 0.03:
                break

            fwd = max(0.04, min(0.10, 0.8 * remaining))
            cmd = Twist()
            cmd.linear.x = fwd
            self._cmd_vel_pub.publish(cmd)
            time.sleep(0.05)

        self._cmd_vel_pub.publish(Twist())
        time.sleep(0.5)   # let the robot fully settle before the final TF read

        # Compute the table centre in base_link from the robot's actual final pose.
        # Robot heading (yaw) may not be exactly -π/2; accounting for it removes
        # any small angular residual from the Nav2 approach.
        try:
            tf_final = self._tf_buf.lookup_transform(
                'map', 'base_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0))
            rx  = tf_final.transform.translation.x
            ry  = tf_final.transform.translation.y
            q   = tf_final.transform.rotation
            yaw = 2.0 * math.atan2(q.z, q.w)          # robot heading in map frame

            # Vector robot → table centre in map frame
            dx = self._table_map_x - rx
            dy = self._table_map_y - ry

            # Rotate into base_link:  forward = (cos yaw, sin yaw), left = (-sin yaw, cos yaw)
            table_x =  dx * math.cos(yaw) + dy * math.sin(yaw)
            table_y = -dx * math.sin(yaw) + dy * math.cos(yaw)

            self.get_logger().info(
                f'map-dock settled: robot=({rx:.3f},{ry:.3f}) yaw={math.degrees(yaw):.1f}° '
                f'→ table in base_link=({table_x:.3f},{table_y:.3f})')

        except Exception as e:
            # Pure-geometry fallback if TF fails
            table_x = abs(self._table_map_y - self._place_map_y_dock)
            table_y = 0.0
            self.get_logger().warn(
                f'TF final lookup failed ({e}); geometric fallback: ({table_x:.3f},0.000)')

        # Safety clamp: arm max reach ~0.30 m; reject anything implausible
        table_x = max(0.15, min(0.30, table_x))
        table_y = max(-0.15, min(0.15, table_y))

        self._latched_drop    = (table_x, table_y)
        self._latched_tag_yaw = 0.0
        return True

    # ── Place: navigate to second table ──────────────────────────────────────

    def navigate_to_place_table(self):
        """Nav2 to the pre-dock position, then creep forward to place distance."""
        self.get_logger().info('=== Place Step 1: Navigate to place table ===')
        self._activate_nav2_cmdvel()

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self._nav_place_x
        goal.pose.pose.position.y = self._nav_place_y
        half = self._nav_place_yaw / 2.0
        goal.pose.pose.orientation.z = math.sin(half)
        goal.pose.pose.orientation.w = math.cos(half)

        gh = None
        for attempt in range(15):
            send_fut = self._nav.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_fut, timeout_sec=10.0)
            if send_fut.done():
                gh = send_fut.result()
                if gh is not None and gh.accepted:
                    self.get_logger().info('Nav2 goal accepted — driving to place table…')
                    break
            self.get_logger().warn(f'Place nav goal rejected (attempt {attempt + 1}/15) — retrying…')
            end = self.get_clock().now() + Duration(seconds=2.0)
            while self.get_clock().now() < end:
                rclpy.spin_once(self, timeout_sec=0.1)
        else:
            self.get_logger().error('Could not reach place table — aborting place')
            return False

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=NAV_TIMEOUT_SEC)
        if not res_fut.done() or res_fut.result().status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error('Navigation to place table failed')
            return False

        self.get_logger().info('Nav2 arrived — docking to place table…')

        # Line up with the tag BEFORE handing /cmd_vel to the dock, so Phase C's
        # ~1.3 m approach runs perpendicular to the table face rather than across it.
        self._align_to_place_tag()

        self._silence_nav2_cmdvel()
        time.sleep(0.5)

        # Primary: AprilTag visual dock — aligns heading precisely regardless of
        # Nav2 final pose, then latches drop (x,y) from the tag estimator.
        # Fallback: map-position creep when the tag is not visible.
        if self.dock_to_tag():
            self.get_logger().info('Place dock complete via AprilTag ✓')
        else:
            self.get_logger().warn('Tag dock failed — falling back to map-position dock')
            if not self._dock_by_map_pos():
                self.get_logger().error('Map-position dock also failed — aborting place')
                return False

        self.get_logger().info(
            f'Place dock ready — arm target locked {self._latched_drop}')
        return True

    # ── Place: lower box onto table and release ───────────────────────────────

    def place_box(self, level: int = 0):
        """Move arm to place the held box on the place table at the given stack level.

        Sequence: ready → hover above table → descend to surface → release → lift → home.
        Arm target (px, py) = self._latched_drop set by navigate_to_place_table().
        Heights use the same calibrated stack_surface_z as the grasp path.
        """
        self.get_logger().info(f'=== Place Step 2: Place box at level {level} ===')

        # Ensure MoveIt is alive after the navigation pause
        if not self._wait_for_move_group():
            self.get_logger().error('MoveIt not ready — dropping box in place')
            self.attach(False)
            self.set_gripper(GRIPPER_OPEN, 'release')
            return False

        px, py = self._latched_drop if self._latched_drop is not None else (0.22, 0.0)
        px = min(px, self._stop_distance)   # arm reach ceiling (config/scene.yaml: robot.arm_reach)
        q  = TOPDOWN_QUAT   # straight down, same as grasp

        # The PLACE table's surface height, scene-derived (config/scene.yaml via
        # nav_pick.launch.py). NOT self._table_top_base_z -- that is set only
        # during PICKUP docking from the pickup table's perceived height, and
        # the two tables differ by 4 cm in world height. Reusing it here was
        # the bug that released the box ~6 cm above an 8x8 cm table. See the
        # declare_parameter comment above for the incident.
        surface_z = self._place_surface_base_z
        self.get_logger().info(f'[place] surface_z={surface_z:.4f} (scene-derived)')
        rest_surface = surface_z + level * BOX_HEIGHT
        hover_z      = rest_surface + BOX_HALF_H + STACK_HOVER
        place_z      = rest_surface + BOX_HALF_H + GRASP_ABOVE

        self.get_logger().info(
            f'Place arm: x={px} y={py} hover_z={hover_z:.3f} place_z={place_z:.3f}')

        # ── go to ready pose first so the arm starts from a known configuration ──
        if not self.go_named('ready'):
            self.get_logger().warn('[place] ready failed — continuing anyway')

        # ── hover above the table surface ────────────────────────────────────────
        # Lateral fallbacks: the dominant cause of hover failure is a large |py|.
        # Nav2's xy_goal_tolerance is 0.25 m and a differential base cannot strafe,
        # so the robot routinely ends up several cm off the table centre line. At
        # full forward reach that sideways component pushes the pose out of the
        # workspace. Pulling py toward the centre line trades lateral placement
        # accuracy for a pose that can actually be reached.
        hovered = False
        for py_try in (py, py * 0.6, py * 0.3):
            if self.go_pose(px, py_try, hover_z, q, f'place hover L{level}'):
                if abs(py_try - py) > 1e-6:
                    self.get_logger().warn(
                        f'[place] hover unreachable at y={py:+.3f}; '
                        f'placing at y={py_try:+.3f} instead '
                        f'({abs(py - py_try) * 100:.1f} cm lateral error)')
                py = py_try
                hovered = True
                break
        if not hovered:
            # NEVER open the gripper here. The arm is at an unknown pose, not over
            # the table — releasing throws the box wherever the hand happens to be.
            # A run once dropped it from 0.504 m, 34 cm from target. A failed place
            # must stay a failed place, not become a destroyed trial.
            self.get_logger().error(
                f'[place] hover unreachable at x={px:.3f} y={py:+.3f} even after '
                f'lateral fallbacks — aborting the place WITHOUT releasing. '
                f'The box stays in the gripper.')
            self.go_named('ready')
            return False

        # ── descend to place height ───────────────────────────────────────────────
        # Straight down, same reason as the grasp descent — and more so once there is
        # a box already on the table: an arced approach sweeps the held box sideways
        # into the stack it is meant to land on.
        # Single-branch IK first, same as the pick descend. The place descend was left
        # on the plain Cartesian path until 2026-08-25, which is why the j2/j3 swing
        # kept appearing on the PLACE side after the pick side was fixed.
        if (self.go_pose_branch(px, py, place_z, q, f'place set L{level}') is not True
                and not self.go_pose_straight(px, py, place_z, q, f'place set L{level}')):
            # Unlike the hover failure above, here the arm IS over the table --
            # the hover pose succeeded. Releasing drops the box a few centimetres
            # onto the target rather than throwing it across the room, so this is
            # a degraded placement, not a lost one. Recorded as such.
            self.get_logger().warn(
                f'[place] descent IK failed at z={place_z:.3f} — releasing from '
                f'hover z={hover_z:.3f} ({(hover_z - place_z) * 100:.1f} cm drop). '
                f'Degraded placement: the box lands on target but from height.')
            self.attach(False)
            self.set_gripper(GRIPPER_OPEN, 'release')
            return False

        # Release: weld first so physics takes over, then open fingers
        self.attach(False)
        time.sleep(0.2)
        self.set_gripper(GRIPPER_OPEN, 'release')
        time.sleep(0.5)   # let box settle

        # Lift clear of the placed box — straight up, so the retreating gripper does
        # not sweep the box it has just released off the table.
        for attempt in range(3):
            lbl = f'place retract L{level} (try {attempt + 1})'
            # This is the move that was still swinging after the box was released:
            # it starts at the place pose and retreats straight up through the same
            # ill-conditioned region, so it hits the identical branch-flip mechanism.
            if self.go_pose_branch(px, py, hover_z, q, lbl) is True:
                break
            if self.go_pose_straight(px, py, hover_z, q, lbl):
                break
            time.sleep(0.3)

        self.go_named('home')
        self.get_logger().info(f'=== Box placed at level {level} ✓ ===')
        return True

    # ── Main sequence ─────────────────────────────────────────────────────────

    def calib_loop(self, base, n):
        """Repeat the grasp N times, auto-resetting the box between picks and reading the
        grasp offset from LIVE ROS params each iteration — so the offset can be tuned
        without relaunching anything:
            ros2 param set /nav_pick_orchestrator grasp_off_y <metres>
            ros2 param set /nav_pick_orchestrator grasp_off_x <metres>
        """
        self.get_logger().info(f'═══ CALIBRATION MODE — {n} grasp iterations ═══')
        self.get_logger().info('tune live, e.g.:  '
                               'ros2 param set /nav_pick_orchestrator grasp_off_y 0.0')
        if not self._wait_for_move_group():
            return
        pause = self.get_parameter('calib_pause').value
        for i in range(n):
            self.reset_box()
            time.sleep(0.7)   # let the box settle on the table
            ox, oy, oz = self._offset()
            bx, by, bz = base[0] + ox, base[1] + oy, base[2] + oz
            self.get_logger().info(
                f'── iter {i + 1}/{n}: base {tuple(round(v, 3) for v in base)} + '
                f'offset ({ox:.3f}, {oy:.3f}, {oz:.3f}) → grasp ({bx:.3f}, {by:.3f}, {bz:.3f}) ──')
            self.grasp_and_retract(bx, by, bz)
            self.get_logger().info(
                f'iter {i + 1} done — look at gripper vs box, then adjust the offset param')
            time.sleep(pause)
        self.get_logger().info('═══ calibration loop complete ═══')

    def run_stack(self, base, n):
        """Pick `n` boxes (auto-resetting the box at the pickup spot between picks) and
        stack them vertically on the hardcoded foundation. Assumes the robot is already
        docked and pinned. Picks reuse the same dock geometry as calib_loop, so the box
        must respawn at the same pickup location each iteration (box_reset_xyz)."""
        self.get_logger().info(f'═══ STACKING MODE — {n} boxes ═══')
        for level in range(n):
            if level > 0:
                self.reset_box()        # respawn the next box at the pickup spot
                time.sleep(0.7)         # let it settle on the table
            ox, oy, oz = self._offset()
            bx, by, bz = base[0] + ox, base[1] + oy, base[2] + oz
            self.get_logger().info(
                f'── box {level + 1}/{n}: pick ({bx:.3f}, {by:.3f}, {bz:.3f}) ──')
            if not self.grasp_and_retract(bx, by, bz):
                self.get_logger().error(f'pick failed at level {level} — aborting stack')
                break
            if not self.place_on_stack(level):
                self.get_logger().error(f'place failed at level {level} — aborting stack')
                break
        self.back_up()
        self.get_logger().info('═══ stacking complete ═══')

    def _backup_from_table(self, seconds: float):
        """Reverse at BACKUP_VEL_X for `seconds` to clear whichever table we're at."""
        _tw = Twist(); _tw.linear.x = BACKUP_VEL_X
        end = time.time() + seconds
        while time.time() < end:
            self._cmd_vel_pub.publish(_tw)
            rclpy.spin_once(self, timeout_sec=0.1)
        self._cmd_vel_pub.publish(Twist())
        time.sleep(0.3)

    def run(self):
        self.get_logger().info('══════════════════════════════════════════════')
        self.get_logger().info('  Nav-Pick Orchestrator — STARTING SEQUENCE   ')
        self.get_logger().info('══════════════════════════════════════════════')

        # 0. Travel pose — wait for MoveIt first, then fold arm DOWN before driving so
        #    joint2 (horizontal hinge) doesn't act as a pendulum during navigation.
        self._wait_for_move_group()
        self.go_named('travel')

        # 1. Navigate to pickup table
        if not self.navigate_to_table():
            self.get_logger().error('Navigation failed — aborting')
            return

        time.sleep(0.5)

        # 1.25. Slide sideways to face the target box square-on. With three boxes on
        #       the table the outer ones sit 0.10 m off the staging centre line, and
        #       Phase A below would turn that into a 7.8 degree diagonal approach.
        self._align_to_target_box()

        # 1.5. Visual docking
        self.visual_docking()

        # 2. Pin base
        self.pin_base()
        time.sleep(1.0)

        # 3. Grasp target (prefer odometry-measured dock geometry)
        base = self._dock_box if self._dock_box is not None else self.get_box_xyz()
        if base is None:
            base = FIXED_BOX
            self.get_logger().warn(f'perception fallback → FIXED_BOX {FIXED_BOX}')

        # ── CALIBRATION MODE ────────────────────────────────────────────────────
        n = self.get_parameter('calib_loops').value
        if n and n > 0:
            self.calib_loop(base, n)
            return

        # ── STACKING MODE (run_stack path — local stack, no nav to place table) ──
        stack_n = self.get_parameter('stack_count').value
        if stack_n and stack_n > 0:
            self.run_stack(base, stack_n)
            return

        # ── FULL PIPELINE: pick → transport → dock → place (multi-level) ────────
        place_levels = max(1, int(self.get_parameter('place_stack_levels').value))

        # All three boxes are back in final_map.world as of 2026-08-25 (single-box
        # grasping is now reliable: three consecutive runs with clean single-branch
        # IK and no Cartesian fallback).
        #
        # Order here is NOT the pick order. _claim_held_box() identifies which box is
        # actually held by GROUND TRUTH -- whichever one is elevated and nearest the
        # gripper -- so this list only has to name every box that exists. The pick
        # order is decided by box_pose_estimator's target_policy ('rightmost' for
        # multi-box, set in nav_pick.launch.py), which is deterministic: as boxes are
        # consumed the remaining ones present a new rightmost each cycle.
        self._remaining_boxes = ['stack_box_0', 'stack_box_1', 'stack_box_2']

        # Level 0: first pick (robot is already docked at pickup table)
        self._cycle_begin(0)
        ox, oy, oz = self._offset()
        bx, by, bz = base[0] + ox, base[1] + oy, base[2] + oz
        self.get_logger().info(
            f'grasp target = {tuple(round(v, 3) for v in base)} + offset '
            f'({ox:.3f}, {oy:.3f}, {oz:.3f}) → ({bx:.3f}, {by:.3f}, {bz:.3f})')
        self.grasp_and_retract(bx, by, bz)
        self._current_cycle['pick_success'] = self._weld_active

        if not self._weld_active:
            self.get_logger().warn('Weld not active after grasp — skipping place step')
            self._cycle_end()
            self.flush_metrics()
            self.back_up()
            return

        self._current_cycle['box_name'] = self._claim_held_box()

        # Transport: unpin, travel pose, clear pickup table
        self.unpin_base()
        self.go_named('travel')
        self.get_logger().info('Clearing pickup table — reversing 0.6 m before Nav2…')
        self._backup_from_table(4.0)

        # Navigate to place table and dock (first time — sets _latched_drop)
        nav_ok = self.navigate_to_place_table()
        result_str = f'OK  drop={self._latched_drop}' if nav_ok else 'FAILED'
        self.get_logger().info(f'navigate_to_place_table → {result_str}')
        self._current_cycle['dock_via_tag'] = (nav_ok and self._latched_drop is not None)
        # Sample at ARRIVAL (not departure) — honest measure of whether the grasp
        # actually survived transport. Under physics grasp this RE-MEASURES ground
        # truth rather than reading a latched flag, because a real joint can break.
        self._current_cycle['transport_retained'] = self._check_transport_retained()

        if not nav_ok:
            self.get_logger().error('Place dock failed — aborting')
            self._cycle_end()
            self.flush_metrics()
            self.back_up()
            return

        self.pin_base()

        # Place + multi-level stacking loop
        for lvl in range(place_levels):
            self.get_logger().info(f'Placing level {lvl}…')
            place_ok = self.place_box(level=lvl)

            # Measure where the box actually landed (Gazebo ground truth, sim only)
            # BEFORE deciding success — place_box() returning True only means the
            # arm's motion plan executed; it says nothing about where the box ended
            # up. Bug fixed 2026-08-04: place_success read True on runs where the
            # box bounced off the table and landed on the floor (see the
            # place_surface_base_z fix above for the release-height root cause).
            stack_top_z = self._place_table_top_z_world + lvl * BOX_HEIGHT + BOX_HALF_H
            self._cycle_measure_placement(self._table_map_x, self._table_map_y, stack_top_z,
                                          box_name=self._current_cycle['box_name'])

            if place_ok:
                ex = self._current_cycle.get('placement_err_x')
                ey = self._current_cycle.get('placement_err_y')
                ez = self._current_cycle.get('placement_err_z')
                if ex is not None and ey is not None:
                    err_xy = math.hypot(ex, ey)
                    # Table half-side is 0.04 m (scene.yaml place_table.side/2);
                    # 0.03 m leaves a small margin for a box still counted "placed"
                    # near the edge, while catching anything that actually missed.
                    PLACE_XY_TOL, PLACE_Z_TOL = 0.03, 0.03
                    if err_xy > PLACE_XY_TOL or (ez is not None and abs(ez) > PLACE_Z_TOL):
                        self.get_logger().error(
                            f'[place] arm motion completed but the box MISSED the '
                            f'table: err_xy={err_xy:.3f} m (tol {PLACE_XY_TOL}), '
                            f'err_z={ez} — marking place_success=False')
                        place_ok = False
                else:
                    self.get_logger().warn(
                        '[place] no ground truth available — cannot verify placement; '
                        'trusting the mechanical result (real-hardware behaviour)')

            self._current_cycle['place_success'] = place_ok
            self._cycle_end()

            if not place_ok:
                self.get_logger().error(f'place_box failed at level {lvl} — stopping stack')
                break

            if lvl + 1 >= place_levels:
                break   # all levels done

            if not self._remaining_boxes:
                self.get_logger().warn('No boxes left to fetch — stopping stack')
                break

            # ── Fetch next box ───────────────────────────────────────────────
            self._cycle_begin(lvl + 1)
            self.get_logger().info(f'Fetching box for level {lvl + 1}…')
            self.unpin_base()
            self.go_named('travel')
            self._backup_from_table(3.0)

            self._activate_nav2_cmdvel()
            if not self.navigate_to_table():
                self.get_logger().error('Re-navigation to pickup failed — stopping stack')
                self._current_cycle['pick_success'] = False
                self._cycle_end()
                break

            # Same alignment as the first pick. This matters MORE here: each level
            # fetches a different box, and after the first is removed the remaining
            # ones are the off-centre ones.
            self._align_to_target_box()
            self.visual_docking()
            self.pin_base()
            time.sleep(1.0)

            base2 = self._dock_box if self._dock_box else FIXED_BOX
            ox2, oy2, oz2 = self._offset()
            bx2, by2, bz2 = base2[0] + ox2, base2[1] + oy2, base2[2] + oz2
            self.grasp_and_retract(bx2, by2, bz2)
            self._current_cycle['pick_success'] = self._weld_active

            if not self._weld_active:
                self.get_logger().error('Pick failed for next level — stopping stack')
                self._cycle_end()
                break

            self._current_cycle['box_name'] = self._claim_held_box()

            self.unpin_base()
            self.go_named('travel')
            self._backup_from_table(4.0)

            nav_ok2 = self.navigate_to_place_table()
            self._current_cycle['transport_retained'] = self._check_transport_retained()
            self._current_cycle['dock_via_tag'] = nav_ok2 and self._latched_drop is not None
            if not nav_ok2:
                self.get_logger().error('Re-nav to place table failed — stopping stack')
                self._cycle_end()
                break
            self.pin_base()

        # 6. Back up and write metrics
        # place_box() leaves the arm at 'home' (all joints zero — the pose the
        # 'travel' comment elsewhere calls the pendulum "worst case"), not a
        # driving-safe pose. Every other transition in this file goes to 'travel'
        # before moving the base; this one didn't, so the robot could clip the
        # just-placed box/table while backing out. Match the existing pattern.
        self.go_named('travel')
        self.back_up()
        self.flush_metrics()

        self.get_logger().info('══════════════════════════════════════════════')
        self.get_logger().info('  SEQUENCE COMPLETE ✓                         ')
        self.get_logger().info('══════════════════════════════════════════════')


def main(args=None):
    rclpy.init(args=args)
    node = NavPickOrchestrator()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.unpin_base()
        node.flush_metrics()   # write CSV even on Ctrl-C
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

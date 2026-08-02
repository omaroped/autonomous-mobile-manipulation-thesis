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

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import (PoseStamped, Pose, Point, PointStamped,
                                PoseWithCovarianceStamped, Twist)
from std_msgs.msg import Float64MultiArray, Bool, Float64
from action_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry

from nav2_msgs.action import NavigateToPose

from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import ApplyPlanningScene
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
# LAG_THRESH/STALL_STEPS were tuned to stop the instant EITHER finger touches — but
# only one side (gripper_controller, the left reference joint) is actually monitored,
# and the real gripper has a single motor driving both sides symmetrically anyway.
# With a light, freely-sliding box, a finger touching first should be allowed to push
# the box sideways rather than halting the whole close — only a real, sustained
# both-sides-blocked stall should count. Loosened accordingly (2026-07-26): more lag
# tolerated, and it must persist much longer before being accepted as a real stall.
LAG_THRESH        = 0.08     # rad: actual lags commanded by this → stall onset (was 0.03)
STALL_STEPS       = 10       # consecutive stall detections to confirm (was 3, ~0.24s → ~0.8s)
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
# dock_range + table_side/2 (0.09 m) must land <= STOP_DISTANCE (0.24 m) so the
# arm reaches the true table centre without the px=min(px, STOP_DISTANCE) clamp
# in place_box() silently shifting the drop point off-centre. 0.15+0.09=0.24 exactly.
PLACE_DOCK_RANGE   = 0.15   # stop when tag is this far from base_link (m)
# Map-pos approach: drive until map_y ≤ this value.
# Table centre at map (-4,-2).  Want table at x≈0.25 in base_link.
# Robot at map_y = -2.0 + 0.25 = -1.75  →  PLACE_MAP_Y_DOCK = -1.75
PLACE_MAP_Y_DOCK   = -1.75   # map y target for the primary dock approach
TABLE_MAP_X        = -4.0    # known table centre map X (used for lateral correction)
TABLE_MAP_Y        = -2.0    # known table centre map Y
PLACE_DOCK_YAW_TOL = 0.04   # Phase A: heading tight enough to start Phase C (rad ≈ 2.3°)
PLACE_DOCK_BEAR_TOL= 0.04   # Phase A: tag bearing tolerance (rad)
PLACE_DOCK_K_ROT   = 2.0    # Phase A/C rotation gain (rad/s per rad error)
PLACE_DOCK_K_FWD   = 0.8    # Phase C forward gain (m/s per m range error)
PLACE_DOCK_MAX_ROT = 0.40   # max rotation speed (rad/s)
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
MOVEIT_SUCCESS = 1

# ── Base pin ──────────────────────────────────────────────────────────────────
PIN_ROBOT_NAME = 'mbot'


class NavPickOrchestrator(Node):

    def __init__(self):
        super().__init__('nav_pick_orchestrator')

        # Nav2
        self._nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # AMCL initial pose publisher
        self._init_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # MoveIt
        self._move  = ActionClient(self, MoveGroup, '/move_action')
        self._scene = self.create_client(ApplyPlanningScene, '/apply_planning_scene')

        # Gripper
        self._gripper      = self.create_publisher(Float64MultiArray, GRIPPER_TOPIC, 10)
        self._attach_pub   = self.create_publisher(Bool, '/grasp_attach', 10)
        self._last_gripper   = [0.0] * len(GRIPPER_OPEN)   # 1 joint (mimic followers)
        self._weld_active    = False
        self._gripper_actual = GRIPPER_OPEN[0]   # actual position from /joint_states
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

        # Which grasp mechanism is running — see the USE_WELD comment block at the top.
        # Must match what nav_pick.launch.py started; the launch file passes the same
        # value it used to gate grasp_attacher/smart_grasp.
        self.declare_parameter('use_physics_grasp', True)
        self._physics_grasp = bool(self.get_parameter('use_physics_grasp').value)
        self.get_logger().info(
            f'grasp mechanism = '
            f'{"PHYSICS (libgazebo_grasp_plugin real fixed joint)" if self._physics_grasp else "LEGACY kinematic weld (grasp_attacher)"}')

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
        self.declare_parameter('box_reset_xyz', [-2.0, 4.0, 0.16])

        # ── Stacking knobs (live, tune with `ros2 param set` — NO relaunch) ──────
        self.declare_parameter('stack_count', STACK_COUNT_DEF)   # >0 → stack this many boxes
        self.declare_parameter('stack_x', STACK_X_DEFAULT)       # base_link x of foundation
        self.declare_parameter('stack_y', STACK_Y_DEFAULT)       # base_link y of foundation
        self.declare_parameter('stack_surface_z', STACK_SURFACE_Z)  # base_link z of surface top

        # ── Tag-dock knobs (live, tune with `ros2 param set` — NO relaunch) ────
        self.declare_parameter('dock_range',      PLACE_DOCK_RANGE)    # tag distance at stop
        self.declare_parameter('place_stack_levels', 1)                 # boxes to stack at place table
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

    def _publish_initial_pose(self):
        """Tell AMCL where the robot starts (spawn pose)."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = -2.0
        msg.pose.pose.position.y =  7.0
        msg.pose.pose.position.z =  0.0
        # yaw −π/2 → quaternion (0, 0, sin(−π/4), cos(−π/4)) = (0, 0, −0.7071, 0.7071)
        msg.pose.pose.orientation.z = -0.7071
        msg.pose.pose.orientation.w =  0.7071
        # diagonal covariance — position known within ±0.5 m, yaw within ±0.2 rad
        msg.pose.covariance[0]  = 0.25
        msg.pose.covariance[7]  = 0.25
        msg.pose.covariance[35] = 0.04

        # Wait for AMCL to subscribe to /initialpose
        self.get_logger().info('Waiting for AMCL to subscribe to /initialpose...')
        for _ in range(30):
            if self._init_pose_pub.get_subscription_count() > 0:
                self.get_logger().info('AMCL subscribed! Publishing initial pose...')
                break
            time.sleep(0.5)
            # Spin to allow discovery to proceed
            rclpy.spin_once(self, timeout_sec=0.1)

        for _ in range(10):    # publish several times so AMCL receives it
            msg.header.stamp = self.get_clock().now().to_msg()
            self._init_pose_pub.publish(msg)
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().info('published AMCL initial pose at (−2, 7, yaw=−π/2)')

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
        self._publish_initial_pose()
        time.sleep(1.0)   # give AMCL a moment to digest the initial pose

        if not self._nav.wait_for_server(timeout_sec=30.0):
            self.get_logger().error(
                'NavigateToPose action server not available — '
                'is nav2_limo.launch.py running?')
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(NAV_GOAL_X)
        goal.pose.pose.position.y = float(NAV_GOAL_Y)
        # Yaw NAV_GOAL_YAW → quaternion
        half = NAV_GOAL_YAW / 2.0
        goal.pose.pose.orientation.z = math.sin(half)
        goal.pose.pose.orientation.w = math.cos(half)

        self.get_logger().info(
            f'sending Nav2 goal → ({NAV_GOAL_X:.2f}, {NAV_GOAL_Y:.2f}, '
            f'yaw={NAV_GOAL_YAW:.3f})')

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
        if z_before is None:
            return None
        z_now = self._box_world_z()
        if z_now is None:
            return None
        rise = z_now - z_before
        ok = rise >= min_rise
        self.get_logger().info(
            f'[verify] GROUND TRUTH: box world z {z_before:.3f} → {z_now:.3f} '
            f'(rise {rise:+.3f} m, need ≥{min_rise:.3f}) → '
            f'{"LIFTED (grasped)" if ok else "DID NOT MOVE (failed)"}')
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
        """
        if drive <= 0.001:
            return
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
            t = Twist(); t.angular.z = clamp(K_ROT * y, -MAX_ROT, MAX_ROT)
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
            self._dock_box = (STOP_DISTANCE, box0[1], box0[2])
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
            self._dock_box = (STOP_DISTANCE, final[1], final[2])
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
        stage2 = max(0.0, FINAL_READ_DIST - STOP_DISTANCE)
        self.get_logger().info(
            f'Phase B stage 3: driving the last {stage2:.3f} m blind to '
            f'{STOP_DISTANCE:.2f} m (the arm reach limit)')
        self._drive_forward(stage2, FINAL_READ_DIST)

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
                self._dock_box = (STOP_DISTANCE, y_new, z_keep)
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
        floor   = GRIPPER_GRASP[0]    # -0.20 rad hard limit
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

            rclpy.spin_once(self, timeout_sec=CLOSE_STEP_SEC)

            # smart_grasp (bumper or joint-error) fired the weld
            if self._weld_active:
                stall_angle = self._gripper_actual
                self.get_logger().info(
                    f'[close] smart_grasp weld  actual={stall_angle:.3f}')
                break

            actual = self._gripper_actual

            if not gripper_tracking and abs(actual - start_actual) >= TRACKING_RAD:
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
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=30.0)
        if not res_fut.done():
            self.get_logger().error(f'[{label}] no result within 30 s')
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
        HOVER      = 0.06
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
        hovered = False
        for clearance in (HOVER, 0.045, 0.03):
            if self.go_pose(tx, ty, tz + clearance, q, f'hover {clearance:.3f} m'):
                hovered = True
                if clearance < HOVER:
                    self.get_logger().warn(
                        f'hover reduced to {clearance:.3f} m — headroom is tight at this '
                        f'table height and dock distance')
                break
        if not hovered:
            self.get_logger().error('no reachable hover above the box — aborting')
            return False

        # ── descend straight down to the grasp point ────────────────────────
        if not self.go_pose(tx, ty, tz + GRASP_Z, q, 'grasp'):
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
                if not self.go_pose(tx, ty, tz + HOVER, q, 'hover for retry'):
                    break
                if not self.go_pose(tx, ty, tz + GRASP_Z, q, f'grasp retry {attempt - 1}'):
                    break

            # True box height BEFORE the close — the reference for the ground-truth
            # lift check below.
            z_before = self._box_world_z()

            result = self.close_until_contact()

            # Nothing was between the fingers — no point lifting or verifying.
            if result == GraspResult.AIR:
                self.get_logger().warn(
                    '[grasp] AIR — gripper closed on nothing, skipping lift/verify')
                verified = False
                self._log_grasp_attempt(attempt, result, verified)
                continue

            # Lift — retry 3× (OMPL can be flaky on the first attempt)
            lifted = False
            for clearance in (HOVER, 0.045, 0.03):
                if self.go_pose(tx, ty, tz + clearance, q, f'lift {clearance:.3f} m'):
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
        if not self.go_pose(sx, sy, place_z, q, f'stack place L{level}'):
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
            if self.go_pose(sx, sy, hover_z, q, f'stack retract L{level} (try {attempt + 1})'):
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

    def dock_to_tag(self):
        """Closed-loop dock to the AprilTag on the place table face.

        Phase A — face (load-bearing): rotate in place, closed-loop on the tag's
                  bearing AND yaw until the chassis is squared to the face.
                  This must be tight: a straight approach amplifies heading error
                  into lateral offset, and the arm must absorb any residual.
        Phase C — straight approach: drive forward with yaw trims keeping the tag
                  centred, until tag range ≤ dock_range.
        Phase D — latch: record drop (x,y) and tag_yaw for the arm.

        Returns True on success, False on timeout/no-tag.
        """
        self.get_logger().info('=== Place Step 1b: dock_to_tag ===')
        dock_range = float(self.get_parameter('dock_range').value)

        def clamp(v, lo, hi):
            return max(lo, min(hi, v))

        def wrap(a):
            return (a + math.pi) % (2 * math.pi) - math.pi

        # ── Phase A: rotate until the chassis is squared to the tag FACE ────────
        # Squaring on yaw (not bearing) is the actual "load-bearing" alignment: the
        # tag_dock_estimator convention has the tag's Z axis point out of the face
        # toward the robot, so a squarely-facing robot sees tag yaw ≈ π in base_link
        # (heading_err = wrap(yaw-π) → 0). Bearing alone only centres the tag ahead —
        # if the robot isn't exactly on the face-normal line (Nav2 tolerance), that
        # leaves the chassis angled to the face, which Phase C's straight approach
        # then amplifies into lateral placement error. Phase C's own bearing-based
        # trim still corrects small residual drift while driving in.
        # 8 s timeout. tag_seen tracks whether ANY reading arrived — if the tag was
        # never visible at all from the Nav2 stop (too far / wrong angle), we skip
        # Phase C immediately instead of burning another 30 s printing "tag lost".
        self.get_logger().info('Phase A: squaring to tag face…')
        ok_count = 0
        tag_seen  = False
        deadline = time.time() + 8.0
        while time.time() < deadline:
            t = self._read_tag_live()
            if t is None:
                self._cmd_vel_pub.publish(Twist())
                self.get_logger().info('Phase A: no tag — stopping', throttle_duration_sec=1.0)
                time.sleep(0.1)
                continue
            tag_seen = True
            _, _, tag_range, bearing, yaw = t
            heading_err = wrap(yaw - math.pi)
            err = heading_err
            self.get_logger().info(
                f'Phase A: bearing={math.degrees(bearing):+.1f}° '
                f'heading_err={math.degrees(heading_err):+.1f}° range={tag_range:.3f}m',
                throttle_duration_sec=0.5)
            if abs(err) < PLACE_DOCK_YAW_TOL:
                ok_count += 1
                if ok_count >= 4:
                    self.get_logger().info('Phase A: aligned ✓')
                    break
            else:
                ok_count = 0
            cmd = Twist()
            cmd.angular.z = clamp(PLACE_DOCK_K_ROT * err, -PLACE_DOCK_MAX_ROT, PLACE_DOCK_MAX_ROT)
            self._cmd_vel_pub.publish(cmd)
            time.sleep(0.05)
        self._cmd_vel_pub.publish(Twist())
        time.sleep(0.3)

        if not tag_seen:
            # Tag never appeared — skip Phase C (would just print "tag lost" for 30 s)
            # and fall straight to Phase D, which will return False → map-pos fallback.
            self.get_logger().warn(
                'Phase A: tag never visible — skipping Phase C, going direct to Phase D')

        # ── Phase C: straight approach until dock_range (skipped if tag never seen) ──
        if tag_seen:
            self.get_logger().info(f'Phase C: approaching to {dock_range:.2f} m…')
            deadline = time.time() + 30.0
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
        self._latched_drop    = (float(drop.point.x), float(drop.point.y))
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
            f'map-pos dock: driving south to map_y ≤ {PLACE_MAP_Y_DOCK}')

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
            remaining = robot_y - PLACE_MAP_Y_DOCK   # positive → still heading south
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
            dx = TABLE_MAP_X - rx
            dy = TABLE_MAP_Y - ry

            # Rotate into base_link:  forward = (cos yaw, sin yaw), left = (-sin yaw, cos yaw)
            table_x =  dx * math.cos(yaw) + dy * math.sin(yaw)
            table_y = -dx * math.sin(yaw) + dy * math.cos(yaw)

            self.get_logger().info(
                f'map-dock settled: robot=({rx:.3f},{ry:.3f}) yaw={math.degrees(yaw):.1f}° '
                f'→ table in base_link=({table_x:.3f},{table_y:.3f})')

        except Exception as e:
            # Pure-geometry fallback if TF fails
            table_x = abs(TABLE_MAP_Y - PLACE_MAP_Y_DOCK)
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
        goal.pose.pose.position.x = float(NAV_PLACE_X)
        goal.pose.pose.position.y = float(NAV_PLACE_Y)
        half = NAV_PLACE_YAW / 2.0
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
        px = min(px, STOP_DISTANCE)   # tag centre is 0.28 m out; arm reaches reliably to 0.24
        q  = TOPDOWN_QUAT   # straight down, same as grasp

        # Prefer the perception-derived table top (captured at pickup dock).
        # Falls back to the live param if perception never ran (e.g. calib mode).
        param_z = float(self.get_parameter('stack_surface_z').value)
        surface_z = self._table_top_base_z if self._table_top_base_z is not None else param_z
        self.get_logger().info(
            f'[place] surface_z={surface_z:.4f} '
            f'({"perceived" if self._table_top_base_z is not None else "param fallback"})')
        rest_surface = surface_z + level * BOX_HEIGHT
        hover_z      = rest_surface + BOX_HALF_H + STACK_HOVER
        place_z      = rest_surface + BOX_HALF_H + GRASP_ABOVE

        self.get_logger().info(
            f'Place arm: x={px} y={py} hover_z={hover_z:.3f} place_z={place_z:.3f}')

        # ── go to ready pose first so the arm starts from a known configuration ──
        if not self.go_named('ready'):
            self.get_logger().warn('[place] ready failed — continuing anyway')

        # ── hover above the table surface ────────────────────────────────────────
        if not self.go_pose(px, py, hover_z, q, f'place hover L{level}'):
            self.get_logger().error('[place] hover IK failed — dropping in place')
            self.attach(False)
            self.set_gripper(GRIPPER_OPEN, 'release')
            return False

        # ── descend to place height ───────────────────────────────────────────────
        if not self.go_pose(px, py, place_z, q, f'place set L{level}'):
            self.get_logger().error('[place] descent IK failed — dropping from hover')
            self.attach(False)
            self.set_gripper(GRIPPER_OPEN, 'release')
            return False

        # Release: weld first so physics takes over, then open fingers
        self.attach(False)
        time.sleep(0.2)
        self.set_gripper(GRIPPER_OPEN, 'release')
        time.sleep(0.5)   # let box settle

        # Lift clear of the placed box
        for attempt in range(3):
            if self.go_pose(px, py, hover_z, q,
                            f'place retract L{level} (try {attempt + 1})'):
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

        # Single-box testing for now — stack_box_1/2 removed from final_map.world.
        # Revert to ['stack_box_0', 'stack_box_1', 'stack_box_2'] once one-box
        # grasping is reliable and multi-box stacking is re-enabled.
        self._remaining_boxes = ['stack_box_0']

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
            self._current_cycle['place_success'] = place_ok

            # Measure where the box actually landed (for the thesis metrics)
            stack_top_z = 0.10 + lvl * BOX_HEIGHT + BOX_HALF_H  # world z of box centre
            self._cycle_measure_placement(TABLE_MAP_X, TABLE_MAP_Y, stack_top_z,
                                          box_name=self._current_cycle['box_name'])
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

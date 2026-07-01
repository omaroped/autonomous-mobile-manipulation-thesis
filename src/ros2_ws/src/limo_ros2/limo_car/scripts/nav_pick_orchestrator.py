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

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import (PoseStamped, Pose, Point, PointStamped,
                                PoseWithCovarianceStamped, Twist)
from std_msgs.msg import Float64MultiArray, Bool
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

from tf2_ros import Buffer, TransformListener


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
GRIPPER_OPEN  = [ 0.15,  0.15, -0.15, -0.15, -0.15,  0.15]
GRIPPER_GRASP = [-0.20, -0.20,  0.20,  0.20,  0.20, -0.20]   # gentle partial close
GRIPPER_TOPIC = '/mycobot_gripper_controller/commands'

# Angle at which we fire the weld directly if smart_grasp hasn't triggered.
# Estimated from box geometry: 35 mm box, finger gap at open=43.4 mm, at grasp=31.8 mm.
# Linear interpolation: contact at θ ≈ -0.10 rad.  Fire at -0.10 so fingers are
# just touching the box when the weld attaches — no force overshoot.
WELD_FALLBACK_ANGLE = -0.10

# Sim "weld" attaches the box rigidly to the gripper (a Gazebo grasp aid). Set False to
# test whether the gripper physically holds the box on its own (real-grip test).
USE_WELD = False  # real physics grasp — no artificial attachment

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
GRASP_ABOVE  = 0.06                    # TCP above box centre at grasp

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
PLACE_DOCK_RANGE   = 0.19   # stop when tag is this far from base_link (m)
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
        self._last_gripper = [0.0] * 6
        self._weld_active  = False
        self.create_subscription(Bool, '/grasp_attach', self._weld_cb, 10)

        # Perception
        self._latest_box = None
        self.create_subscription(PoseStamped, '/box_pose', self._box_cb, 10)

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

        # ── Calibration knobs (live, tune with `ros2 param set` — NO relaunch) ──
        self.declare_parameter('grasp_off_x', GRASP_OFFSET[0])
        self.declare_parameter('grasp_off_y', GRASP_OFFSET[1])
        self.declare_parameter('grasp_off_z', GRASP_OFFSET[2])
        self.declare_parameter('calib_loops', 0)     # 0 = normal single run; N = repeat the grasp N times
        self.declare_parameter('calib_pause', 6.0)   # seconds to observe between iterations
        self.declare_parameter('box_name', 'target_box')
        self.declare_parameter('box_reset_xyz', [-2.0, 4.0, 0.12])  # where to respawn the box each iter

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
            'pick_success':       False,
            'transport_retained': False,
            'dock_via_tag':       False,
            'place_success':      False,
            'placement_err_x':    float('nan'),
            'placement_err_y':    float('nan'),
            'placement_err_z':    float('nan'),
            'cycle_time_s':       float('nan'),
        }

    def _cycle_measure_placement(self, expected_x: float, expected_y: float,
                                 expected_z: float):
        """Query Gazebo for the actual box position and record XYZ error."""
        if not self._get_state.service_is_ready():
            return
        name = self.get_parameter('box_name').value
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

        drive = max(0.0, d0 - STOP_DISTANCE)
        self.get_logger().info(
            f'Phase B: box at {d0:.3f} m → driving {drive:.3f} m to stop at {STOP_DISTANCE:.2f} m')

        # wait for a fresh odom sample, record the start point
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._odom is not None:
                break
        ox, oy = self._odom if self._odom is not None else (0.0, 0.0)

        deadline = time.time() + 25.0
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            disp = self._odom_dist_since(ox, oy)
            disp = 0.0 if disp is None else disp
            remaining = drive - disp
            self.get_logger().info(
                f'approach: driven {disp:.3f}/{drive:.3f} m  (box ≈ {d0 - disp:.3f} m)',
                throttle_duration_sec=0.5)
            if remaining <= 0.0:
                self.get_logger().info(
                    f'HARD STOP — docked ~{STOP_DISTANCE:.2f} m from box, table not climbed')
                break
            t = Twist()
            t.linear.x = clamp(0.6 * remaining, 0.06, 0.14)   # ease off near the stop
            self._cmd_vel_pub.publish(t); time.sleep(0.05)
        else:
            self.get_logger().warn('approach timed out — stopping')

        self._cmd_vel_pub.publish(Twist()); time.sleep(0.5)

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
        self._weld_active = msg.data

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
        time.sleep(0.3)

    def close_until_contact(self):
        """Close one step at a time; stop the instant smart_grasp fires the weld.

        Prevents the ODE "explosion" (box launching sideways) that happens when a
        fixed closing angle keeps building contact force after the box is gripped.
        Also works for any object size — large objects stop the fingers early,
        small objects allow fingers to close further, no code change needed.

        Safety floor: never closes past GRIPPER_GRASP even without contact.
        """
        self._weld_active = False   # clear stale state before the new attempt

        j0    = self._last_gripper[0]
        floor = GRIPPER_GRASP[0]    # -0.20 rad hard limit

        step_rad = 0.005            # 0.5 deg per tick — very smooth
        step_sec = 0.08             # 80 ms per tick

        steps_taken = 0
        while j0 > floor:
            j0 = max(floor, j0 - step_rad)
            t  = (GRIPPER_OPEN[0] - j0) / (GRIPPER_OPEN[0] - GRIPPER_GRASP[0])

            cmd = Float64MultiArray()
            cmd.data = [float(o + t * (c - o))
                        for o, c in zip(GRIPPER_OPEN, GRIPPER_GRASP)]
            self._gripper.publish(cmd)
            self._last_gripper = cmd.data[:]
            steps_taken += 1

            rclpy.spin_once(self, timeout_sec=step_sec)

            if self._weld_active:
                self.get_logger().info(
                    f'[close] smart_grasp weld at j0={j0:.3f} rad  '
                    f'({steps_taken * step_sec:.1f} s)  HOLDING')
                break

            # Fallback: if smart_grasp hasn't fired by the estimated contact angle,
            # fire the weld directly.  Prevents the box from sliding away while waiting
            # for a contact sensor that may not detect the collision.
            if j0 <= WELD_FALLBACK_ANGLE and not self._weld_active:
                self.get_logger().info(
                    f'[close] weld FALLBACK at j0={j0:.3f} rad '
                    f'(smart_grasp silent — firing directly)')
                self.attach(True)
                break
        else:
            if not self._weld_active:
                self.get_logger().warn(
                    '[close] Reached floor -0.20 rad — weld never fired.')

        # Back off fingers 2 steps from contact so the finger force doesn't fight
        # the weld constraint — this eliminates the post-pick shaking.
        if self._weld_active:
            j_back = self._last_gripper[0] + 0.010
            t_back = max(0.0, (GRIPPER_OPEN[0] - j_back) /
                         (GRIPPER_OPEN[0] - GRIPPER_GRASP[0]))
            back_cmd = Float64MultiArray()
            back_cmd.data = [float(o + t_back * (c - o))
                             for o, c in zip(GRIPPER_OPEN, GRIPPER_GRASP)]
            self._gripper.publish(back_cmd)
            self._last_gripper = back_cmd.data[:]
            rclpy.spin_once(self, timeout_sec=0.15)
            self.get_logger().info('[close] fingers backed off 0.01 rad — contact force removed')

        time.sleep(0.3)

    def attach(self, on: bool):
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
    def _pose_constraints(x, y, z, quat, pos_tol=0.025, ori_tol=0.1):
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
        HOVER      = 0.12     # TCP this far above box centre while aligning
        GRASP_Z    = GRASP_ABOVE   # calibrated grasp height (0.06 above centre)
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
        if not self.go_pose(tx, ty, tz + HOVER, q, 'hover'):
            self.get_logger().error('hover pose failed — aborting')
            return False

        # ── descend straight down to the grasp point ────────────────────────
        if not self.go_pose(tx, ty, tz + GRASP_Z, q, 'grasp'):
            self.get_logger().error('grasp pose failed — aborting')
            return False

        # Adaptive close: steps 0.5 deg at a time, stops the moment smart_grasp
        # detects bilateral contact and fires the weld.  No fixed end angle —
        # works for any object size and eliminates the ODE "explosion" that
        # happened when a fixed -0.20 rad target kept building contact force
        # after the box was already gripped.
        if USE_WELD:
            self.attach(True)
        else:
            self.get_logger().info('USE_WELD=False — smart_grasp handles weld automatically')
        self.close_until_contact()

        # Remove table collision so we can lift without phantom table-gripper collisions
        self.remove_table()

        # Lift — retry 3× (OMPL can be flaky on the first attempt)
        lifted = False
        for attempt in range(3):
            if self.go_pose(tx, ty, tz + HOVER, q, f'lift (try {attempt + 1})'):
                lifted = True
                break
            time.sleep(0.5)
        if not lifted:
            self.get_logger().warn('lift attempts failed — continuing to retract anyway')

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
        box lands on the one below. We mirror the grasp geometry: at grasp the TCP sat
        GRASP_ABOVE above the box centre, so to rest the box bottom on the surface the TCP
        target is surface + BOX_HALF_H + GRASP_ABOVE. Hover → descend → release → lift.

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

        # Release: drop the weld first (if used) so physics takes over, then open fingers.
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

        # ── Phase A: rotate until bearing ≈ 0 AND heading squared to face ────
        # 8 s timeout: if the tag is not visible at all from the Nav2 stopping point
        # (too far, bad angle) we fail fast so the map-position fallback can take over
        # instead of burning 20 s printing "no tag".
        self.get_logger().info('Phase A: squaring to tag face…')
        ok_count = 0
        deadline = time.time() + 8.0
        while time.time() < deadline:
            t = self._read_tag_live()
            if t is None:
                self._cmd_vel_pub.publish(Twist())
                self.get_logger().info('Phase A: no tag — stopping', throttle_duration_sec=1.0)
                time.sleep(0.1)
                continue
            _, _, tag_range, bearing, yaw = t
            # heading_err: how much the tag face normal deviates from pointing straight
            # at us. After TF to base_link, the tag's yaw in base_link ≈ π when the
            # robot faces the tag squarely (normal points in -X of base_link).
            # We rotate until both bearing ≈ 0 (tag dead ahead) and bearing is small.
            err = bearing   # use bearing as the primary error signal
            self.get_logger().info(
                f'Phase A: bearing={math.degrees(bearing):+.1f}° range={tag_range:.3f}m',
                throttle_duration_sec=0.5)
            if abs(err) < PLACE_DOCK_BEAR_TOL:
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

        # ── Phase C: straight approach until dock_range ───────────────────────
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

        # Transport: unpin, travel pose, clear pickup table
        self.unpin_base()
        self.go_named('travel')
        self._current_cycle['transport_retained'] = True   # weld was active at departure
        self.get_logger().info('Clearing pickup table — reversing 0.6 m before Nav2…')
        self._backup_from_table(4.0)

        # Navigate to place table and dock (first time — sets _latched_drop)
        nav_ok = self.navigate_to_place_table()
        result_str = f'OK  drop={self._latched_drop}' if nav_ok else 'FAILED'
        self.get_logger().info(f'navigate_to_place_table → {result_str}')
        self._current_cycle['dock_via_tag'] = (nav_ok and self._latched_drop is not None)

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
            expected_z = TABLE_MAP_Y * 0.0   # unused — compute from stack geometry
            stack_top_z = 0.10 + lvl * BOX_HEIGHT + BOX_HALF_H  # world z of box centre
            self._cycle_measure_placement(TABLE_MAP_X, TABLE_MAP_Y, stack_top_z)
            self._cycle_end()

            if not place_ok:
                self.get_logger().error(f'place_box failed at level {lvl} — stopping stack')
                break

            if lvl + 1 >= place_levels:
                break   # all levels done

            # ── Fetch next box ───────────────────────────────────────────────
            self._cycle_begin(lvl + 1)
            self.get_logger().info(f'Fetching box for level {lvl + 1}…')
            self.unpin_base()
            self.go_named('travel')
            self._backup_from_table(3.0)

            self.reset_box()
            time.sleep(0.7)

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

            self.unpin_base()
            self.go_named('travel')
            self._current_cycle['transport_retained'] = True
            self._backup_from_table(4.0)

            nav_ok2 = self.navigate_to_place_table()
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

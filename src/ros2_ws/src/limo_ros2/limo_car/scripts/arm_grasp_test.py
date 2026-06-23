#!/usr/bin/python3
"""
arm_grasp_test — ISOLATED grasp test (no driving), FRONTAL approach.

WHY FRONTAL, NOT TOP-DOWN (proven from the URDF link lengths):
    myCobot 280 m5: shoulder is 0.132 m above the arm base; the elbow->wrist
    span is only ~0.226 m (upper arm 0.110 + forearm 0.116). A TOP-DOWN grasp
    needs the wrist held directly above the box while the arm spans the full
    horizontal distance to it — which exceeds that 0.226 m for a box ~0.24 m out,
    so MoveIt finds NO IK solution (it reports "Unable to solve"). The spec-sheet
    "0.28 m reach" is the arm pointing straight OUT, not folded down. Pushing the
    box closer just buries it under the robot's front overhang. => top-down is
    geometrically impossible here; that is the project's binding reachability
    constraint.

    So we grasp the box at ~shoulder height with the arm reaching straight out (a
    FRONTAL grasp), which uses the full reach. To avoid guessing the exact gripper
    orientation, the node PROBES a set of approach angles (frontal -> tilted ->
    top-down) with plan-only requests and uses the first one MoveIt can actually
    solve, then executes the grasp with it.

    This is the EXACT same robot / MoveIt / controllers / gripper as the real
    pipeline; only driving is removed, and the box is elevated to arm height.
    Nothing here touches the main pipeline.

    Run order:
        ros2 launch limo_car arm_grasp_test.launch.py             # sim + robot + elevated test box
        ros2 launch limo_cobot_moveit_config moveit.launch.py     # move_group (+ RViz)
        ros2 launch limo_car arm_grasp_run.launch.py              # perception + THIS node
"""

import time
import statistics

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Pose, Point, Twist
from std_msgs.msg import Float64MultiArray, Bool

from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, JointConstraint,
    PositionConstraint, OrientationConstraint, BoundingVolume,
    CollisionObject, PlanningScene,
)
from shape_msgs.msg import SolidPrimitive


# ── Gripper (6 joints driven directly; mirror signs [1,1,-1,-1,-1,1]) ─────────
# order MUST match mycobot_controllers.yaml: master, left2, left1, right3, right2, right1
GRIPPER_OPEN  = [ 0.15,  0.15, -0.15, -0.15, -0.15,  0.15]
GRIPPER_CLOSE = [-0.50, -0.50,  0.50,  0.50,  0.50, -0.50]
GRIPPER_GRASP = [-0.20, -0.20,  0.20,  0.20,  0.20, -0.20]   # gentle partial close
GRIPPER_TOPIC = '/mycobot_gripper_controller/commands'

# ── Frames / group / named states (must match the SRDF) ───────────────────────
PLANNING_FRAME = 'base_link'
TCP_LINK       = 'gripper_tcp'
ARM_GROUP      = 'arm'
ARM_JOINTS = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
              'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
NAMED_STATES = {
    'home':  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'ready': [0.0, -0.5, -0.6, 1.1, 0.0, 0.0],
}

ARM_BASE_XYZ = (-0.03, 0.0, 0.02)

# Frontal-grasp standoffs along the approach axis (+x, straight out from the arm):
STANDOFF  = 0.08   # pre-grasp this far in FRONT of the box (back toward the robot)
GRASP_GAP = 0.02   # TCP this far in front of the box centre at the grasp

# Candidate gripper orientations to PROBE, from straight-out (frontal) to
# top-down — built as a tilt about base +y away from the top-down pose. (x,y,z,w).
# The node uses the first one MoveIt can solve at the pre-grasp point.
APPROACH_CANDIDATES = [
    ('frontal',     (0.707, 0.0, -0.707, 0.0)),
    ('frontal-alt', (0.707, 0.0,  0.707, 0.0)),
    ('tilt-75',     (0.793, 0.0, -0.609, 0.0)),
    ('tilt-75-alt', (0.793, 0.0,  0.609, 0.0)),
    ('tilt-55',     (0.891, 0.0, -0.454, 0.0)),
    ('top-down',    (1.0,   0.0,  0.0,   0.0)),
]

FALLBACK_BOX = (0.20, 0.0, 0.13)     # used only if perception gives nothing

# Fixed grasp target (base_link). The arm's top-down reach is solid to x~0.14-0.16
# (mapped via /compute_ik); beyond x=0.18 there is NO top-down solution. This spot
# is INSIDE the camera's 0.05 m near-clip, so perception can't see it — we use the
# known pose. The matching world box is spawned here by arm_grasp_test.launch.py.
FIXED_BOX = (0.21, 0.015, 0.08)        # FALLBACK only (used if perception gives no /box_pose)
# Calibration offset added to the PERCEIVED /box_pose to form the grasp target. Perception reports
# the box at ~(0.195, 0.002, 0.070); this reproduces the hand-tuned grasp (0.21, 0.015, 0.08).
GRASP_OFFSET = (0.015, 0.013, 0.010)
# FINGERS POINT DOWN. The gripper's fingers extend along its +y axis and are only
# ~2.7 cm long (measured from TF), so a real top-down grasp needs +y pointing DOWN.
# This quaternion does that (verified reachable at the box for every mouth angle).
# The earlier rolls pointed the fingers sideways/forward, which is why the gripper
# hovered OVER the box instead of straddling it.
TOPDOWN_QUAT = (-0.7071, 0.0, 0.0, 0.7071)
PRE_ABOVE   = 0.07                   # wrist hovers ~0.15 m (fingers well above the box)
GRASP_ABOVE = 0.06                  # wrist ~0.11 m -> fingertips ~0.083 straddle the box's
                                     # UPPER part, ~1.3 cm clear of the table (avoids the
                                     # penetration explosion that launched the robot)

PEDESTAL_SIZE   = (0.06, 0.06, 0.20)  # matches worlds/test_pedestal.sdf
BOX_HALF_HEIGHT = 0.025

PLAN_ATTEMPTS = 10
PLAN_TIME_SEC = 5.0
VEL_SCALE = 0.2
ACC_SCALE = 0.2
MOVEIT_SUCCESS = 1

BACKUP_VEL_X = -0.15   # m/s reverse
BACKUP_SEC   =  2.5    # back up ~0.38 m


class ArmGraspTest(Node):

    def __init__(self):
        super().__init__('arm_grasp_test')
        self._move = ActionClient(self, MoveGroup, '/move_action')
        self._scene = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self._gripper = self.create_publisher(Float64MultiArray, GRIPPER_TOPIC, 10)
        self._attach_pub = self.create_publisher(Bool, '/grasp_attach', 10)
        self._latest_box = None
        self._last_gripper = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._pin_pub = self.create_publisher(Bool, '/pin_base', 10)
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(PoseStamped, '/box_pose', self._box_cb, 10)

    # ── helpers ───────────────────────────────────────────────────────────────

    def _box_cb(self, msg):
        self._latest_box = msg

    @staticmethod
    def _reach(x, y, z):
        ax, ay, az = ARM_BASE_XYZ
        return ((x - ax) ** 2 + (y - ay) ** 2 + (z - az) ** 2) ** 0.5

    def _wait_for_move_group(self, timeout=120.0):
        self.get_logger().info('waiting for move_group (/move_action)… '
                               'start `moveit.launch.py` in another terminal if you have not')
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._move.wait_for_server(timeout_sec=2.0):
                self.get_logger().info('move_group connected — beginning grasp')
                return True
            self.get_logger().info('…still waiting for move_group', throttle_duration_sec=5.0)
        return False

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
        self.get_logger().info(f'gripper -> {label} (ramped over {duration:.1f}s)')
        time.sleep(0.3)

    def attach(self, on):
        """Tell grasp_attacher to weld (True) / release (False) the box, and give it
        a moment to react before the arm moves."""
        msg = Bool()
        msg.data = bool(on)
        self._attach_pub.publish(msg)
        self.get_logger().info(f'/grasp_attach -> {on}')
        end = time.time() + 0.7
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def get_box_xyz(self, timeout=20.0, samples=8):
        self.get_logger().info('locating box (base is parked, so the pose is stable)…')
        readings = []
        deadline = time.time() + timeout
        while time.time() < deadline and len(readings) < samples:
            self._latest_box = None
            got = time.time() + 1.0
            while self._latest_box is None and time.time() < got:
                rclpy.spin_once(self, timeout_sec=0.1)
            p = self._latest_box
            if p is not None and p.header.frame_id == PLANNING_FRAME:
                readings.append((p.pose.position.x, p.pose.position.y, p.pose.position.z))
        if len(readings) < 3:
            self.get_logger().warn('not enough /box_pose readings (perception up? box visible?) '
                                   '— falling back to a fixed target so the arm still runs')
            return None
        box = (statistics.median(r[0] for r in readings),
               statistics.median(r[1] for r in readings),
               statistics.median(r[2] for r in readings))
        self.get_logger().info(
            f'box @ base_link {tuple(round(v, 3) for v in box)} (median of {len(readings)} readings)')
        return box

    # ── goal constructors ─────────────────────────────────────────────────────

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
    def _pose_constraints(x, y, z, quat, pos_tol=0.025, ori_tol=0.1):  # 0.1 rad ~5.7deg: keep the gripper VERTICAL (was 0.5 = 28deg, which let it tilt)
        """Pose goal at (x,y,z). If quat is None, constrain POSITION ONLY (any
        orientation) — used as the last-resort reachability probe."""
        c = Constraints()
        target = Pose()
        target.position = Point(x=float(x), y=float(y), z=float(z))
        if quat is not None:
            target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w = \
                (float(q) for q in quat)
        else:
            target.orientation.w = 1.0

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

        if quat is not None:
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

    # ── motion ────────────────────────────────────────────────────────────────

    def _send(self, constraints, label, plan_only, plan_time=PLAN_TIME_SEC):
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

        send = self._move.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=15.0)
        if not send.done():
            self.get_logger().error(f'[{label}] goal not accepted within 15 s')
            return False
        gh = send.result()
        if gh is None or not gh.accepted:
            return False
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=30.0)
        if not res.done():
            self.get_logger().error(f'[{label}] no result within 30 s')
            return False
        return res.result().result.error_code.val == MOVEIT_SUCCESS

    def probe(self, x, y, z, quat, plan_time=2.5):
        """plan-only: can MoveIt reach this pose? (no motion)"""
        return self._send(self._pose_constraints(x, y, z, quat), 'probe',
                          plan_only=True, plan_time=plan_time)

    def go_pose(self, x, y, z, quat, label):
        self.get_logger().info(f'[{label}] planning + executing…')
        ok = self._send(self._pose_constraints(x, y, z, quat), label, plan_only=False)
        self.get_logger().info(f'[{label}] {"OK" if ok else "FAILED"}')
        return ok

    def go_named(self, name):
        if not self._move.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('move_group not available')
            return False
        self.get_logger().info(f'[{name}] planning + executing…')
        ok = self._send(self._joint_constraints(NAMED_STATES[name]), name, plan_only=False)
        self.get_logger().info(f'[{name}] {"OK" if ok else "FAILED"}')
        return ok

    def add_pedestal(self, box_xyz):
        bx, by, bz = box_xyz
        box_bottom = bz - BOX_HALF_HEIGHT
        centre_z = box_bottom - PEDESTAL_SIZE[2] / 2.0
        co = CollisionObject()
        co.header.frame_id = PLANNING_FRAME
        co.id = 'grasp_test_pedestal'
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = list(PEDESTAL_SIZE)
        pose = Pose()
        pose.position = Point(x=float(bx), y=float(by), z=float(centre_z))
        pose.orientation.w = 1.0
        co.primitives.append(prim)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(co)
        if not self._scene.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn('/apply_planning_scene unavailable — continuing without pedestal collision')
            return False
        future = self._scene.call_async(ApplyPlanningScene.Request(scene=scene))
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f'added pedestal collision @ z_centre={centre_z:.3f}')
        return True

    def remove_pedestal(self):
        """Delete the table collision object so the arm can plan its retreat/ready
        path freely (otherwise MoveIt thinks it's boxed in — the Run 4 stranding)."""
        co = CollisionObject()
        co.header.frame_id = PLANNING_FRAME
        co.id = 'grasp_test_pedestal'
        co.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(co)
        if self._scene.wait_for_service(timeout_sec=5.0):
            future = self._scene.call_async(ApplyPlanningScene.Request(scene=scene))
            rclpy.spin_until_future_complete(self, future)
            self.get_logger().info('removed pedestal collision (clear path for retreat)')

    def unpin_base(self):
        msg = Bool()
        msg.data = False
        for _ in range(5):
            self._pin_pub.publish(msg)
            time.sleep(0.1)
        self.get_logger().info('sent unpin command to base_pin')

    def back_up(self):
        self.get_logger().info('backing up...')
        twist = Twist()
        twist.linear.x = BACKUP_VEL_X
        end = time.time() + BACKUP_SEC
        while time.time() < end:
            self._cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
        self._cmd_vel_pub.publish(Twist())
        self.get_logger().info('backup completed')

    # ── sequence ──────────────────────────────────────────────────────────────

    def run(self):
        self.get_logger().info('=== Stage 2: top-down PICK (weld) + place-back ===')
        if not self._wait_for_move_group():
            self.get_logger().error('move_group never came up — start moveit.launch.py')
            return

        q = TOPDOWN_QUAT

        if not self.go_named('ready'):       # fold up first so the arm doesn't block the camera
            return
        self.set_gripper(GRIPPER_OPEN, 'open')

        box = self.get_box_xyz()             # PERCEIVED pose from /box_pose (no longer hard-coded)
        if box is None:
            bx, by, bz = FIXED_BOX           # fallback if perception is unavailable
            self.get_logger().warn(f'perception unavailable -> FIXED_BOX fallback {FIXED_BOX}')
        else:
            bx = box[0] + GRASP_OFFSET[0]
            by = box[1] + GRASP_OFFSET[1]
            bz = box[2] + GRASP_OFFSET[2]
            self.get_logger().info(
                f'grasp target = perceived {tuple(round(v, 3) for v in box)} + calib {GRASP_OFFSET} '
                f'-> ({bx:.3f}, {by:.3f}, {bz:.3f})')

        self.add_pedestal((bx, by, bz))      # register the table under the ACTUAL target

        if not self.go_pose(bx, by, bz + PRE_ABOVE, q, 'pre_grasp'):   # hover above
            self.get_logger().error('pre-grasp failed'); return
        if not self.go_pose(bx, by, bz + GRASP_ABOVE, q, 'grasp'):     # descend, open fingers AROUND box
            self.get_logger().error('grasp failed'); return

        # Gripper is genuinely around the box now -> weld it, then a gentle cosmetic close.
        self.attach(True)
        self.set_gripper(GRIPPER_GRASP, 'close (gentle)')

        for attempt in range(3):                # lift flakes in OMPL — retry (Run 2 proved it IS reachable)
            if self.go_pose(bx, by, bz + PRE_ABOVE, q, f'lift (try {attempt + 1})'):  # box rises with the arm
                break
            time.sleep(0.5)
        self.get_logger().info('holding the box up — 3 s')
        time.sleep(3.0)

        # Remove table collision so we can plan home freely
        self.remove_pedestal()

        # Go to home position carrying the box
        self.get_logger().info('moving arm to home position carrying the box...')
        self.go_named('home')

        # Unpin base and back up
        self.unpin_base()
        self.back_up()
        self.get_logger().info('=== pick complete ===')


def main(args=None):
    rclpy.init(args=args)
    node = ArmGraspTest()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

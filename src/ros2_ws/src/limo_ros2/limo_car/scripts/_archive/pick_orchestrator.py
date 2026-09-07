#!/usr/bin/python3
"""
pick_orchestrator — collision-aware top-down pick with MoveIt 2 (Stage 1).

NOTE ON API CHOICE: moveit_py (the Python bindings) is NOT packaged for ROS 2
Humble, so this node talks to the running move_group via the standard
moveit_msgs ACTION + SERVICE interfaces over rclpy. That means: pure Python, no
extra packages to install, and it reuses the move_group you already launch with
`ros2 launch limo_cobot_moveit_config moveit.launch.py`. The gripper stays on its
own topic (outside MoveIt), exactly as before. No MTC, no PCL.

Requires (run in this order):
    ros2 launch limo_car ackermann_gazebo.launch.py            # sim + controllers
    ros2 launch limo_cobot_moveit_config moveit.launch.py      # move_group (+ RViz)
    ros2 launch limo_cobot_moveit_config stage1_pick.launch.py # perception + this

Sequence:
  ready (named) -> open gripper -> add TABLE collision object
  -> pre-grasp (hover, top-down) -> grasp (descend, top-down)
  -> close gripper -> lift -> ready

Why MoveIt here: the table is added as a collision object so the planner routes
the arm AROUND it — the real fix for the old "gripper clips the table edge ->
robot flies into the sky" explosion (that was an open-loop descent into geometry
MoveIt would refuse to enter).

Box pose: uses the PERCEIVED /box_pose (base_link) from box_pose_estimator if
present, else BOX_FALLBACK. Grasp orientation + height offsets are TUNABLE
constants below — calibrate once visually in the sim (also when we set the real
gripper_tcp offset; right now gripper_tcp ~= gripper_base, so GRASP_TCP_ABOVE ~=
finger length).
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Pose, Point
from std_msgs.msg import Float64MultiArray

from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, JointConstraint,
    PositionConstraint, OrientationConstraint, BoundingVolume,
    CollisionObject, PlanningScene,
)
from shape_msgs.msg import SolidPrimitive


# ── Gripper commands (6 joints driven directly; mirror signs [1,1,-1,-1,-1,1]) ─
# order MUST match mycobot_controllers.yaml: master, left2, left1, right3, right2, right1
GRIPPER_OPEN  = [ 0.15,  0.15, -0.15, -0.15, -0.15,  0.15]
GRIPPER_CLOSE = [-0.50, -0.50,  0.50,  0.50,  0.50, -0.50]
GRIPPER_TOPIC = '/mycobot_gripper_controller/commands'

# ── Frames / group / named states (must match the SRDF) ───────────────────────
PLANNING_FRAME = 'base_link'
TCP_LINK       = 'gripper_tcp'
ARM_GROUP      = 'arm'
ARM_JOINTS = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
              'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
NAMED_STATES = {
    'home':  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'ready': [0.0, -0.5, -0.6, 1.1, 0.0, 0.0],   # keep in sync with limo_cobot.srdf
}

# ── TUNABLE geometry (all in base_link). Calibrate once in the sim. ───────────
BOX_FALLBACK = (0.30, 0.00, 0.07)     # box CENTRE if /box_pose absent (x fwd, y left, z up)
# Top-down TCP orientation (gripper pointing straight down). STARTING GUESS —
# verify in RViz that the gripper actually points down, then adjust. (x,y,z,w)
TOPDOWN_QUAT = (1.0, 0.0, 0.0, 0.0)
GRASP_TCP_ABOVE     = 0.07            # TCP height above box centre at grasp (kept low — short arm)
PRE_GRASP_TCP_ABOVE = 0.10            # hover height above box centre (0.15+ exceeds the arm's 3-D reach)

# Table collision box (size from the world: 0.30 x 0.15 x 0.10), placed relative
# to the box (5 cm cube sits on the table top).
TABLE_SIZE = (0.30, 0.15, 0.10)
BOX_HALF_HEIGHT = 0.025

# Planning effort
PLAN_ATTEMPTS = 10
PLAN_TIME_SEC = 5.0
VEL_SCALE = 0.2
ACC_SCALE = 0.2
BOX_POSE_WAIT_SEC = 4.0

MOVEIT_SUCCESS = 1  # moveit_msgs/MoveItErrorCodes.SUCCESS


class PickOrchestrator(Node):

    def __init__(self):
        super().__init__('pick_orchestrator')
        self._move = ActionClient(self, MoveGroup, '/move_action')
        self._scene = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self._gripper = self.create_publisher(Float64MultiArray, GRIPPER_TOPIC, 10)
        self._latest_box = None
        self._last_gripper = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # last gripper command, for smooth ramping
        self.create_subscription(PoseStamped, '/box_pose', self._box_cb, 10)

    # ── helpers ───────────────────────────────────────────────────────────────

    def _box_cb(self, msg):
        self._latest_box = msg

    def set_gripper(self, values, label, duration=0.8, steps=20):
        # Ramp from the last command to the target so the gripper moves smoothly
        # instead of snapping — a one-shot position jump imparts a big impulse and
        # can kick the box or the robot when it closes.
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

    def get_box_xyz(self):
        # Wait for the base to ARRIVE and SETTLE before locking in the box pose.
        # Capturing the FIRST reading grabbed a far pose mid-approach (box ~0.26 m ->
        # grasp unreachable); the base then settles ~0.22 m, which IS reachable. So we
        # wait until the box position stops changing (base stopped), then capture that.
        deadline = time.time() + 40.0
        last = None
        stable_since = None
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            p = self._latest_box
            if p is None or p.header.frame_id != PLANNING_FRAME:
                continue
            xyz = (p.pose.position.x, p.pose.position.y, p.pose.position.z)
            if last is not None and abs(xyz[0] - last[0]) < 0.015 and abs(xyz[1] - last[1]) < 0.015:
                if stable_since is None:
                    stable_since = time.time()
                elif time.time() - stable_since > 1.5:
                    self.get_logger().info(
                        f'base settled — capturing box @ {tuple(round(v, 3) for v in xyz)}')
                    return xyz
            else:
                stable_since = None
                self.get_logger().info(
                    f'waiting for base to settle… box x={xyz[0]:.3f}', throttle_duration_sec=1.0)
            last = xyz
        if self._latest_box is not None:
            p = self._latest_box
            xyz = (p.pose.position.x, p.pose.position.y, p.pose.position.z)
            self.get_logger().warn(f'settle timeout — using last box @ {tuple(round(v, 3) for v in xyz)}')
            return xyz
        self.get_logger().warn(f'no /box_pose — using fallback {BOX_FALLBACK}')
        return BOX_FALLBACK

    # ── goal constructors ─────────────────────────────────────────────────────

    @staticmethod
    def _joint_constraints(values, tol=0.05):
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
    def _pose_constraints(x, y, z, quat, pos_tol=0.02, ori_tol=0.4):
        c = Constraints()
        target = Pose()
        target.position = Point(x=float(x), y=float(y), z=float(z))
        target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w = \
            (float(q) for q in quat)

        pc = PositionConstraint()
        pc.header.frame_id = PLANNING_FRAME
        pc.link_name = TCP_LINK
        pc.target_point_offset.x = pc.target_point_offset.y = pc.target_point_offset.z = 0.0
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

    # ── motion ────────────────────────────────────────────────────────────────

    def _plan_execute(self, constraints, label):
        self.get_logger().info(f'[{label}] requesting plan from move_group…')
        if not self._move.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('move_group action server not available — is moveit.launch.py running?')
            return False
        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = ARM_GROUP
        req.goal_constraints = [constraints]
        req.num_planning_attempts = PLAN_ATTEMPTS
        req.allowed_planning_time = PLAN_TIME_SEC
        req.max_velocity_scaling_factor = VEL_SCALE
        req.max_acceleration_scaling_factor = ACC_SCALE
        goal.request = req
        goal.planning_options.plan_only = False   # plan AND execute

        send = self._move.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=15.0)
        if not send.done():
            self.get_logger().error(f'[{label}] move_group did not accept the goal within 15 s '
                                    '(server busy/stuck — restart moveit.launch.py)')
            return False
        gh = send.result()
        if gh is None or not gh.accepted:
            self.get_logger().error(f'[{label}] goal rejected')
            return False
        self.get_logger().info(f'[{label}] goal accepted — planning + executing…')
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=30.0)
        if not res.done():
            self.get_logger().error(f'[{label}] no result within 30 s (planning/execution stalled)')
            return False
        code = res.result().result.error_code.val
        ok = (code == MOVEIT_SUCCESS)
        self.get_logger().info(f'[{label}] {"OK" if ok else f"FAILED (MoveItErrorCode {code})"}')
        return ok

    def go_named(self, name):
        return self._plan_execute(self._joint_constraints(NAMED_STATES[name]), name)

    def go_tcp(self, x, y, z, label):
        return self._plan_execute(self._pose_constraints(x, y, z, TOPDOWN_QUAT), label)

    def add_table(self, box_xyz):
        bx, by, bz = box_xyz
        table_top_z = bz - BOX_HALF_HEIGHT
        co = CollisionObject()
        co.header.frame_id = PLANNING_FRAME
        co.id = 'pickup_table'
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = list(TABLE_SIZE)
        pose = Pose()
        pose.position = Point(x=float(bx), y=float(by),
                              z=float(table_top_z - TABLE_SIZE[2] / 2.0))
        pose.orientation.w = 1.0
        co.primitives.append(prim)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(co)
        if not self._scene.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('/apply_planning_scene unavailable — skipping table (UNSAFE)')
            return False
        future = self._scene.call_async(ApplyPlanningScene.Request(scene=scene))
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f'added table collision @ z_centre={pose.position.z:.3f}')
        return True

    # ── sequence ──────────────────────────────────────────────────────────────

    def run(self):
        self.get_logger().info('=== Stage 1: collision-aware top-down pick ===')
        bx, by, bz = self.get_box_xyz()

        if not self.go_named('ready'):
            return
        self.set_gripper(GRIPPER_OPEN, 'open')
        self.add_table((bx, by, bz))

        if not self.go_tcp(bx, by, bz + PRE_GRASP_TCP_ABOVE, 'pre_grasp'):
            self.get_logger().error('pre-grasp unreachable — aborting before any descent')
            return
        if not self.go_tcp(bx, by, bz + GRASP_TCP_ABOVE, 'grasp'):
            self.get_logger().error('grasp pose unreachable — aborting')
            return

        self.set_gripper(GRIPPER_CLOSE, 'close')
        # TODO(Stage 3): attach the box to gripper_tcp (publish AttachedCollisionObject
        # to /attached_collision_object) for collision-aware carry.

        self.go_tcp(bx, by, bz + PRE_GRASP_TCP_ABOVE, 'lift')
        self.go_named('ready')
        self.get_logger().info('=== pick complete ===')


def main(args=None):
    rclpy.init(args=args)
    node = PickOrchestrator()
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

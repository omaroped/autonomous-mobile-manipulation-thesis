#!/usr/bin/env python3
"""real_grasp_test.py — Phase 3: isolated real-hardware grasp test.

Same idea as grip_bench.py on the sim side: isolate the grasp subsystem before
integrating it with navigation. Arm is stationary (mounted on the parked LIMO),
no docking, no base motion. Mirrors the sim's proven motion patterns exactly:
  - go_pose():          MoveGroup pose goal (position+orientation constraint, IK)
  - go_pose_straight():  Cartesian path (GetCartesianPath + ExecuteTrajectory) for
                          the descent/lift, so the tool doesn't arc sideways into
                          the box (measured in sim: ~half the vertical travel goes
                          sideways on an arced joint-space descent)
  - go_named():          all 6 joints explicitly constrained (the 2026-08-10 fix --
                          a single-joint constraint lets OMPL swing the other 5 free)

Geometry below is MEASURED on this specific setup, not the sim's defaults:
  box edge   = 2.5 cm
  stand top  = 12.5 cm off the ground
  box centre = 22.4 cm from base_link (measured from the chassis front + bumper_x)

Requires: mock-hardware MoveIt stack + arm_bridge.py both already running
(same as the Phase 2 setup). Run directly on the robot:
    python3 real_grasp_test.py
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose, Point
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (Constraints, JointConstraint, PositionConstraint,
                              OrientationConstraint, BoundingVolume,
                              MotionPlanRequest)
from moveit_msgs.srv import GetCartesianPath

# ── MoveIt constants (mirrors nav_pick_orchestrator.py) ─────────────────────
PLANNING_FRAME = 'base_link'
TCP_LINK       = 'gripper_tcp'
ARM_GROUP      = 'arm'
PLAN_ATTEMPTS  = 5
PLAN_TIME_SEC  = 5.0
# 0.1 was fine for joint-space goals (OMPL times those directly and respects the
# scale). GetCartesianPath does NOT respect velocity scaling -- it always returns a
# full-speed path -- so straight-line moves are manually retimed by 1/VEL_SCALE.
# At 0.1 that's a 10x stretch: a 6 cm descent took 62s, and the resulting per-tick
# angular change was often small enough to fall under arm_bridge.py's 1deg minimum
# step, which is why it looked like stop-start jerking instead of smooth motion.
# 0.2 matches the sim orchestrator's own default and should fix both.
VEL_SCALE      = 0.2
ACC_SCALE      = 0.2
EXEC_TIMEOUT_SEC = 90.0   # generous ceiling; a cancel is sent if this is hit
CART_MAX_STEP     = 0.005
CART_MIN_FRACTION = 0.90
MOVEIT_SUCCESS = 1

ARM_JOINTS = [
    'joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
    'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6',
]
READY_POSE = {
    'joint2_to_joint1': 0.0, 'joint3_to_joint2': -0.5, 'joint4_to_joint3': -0.6,
    'joint5_to_joint4': 1.1, 'joint6_to_joint5': 0.0, 'joint6output_to_joint6': 0.0,
}

TOPDOWN_QUAT = (-0.7071, 0.0, 0.0, 0.7071)
GRIPPER_TOPIC = '/mycobot_gripper_controller/commands'
GRIPPER_OPEN  = [ 0.15,  0.15, -0.15, -0.15, -0.15,  0.15]
GRIPPER_GRASP = [-0.20, -0.20,  0.20,  0.20,  0.20, -0.20]

# ── Real, MEASURED geometry for this setup ───────────────────────────────────
BOX_HALF          = 0.0125   # half of 2.5 cm box
BASE_LINK_GROUND_Z = 0.145   # this robot's base_link height off the ground
STAND_HEIGHT      = 0.13     # adjusted from 0.125 -- gripper was pressing hard into
                              # the stand at grasp height; +0.5cm raises the TCP target
TABLE_TOP_BASE_Z  = STAND_HEIGHT - BASE_LINK_GROUND_Z   # -0.02 m
GRASP_Z           = TABLE_TOP_BASE_Z + BOX_HALF          # TCP target = box centre
BOX_X             = 0.224    # measured: base_link -> box centre
BOX_Y             = 0.0      # centred
HOVER_CLEARANCE   = 0.06     # same starting point as the sim; will warn if unreachable


class RealGraspTest(Node):
    def __init__(self):
        super().__init__('real_grasp_test')
        self._move = ActionClient(self, MoveGroup, '/move_action')
        self._exec = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self._cart = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self._gripper = self.create_publisher(Float64MultiArray, GRIPPER_TOPIC, 10)
        self._latest_joint_state = None
        from sensor_msgs.msg import JointState
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

    def _js_cb(self, msg):
        self._latest_joint_state = msg

    def current_angles(self):
        while self._latest_joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.2)
        msg = self._latest_joint_state
        return {j: msg.position[msg.name.index(j)] for j in ARM_JOINTS if j in msg.name}

    @staticmethod
    def _retime(traj, scale):
        k = 1.0 / max(float(scale), 1e-3)
        for pt in traj.joint_trajectory.points:
            t = (pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9) * k
            pt.time_from_start.sec     = int(t)
            pt.time_from_start.nanosec = int(round((t - int(t)) * 1e9))
            pt.velocities    = [v / k for v in pt.velocities]
            pt.accelerations = [a / (k * k) for a in pt.accelerations]
        return traj

    def _send(self, constraints, label):
        if not self._move.wait_for_server(timeout_sec=10.0):
            print(f'[{label}] FAILED: /move_action server not available')
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
        goal.planning_options.plan_only = False

        send_fut = self._move.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut, timeout_sec=10.0)
        gh = send_fut.result()
        if gh is None or not gh.accepted:
            print(f'[{label}] FAILED: goal rejected')
            return False
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=EXEC_TIMEOUT_SEC)
        if not res_fut.done():
            print(f'[{label}] FAILED: no result in {EXEC_TIMEOUT_SEC:.0f}s — cancelling')
            cancel_fut = gh.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_fut, timeout_sec=5.0)
            return False
        ok = res_fut.result().result.error_code.val == MOVEIT_SUCCESS
        print(f'[{label}] {"OK" if ok else "FAILED"}')
        return ok

    def go_named_ready(self):
        current = self.current_angles()
        jcs = []
        for j in ARM_JOINTS:
            jc = JointConstraint()
            jc.joint_name = j
            jc.position = READY_POSE[j]
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            jcs.append(jc)
        c = Constraints()
        c.joint_constraints = jcs
        return self._send(c, 'ready')

    def _pose_constraints(self, x, y, z, quat, pos_tol=0.01, ori_tol=0.1):
        c = Constraints()
        target = Pose()
        target.position = Point(x=float(x), y=float(y), z=float(z))
        (target.orientation.x, target.orientation.y,
         target.orientation.z, target.orientation.w) = (float(v) for v in quat)

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

    def go_pose(self, x, y, z, q, label):
        print(f'[{label}] planning to ({x:.3f}, {y:.3f}, {z:.3f})...')
        return self._send(self._pose_constraints(x, y, z, q), label)

    def go_pose_straight(self, x, y, z, q, label):
        target = Pose()
        target.position = Point(x=float(x), y=float(y), z=float(z))
        (target.orientation.x, target.orientation.y,
         target.orientation.z, target.orientation.w) = (float(v) for v in q)

        req = GetCartesianPath.Request()
        req.header.frame_id = PLANNING_FRAME
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = ARM_GROUP
        req.link_name = TCP_LINK
        req.waypoints = [target]
        req.max_step = CART_MAX_STEP
        req.jump_threshold = 0.0
        req.avoid_collisions = True

        print(f'[{label}] planning STRAIGHT line to ({x:.3f}, {y:.3f}, {z:.3f})...')
        if not self._cart.wait_for_service(timeout_sec=5.0):
            print(f'[{label}] /compute_cartesian_path unavailable — falling back')
            return self.go_pose(x, y, z, q, label)
        fut = self._cart.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        if not fut.done() or fut.result() is None:
            print(f'[{label}] cartesian service no answer — falling back')
            return self.go_pose(x, y, z, q, label)

        frac = float(fut.result().fraction)
        if frac < CART_MIN_FRACTION:
            print(f'[{label}] straight path only {frac*100:.0f}% solvable — falling back')
            return self.go_pose(x, y, z, q, label)

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = self._retime(fut.result().solution, VEL_SCALE)

        if not self._exec.wait_for_server(timeout_sec=5.0):
            print(f'[{label}] /execute_trajectory unavailable — falling back')
            return self.go_pose(x, y, z, q, label)
        send_fut = self._exec.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut, timeout_sec=15.0)
        gh = send_fut.result() if send_fut.done() else None
        if gh is None or not gh.accepted:
            print(f'[{label}] straight-line goal rejected — falling back')
            return self.go_pose(x, y, z, q, label)
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=EXEC_TIMEOUT_SEC)
        if not res_fut.done():
            print(f'[{label}] straight-line move did not finish in {EXEC_TIMEOUT_SEC:.0f}s — cancelling')
            cancel_fut = gh.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_fut, timeout_sec=5.0)
            return False
        ok = res_fut.result().result.error_code.val == MOVEIT_SUCCESS
        print(f'[{label}] STRAIGHT {"OK" if ok else "FAILED"} ({frac*100:.0f}% interpolated)')
        return ok

    def set_gripper(self, values, label, duration=0.8, steps=20):
        start = GRIPPER_OPEN   # assume open at script start; harmless if already there
        for i in range(1, steps + 1):
            a = i / steps
            interp = [s + a * (v - s) for s, v in zip(start, values)]
            msg = Float64MultiArray()
            msg.data = [float(x) for x in interp]
            self._gripper.publish(msg)
            rclpy.spin_once(self, timeout_sec=duration / steps)
        print(f'gripper -> {label}')


def main():
    rclpy.init()
    node = RealGraspTest()

    print(f'Grasp target (base_link): x={BOX_X:.3f} y={BOX_Y:.3f} z={GRASP_Z:.4f}')
    print(f'Hover height: z={GRASP_Z + HOVER_CLEARANCE:.4f}')

    print('\n=== 1. ready ===')
    if not node.go_named_ready():
        print('ABORT: ready failed'); return

    print('\n=== 2. open gripper ===')
    node.set_gripper(GRIPPER_OPEN, 'open')

    print('\n=== 3. hover above box ===')
    if not node.go_pose(BOX_X, BOX_Y, GRASP_Z + HOVER_CLEARANCE, TOPDOWN_QUAT, 'hover'):
        print('ABORT: hover unreachable — check BOX_X/GRASP_Z geometry'); return

    print('\n=== 4. descend straight to grasp point ===')
    if not node.go_pose_straight(BOX_X, BOX_Y, GRASP_Z, TOPDOWN_QUAT, 'descend'):
        print('ABORT: descent failed'); return

    print('\n=== 5. close gripper ===')
    node.set_gripper(GRIPPER_GRASP, 'closed')
    time.sleep(0.5)

    print('\n=== 6. lift straight up ===')
    if not node.go_pose_straight(BOX_X, BOX_Y, GRASP_Z + HOVER_CLEARANCE, TOPDOWN_QUAT, 'lift'):
        print('WARNING: lift failed — box may still be on the stand')

    print('\n>>> CHECK NOW: is the box actually held in the gripper? <<<')
    input('Press Enter once you have looked...')

    print('\n=== 7. lower back down ===')
    node.go_pose_straight(BOX_X, BOX_Y, GRASP_Z, TOPDOWN_QUAT, 'lower')

    print('\n=== 8. open gripper (release) ===')
    node.set_gripper(GRIPPER_OPEN, 'open')
    time.sleep(0.3)

    print('\n=== 9. retract to ready ===')
    node.go_named_ready()

    print('\n=== done ===')
    rclpy.shutdown()


if __name__ == '__main__':
    main()

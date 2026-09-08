#!/usr/bin/python3
"""
place_only_test.py — pick up the box (weld), show it between fingers, then place it.

Sequence:
  1. Navigate to the table (same as normal pipeline)
  2. Arm descends to box → weld fires → box follows gripper
  3. Arm lifts → you SEE box between fingers rising with the arm
  4. Arm lowers back down to table
  5. Gripper opens + weld releases → box sits on table
  6. Arm retreats up

Run AFTER:
  ros2 launch limo_car nav_pick.launch.py
  ros2 launch limo_cobot_moveit_config moveit.launch.py
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Float64MultiArray, Bool
from geometry_msgs.msg import Pose, Point, PoseWithCovarianceStamped
from shape_msgs.msg import SolidPrimitive

from nav2_msgs.action import NavigateToPose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, JointConstraint,
    PositionConstraint, OrientationConstraint, BoundingVolume,
)

# ── Nav goal (same as orchestrator) ──────────────────────────────────────────
NAV_X   = -2.0
NAV_Y   =  4.60
NAV_YAW = -1.5708   # facing -Y toward the table

# ── Grasp geometry (same calibrated values as orchestrator) ───────────────────
BOX_X       =  0.28
BOX_Y       =  0.015
BOX_Z       =  0.08   # box centre height
HOVER_ABOVE =  0.07   # TCP above box at pre-grasp hover
GRASP_ABOVE =  0.0    # TCP above box centre at grasp. 0 since 2026-08-01: gripper_tcp was
                       # moved to the real grasp point (ackermann_with_sensor.xacro),
                       # so the tool goes straight to the box centre. Was 0.06, which
                       # was silently correcting for the TCP sitting on the palm.
PLACE_Z     =  BOX_Z + GRASP_ABOVE   # same height as grasp — set box back down

TOPDOWN_QUAT = (-0.7071, 0.0, 0.0, 0.7071)

# ── Gripper ───────────────────────────────────────────────────────────────────
GRIPPER_OPEN  = [ 0.15,  0.15, -0.15, -0.15, -0.15,  0.15]
GRIPPER_GRASP = [-0.20, -0.20,  0.20,  0.20,  0.20, -0.20]
GRIPPER_TOPIC = '/mycobot_gripper_controller/commands'

# ── MoveIt ────────────────────────────────────────────────────────────────────
ARM_GROUP  = 'arm'
FRAME      = 'base_link'
TCP_LINK   = 'gripper_tcp'
ARM_JOINTS = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
              'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
READY_J    = [0.0, -0.5, -0.6, 1.1, 0.0, 0.0]
MOVEIT_OK  = 1


class PlaceOnlyTest(Node):
    def __init__(self):
        super().__init__('place_only_test')
        self._nav    = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._move   = ActionClient(self, MoveGroup, '/move_action')
        self._grip   = self.create_publisher(Float64MultiArray, GRIPPER_TOPIC, 10)
        self._weld   = self.create_publisher(Bool, '/grasp_attach', 10)   # CORRECT topic
        self._amcl   = self.create_publisher(
                           PoseWithCovarianceStamped, '/initialpose', 10)
        self._last_g = list(GRIPPER_OPEN)

    # ── MoveIt ────────────────────────────────────────────────────────────────

    def _wait_move(self):
        self.get_logger().info('waiting for move_group…')
        while rclpy.ok():
            if self._move.wait_for_server(timeout_sec=2.0):
                self.get_logger().info('move_group ready')
                return

    def _send(self, c, label):
        goal = MoveGroup.Goal()
        req  = MotionPlanRequest()
        req.group_name  = ARM_GROUP
        req.goal_constraints = [c]
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor     = 0.4
        req.max_acceleration_scaling_factor = 0.4
        goal.request = req
        fut = self._move.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        if not fut.done() or not fut.result().accepted:
            self.get_logger().error(f'[{label}] rejected')
            return False
        r = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self, r, timeout_sec=30.0)
        ok = r.result().result.error_code.val == MOVEIT_OK
        self.get_logger().info(f'[{label}] {"OK" if ok else "FAILED"}')
        return ok

    def go_joints(self, joints, label):
        c = Constraints()
        for name, val in zip(ARM_JOINTS, joints):
            jc = JointConstraint()
            jc.joint_name      = name
            jc.position        = float(val)
            jc.tolerance_above = 0.05
            jc.tolerance_below = 0.05
            jc.weight          = 1.0
            c.joint_constraints.append(jc)
        return self._send(c, label)

    def go_pose(self, x, y, z, q, label):
        c  = Constraints()
        tg = Pose()
        tg.position = Point(x=float(x), y=float(y), z=float(z))
        tg.orientation.x, tg.orientation.y = float(q[0]), float(q[1])
        tg.orientation.z, tg.orientation.w = float(q[2]), float(q[3])

        pc = PositionConstraint()
        pc.header.frame_id = FRAME
        pc.link_name = TCP_LINK
        bv = BoundingVolume()
        sp = SolidPrimitive()
        sp.type       = SolidPrimitive.SPHERE
        sp.dimensions = [0.025]
        bv.primitives.append(sp)
        bv.primitive_poses.append(tg)
        pc.constraint_region = bv
        pc.weight = 1.0
        c.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.header.frame_id = FRAME
        oc.link_name       = TCP_LINK
        oc.orientation     = tg.orientation
        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = 0.1
        oc.weight = 1.0
        c.orientation_constraints.append(oc)
        return self._send(c, label)

    # ── Gripper & weld ────────────────────────────────────────────────────────

    def set_gripper(self, target, label, steps=20, dur=0.8):
        start = self._last_g[:]
        for i in range(1, steps + 1):
            a = i / steps
            msg = Float64MultiArray()
            msg.data = [float(s + a * (v - s)) for s, v in zip(start, target)]
            self._grip.publish(msg)
            rclpy.spin_once(self, timeout_sec=dur / steps)
        self._last_g = list(target)
        self.get_logger().info(f'gripper → {label}')
        time.sleep(0.3)

    def weld(self, on: bool):
        msg = Bool()
        msg.data = on
        self._weld.publish(msg)
        time.sleep(0.5)
        self.get_logger().info(f'weld → {"ON — box attaches to gripper" if on else "OFF — box released"}')

    # ── Navigation ────────────────────────────────────────────────────────────

    def navigate(self):
        self.get_logger().info(f'navigating to table ({NAV_X}, {NAV_Y})…')
        self._nav.wait_for_server(timeout_sec=20.0)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp    = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = NAV_X
        goal.pose.pose.position.y = NAV_Y
        import math
        goal.pose.pose.orientation.z = math.sin(NAV_YAW / 2)
        goal.pose.pose.orientation.w = math.cos(NAV_YAW / 2)
        fut = self._nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error('nav goal rejected')
            return False
        r = gh.get_result_async()
        rclpy.spin_until_future_complete(self, r, timeout_sec=120.0)
        self.get_logger().info('arrived at table')
        return True

    # ── Main ─────────────────────────────────────────────────────────────────

    def run(self):
        self.get_logger().info('══ PLACE TEST: navigate → pick (weld) → show → place ══')

        self._wait_move()

        # fold arm for driving
        self.set_gripper(GRIPPER_OPEN, 'open')
        self.go_joints([0.0, 1.2, -0.6, -0.6, 0.0, 0.0], 'travel')

        # navigate
        if not self.navigate():
            return
        time.sleep(0.5)

        # ready pose
        self.go_joints(READY_J, 'ready')

        # hover above box
        self.go_pose(BOX_X, BOX_Y, BOX_Z + HOVER_ABOVE, TOPDOWN_QUAT, 'hover')

        # descend to box
        self.go_pose(BOX_X, BOX_Y, BOX_Z + GRASP_ABOVE, TOPDOWN_QUAT, 'descend')

        # close fingers — pure friction grasp, no weld
        self.set_gripper(GRIPPER_GRASP, 'close')
        self.get_logger().info('>>> FINGERS CLOSED — physical grip only <<<')
        time.sleep(1.5)   # pause — watch the box attach in Gazebo

        # LIFT — box rises with the arm
        self.go_pose(BOX_X, BOX_Y, BOX_Z + HOVER_ABOVE + 0.08, TOPDOWN_QUAT, 'lift')
        self.get_logger().info('>>> ARM LIFTED WITH BOX — watch it in Gazebo <<<')
        time.sleep(2.0)   # pause — see the box between the fingers while lifted

        # LOWER back to table to PLACE
        self.go_pose(BOX_X, BOX_Y, BOX_Z + GRASP_ABOVE, TOPDOWN_QUAT, 'lower to place')

        # RELEASE — box drops onto table (friction only, no weld)
        self.set_gripper(GRIPPER_OPEN, 'open')
        self.get_logger().info('>>> RELEASED — box should be on the table <<<')
        time.sleep(1.5)

        # retreat
        self.go_pose(BOX_X, BOX_Y, BOX_Z + HOVER_ABOVE + 0.10, TOPDOWN_QUAT, 'retreat')
        self.go_joints(READY_J, 'ready')

        self.get_logger().info('══ DONE ══')


def main(args=None):
    rclpy.init(args=args)
    node = PlaceOnlyTest()
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

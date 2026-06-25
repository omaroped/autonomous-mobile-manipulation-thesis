#!/usr/bin/env python3
"""test_place_arm.py — isolated arm placement test.

Run while the sim is live (after nav_pick.launch.py is up).
The robot does NOT need to be at the place table — this just tests
whether the arm can reach the place position and release.

Usage:
    ros2 run limo_car test_place_arm
"""
import time, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float64MultiArray, Bool
from geometry_msgs.msg import Pose, Point
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (MotionPlanRequest, Constraints, JointConstraint,
                              PositionConstraint, OrientationConstraint, BoundingVolume)

ARM_JOINTS = ['joint2_to_joint1','joint3_to_joint2','joint4_to_joint3',
              'joint5_to_joint4','joint6_to_joint5','joint6output_to_joint6']
READY_J    = [0.0, -0.5, -0.6, 1.1, 0.0, 0.0]
TOPDOWN_Q  = (-0.7071, 0.0, 0.0, 0.7071)
GRIPPER_OPEN  = [ 0.15,  0.15, -0.15, -0.15, -0.15,  0.15]

# ── place target (matches orchestrator) ───────────────────────────────────────
PX, PY    = 0.25, 0.0   # table centre in base_link
HOVER_Z   = 0.18        # 0.06 surface + 0.02 half-box + 0.10 hover
PLACE_Z   = 0.14        # 0.06 surface + 0.02 half-box + 0.06 above


class ArmPlaceTest(Node):
    def __init__(self):
        super().__init__('test_place_arm')
        self._move  = ActionClient(self, MoveGroup, '/move_action')
        self._grip  = self.create_publisher(Float64MultiArray,
                          '/mycobot_gripper_controller/commands', 10)
        self._weld  = self.create_publisher(Bool, '/grasp_attach', 10)

    def _send(self, c, label):
        goal = MoveGroup.Goal()
        req  = MotionPlanRequest()
        req.group_name = 'arm'
        req.goal_constraints = [c]
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3
        goal.request = req
        fut = self._move.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        if not fut.done() or not fut.result().accepted:
            self.get_logger().error(f'[{label}] rejected')
            return False
        r = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self, r, timeout_sec=30.0)
        ok = r.result().result.error_code.val == 1
        self.get_logger().info(f'[{label}] {"✓ OK" if ok else "✗ FAILED"}')
        return ok

    def go_joints(self, joints, label):
        c = Constraints()
        for name, val in zip(ARM_JOINTS, joints):
            jc = JointConstraint()
            jc.joint_name = name; jc.position = float(val)
            jc.tolerance_above = jc.tolerance_below = 0.05
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        return self._send(c, label)

    def go_pose(self, x, y, z, q, label):
        c  = Constraints()
        tg = Pose()
        tg.position = Point(x=float(x), y=float(y), z=float(z))
        tg.orientation.x, tg.orientation.y = float(q[0]), float(q[1])
        tg.orientation.z, tg.orientation.w = float(q[2]), float(q[3])
        pc = PositionConstraint()
        pc.header.frame_id = 'base_link'; pc.link_name = 'gripper_tcp'
        bv = BoundingVolume(); sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE; sp.dimensions = [0.025]
        bv.primitives.append(sp); bv.primitive_poses.append(tg)
        pc.constraint_region = bv; pc.weight = 1.0
        c.position_constraints.append(pc)
        oc = OrientationConstraint()
        oc.header.frame_id = 'base_link'; oc.link_name = 'gripper_tcp'
        oc.orientation = tg.orientation
        oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance = \
            oc.absolute_z_axis_tolerance = 0.15
        oc.weight = 1.0
        c.orientation_constraints.append(oc)
        return self._send(c, label)

    def gripper_open(self):
        msg = Float64MultiArray()
        msg.data = list(GRIPPER_OPEN)
        self._grip.publish(msg)
        time.sleep(0.5)

    def run(self):
        self.get_logger().info('=== ARM PLACE TEST ===')
        self.get_logger().info(f'Target: ({PX}, {PY})  hover_z={HOVER_Z}  place_z={PLACE_Z}')

        self.get_logger().info('Waiting for MoveIt…')
        self._move.wait_for_server(timeout_sec=30.0)

        # 1. ready
        self.get_logger().info('Step 1: ready pose')
        if not self.go_joints(READY_J, 'ready'):
            self.get_logger().error('READY failed — is MoveIt running?'); return

        # 2. hover above place target
        self.get_logger().info(f'Step 2: hover  ({PX}, {PY}, {HOVER_Z})')
        if not self.go_pose(PX, PY, HOVER_Z, TOPDOWN_Q, 'hover'):
            self.get_logger().error('HOVER failed — arm cannot reach this position'); return

        # 3. descend to place height
        self.get_logger().info(f'Step 3: descend ({PX}, {PY}, {PLACE_Z})')
        if not self.go_pose(PX, PY, PLACE_Z, TOPDOWN_Q, 'place'):
            self.get_logger().error('PLACE descent failed'); return

        # 4. release
        self.get_logger().info('Step 4: release')
        weld_msg = Bool(); weld_msg.data = False
        self._weld.publish(weld_msg)
        self.gripper_open()

        # 5. lift and home
        self.get_logger().info('Step 5: lift + home')
        self.go_pose(PX, PY, HOVER_Z, TOPDOWN_Q, 'lift')
        self.go_joints(READY_J, 'home')

        self.get_logger().info('=== DONE — if the arm moved through all steps, placing works ===')


def main(args=None):
    rclpy.init(args=args)
    node = ArmPlaceTest()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

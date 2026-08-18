#!/usr/bin/env python3
"""
grip_bench.py — a test bench for ONE question: does the gripper actually hold the box?

WHY THIS EXISTS
---------------
Debugging the grasp through the full pipeline costs ~3.5 minutes per data point and
mixes in every other subsystem: Nav2 docking, perception, the approach IK. If a pick
fails you cannot tell whether the grip is bad or the arm simply arrived 8 mm off.
calib_pick.py already removes the driving, but it still re-perceives, hovers and
descends every iteration (~25 s), so approach error is still in the measurement.

This bench removes the approach entirely. The arm goes to the grasp pose ONCE, and
then every trial TELEPORTS the box to sit exactly between the fingers before closing.
Centring error is therefore zero by construction, and anything that still fails is a
property of the GRIP — which is what we are trying to fix.

One trial is ~12 s instead of ~210 s, and every parameter worth sweeping is a live
ROS parameter, so a sweep needs no relaunch.

WHAT EACH TRIAL MEASURES
------------------------
  held          did the box rise with the gripper (Gazebo ground truth, not vision)
  rise_m        how far it actually rose
  shove_mm      how far the box was pushed sideways BY THE CLOSING FINGERS. A large
                value means one finger reached the box first and shoved it instead of
                the pair clamping it — a real failure mode that the old kinematic weld
                completely hid, because the weld teleported the box into the hand
                regardless of what the fingers did.
  l_frames /    how many samples during the close each finger's contact sensor
  r_frames      reported touching, and
  both_frames   how many samples BOTH reported at once. This is the direct evidence
                for "are the finger faces really touching the box faces": if these
                stay 0, the fingers never touch; if they flicker, contact is marginal;
                if both are high and steady, contact is fine and the fault is in the
                grasp plugin's detection instead.
  stall         the gripper angle at which the close stopped resisting-free motion,
                or None if it swept to the floor with nothing in the way.

LIVE PARAMETERS (ros2 param set /grip_bench <name> <value> — applies next trial)
-------------------------------------------------------------------------------
  close_sec     how long the close takes. Slower = gentler contact.
  close_floor   how far the fingers squeeze (rad, negative = tighter).
  backoff       after closing, open by one step to relieve force. Suspect 4: this may
                break contact at exactly the moment the plugin needs to see it.
  pose_x/y/z    the grasp point in base_link. Nudge it to change where the tool aims
                relative to the box — that tolerance is a real thesis number.
  box_world_x/y/z  the box's home on the table. It is always put back exactly here.
  lift_m        how far to lift when testing whether the grip holds (default 0.10).
  hold_sec      how long to hang in the air before judging (default 2.0).
  min_rise      how far the box must rise to count as held (default 0.05).

Usage:
    ros2 launch limo_car grip_bench.launch.py
    ros2 launch limo_car grip_bench.launch.py loops:=20 close_sec:=2.0
"""

import csv
import math
import os
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Pose
from shape_msgs.msg import SolidPrimitive

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, JointConstraint,
    PositionConstraint, OrientationConstraint, BoundingVolume,
)
from gazebo_msgs.srv import GetEntityState, SetEntityState
from gazebo_msgs.msg import EntityState, ContactsState

import tf2_ros


# ── Gripper (must match nav_pick_orchestrator.py) ────────────────────────────
GRIPPER_OPEN  = [ 0.15,  0.15, -0.15, -0.15, -0.15,  0.15]
GRIPPER_GRASP = [-0.20, -0.20,  0.20,  0.20,  0.20, -0.20]
GRIPPER_TOPIC = '/mycobot_gripper_controller/commands'
CLOSE_STEP_RAD = 0.005
CLOSE_STEP_SEC = 0.08
LAG_THRESH     = 0.08     # rad the actual joint may lag the command before "stalled"
STALL_STEPS    = 10       # consecutive lagging steps that confirm a real stall

# ── MoveIt ───────────────────────────────────────────────────────────────────
TOPDOWN_QUAT   = (-0.7071, 0.0, 0.0, 0.7071)
PLANNING_FRAME = 'base_link'
TCP_LINK       = 'gripper_tcp'
ARM_GROUP      = 'arm'
ARM_JOINTS     = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
                  'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
# A known-good arm configuration to recover to. Planning FROM a bad configuration is far
# harder than planning from a neutral one, so after any failure the arm is sent here before
# retrying — otherwise each failed attempt leaves the arm somewhere worse and the next
# attempt is less likely to succeed than the last.
READY_J = [0.0, -0.5, -0.6, 1.1, 0.0, 0.0]
MOVEIT_SUCCESS = 1
VEL_SCALE = 0.2
ACC_SCALE = 0.2

# NOTE: gripper_tcp is a massless frame welded to gripper_base with a ZERO offset
# (ackermann_with_sensor.xacro:73-78), and Gazebo's URDF->SDF conversion folds such a
# link into its parent — so `mbot::gripper_tcp` is NOT a queryable Gazebo link. If a
# future change needs the tool pose from Gazebo, ask for `mbot::gripper_base`, which
# sits at exactly the same place.

# Where the box rests on the pickup table, in WORLD coordinates. Must match
# final_map.world's stack_box_0.
# Table top 0.14 (height +40% on 2026-08-02) + 0.02 half box height = 0.16.
BOX_TABLE_WORLD = (-2.0, 4.0, 0.16)

# The grasp point in base_link. The robot spawns pre-docked (spawn_y=4.24) and never
# drives, so this is a constant, not something to compute. Measured for calib_pick.py:
# box at base_link x=0.240, z=-0.021. gripper_tcp is the grasp point itself.
POSE_X_DEF = 0.240
POSE_Y_DEF = 0.000
# The box centre in base_link — MEASURED from Gazebo 2026-08-02, not inferred:
#   base_footprint world z = -0.005   (the base settles 5 mm onto its wheels)
#   base_link      world z =  0.145   (= -0.005 + the 0.15 base_joint offset, ackermann.xacro:189)
#   box centre     world z =  0.16    (table top 0.14 + half box 0.02)
#   -> box centre in base_link = 0.16 - 0.145 = 0.015
# The previous 0.029 came from assuming base_link sat at world 0.141, a figure back-derived
# from calib_pick's BOX_BASE_Z = -0.021 — itself a PERCEPTION reading, and perception is
# known to bias the box centre high at close range. So the arm was aiming 4 mm above the
# box on every grasp, which is enough to make a good grip marginal.
POSE_Z_DEF = 0.015     # the box CENTRE in base_link. Earlier 0.039 = -0.021 + 0.06,
                       # where the 0.06 pushed the old palm-mounted TCP down to the
                       # fingers. gripper_tcp now IS the grasp point, so the target
                       # is simply the box centre.


class GripBench(Node):

    def __init__(self):
        super().__init__('grip_bench')

        self.declare_parameter('loops',       5)
        self.declare_parameter('close_sec',   4.0)
        self.declare_parameter('close_floor', GRIPPER_GRASP[0])
        self.declare_parameter('backoff',     True)
        # Where the box lives on the table, in WORLD coordinates. The box is always
        # put back exactly here — it is never teleported to the gripper, because doing
        # that drops it in mid-air whenever the arm is not already at the table.
        self.declare_parameter('box_world_x', BOX_TABLE_WORLD[0])
        self.declare_parameter('box_world_y', BOX_TABLE_WORLD[1])
        self.declare_parameter('box_world_z', BOX_TABLE_WORLD[2])
        # The grasp point in base_link. Fixed, because the robot spawns pre-docked and
        # never drives. From the docked geometry measured for calib_pick.py.
        self.declare_parameter('pose_x', POSE_X_DEF)
        self.declare_parameter('pose_y', POSE_Y_DEF)
        self.declare_parameter('pose_z', POSE_Z_DEF)
        # capture_pose: move to the grasp point once, print the six arm joint angles
        # for the URDF, and exit. Used to calibrate the arm's SPAWN pose (Step 2).
        self.declare_parameter('capture_pose', False)
        # interactive: instead of running N trials automatically, take typed commands.
        # Lets the docking distance be walked in by hand and a pick fired on demand,
        # in ONE running simulation — which is the only way to actually watch what
        # changes between one distance and the next.
        self.declare_parameter('interactive', False)
        # wiggle: after lifting, swing and roll the wrist while holding the box so
        # you can SEE whether the box turns with the hand. The old kinematic weld
        # copied POSITION ONLY, so the box stayed flat however the wrist rotated —
        # a real fixed joint couples all 6 DOF. This is the visual proof.
        self.declare_parameter('wiggle', False)
        # How close the TCP must get to the commanded point. Was effectively 0.025,
        # which is wider than the 0.035 m box — the arm could stop 2.5 cm low and
        # still be called a success, which is why it sometimes hit the table.
        self.declare_parameter('pos_tol', 0.01)
        # Lift/clearance above the box. Kept SHORT on purpose: the wrist+gripper stack is
        # 213 mm long and must hang straight down from the elbow, so every centimetre the
        # tool rises forces the elbow higher — and at a 0.15 m table the elbow runs out of
        # room. Measured: a 0.10 m lift makes planning fail; the box only needs to clear
        # the table, which 0.06 m does with margin.
        self.declare_parameter('lift_m',      0.06)
        self.declare_parameter('hold_sec',    2.0)
        self.declare_parameter('settle_sec',  0.6)
        # A held box rises by lift_m; a box left on the table rises 0.000. 0.03 separates
        # them unambiguously while allowing the shorter lift above.
        self.declare_parameter('min_rise',    0.03)
        # Robot spawn pose, reset at the start of EVERY trial so the fixed grasp point
        # keeps lining up with the box. Must match spawn_y in grip_bench.launch.py.
        self.declare_parameter('robot_x',   -2.0)
        self.declare_parameter('robot_y',    4.24)
        self.declare_parameter('robot_yaw', -1.5708)
        self.declare_parameter('box_name',    'stack_box_0')
        self.declare_parameter('csv',         '')

        self._move = ActionClient(self, MoveGroup, '/move_action')
        self._grip = self.create_publisher(Float64MultiArray, GRIPPER_TOPIC, 10)
        self._last_grip = list(GRIPPER_OPEN)

        self._get_state = self.create_client(GetEntityState, '/get_entity_state')
        self._set_state = self.create_client(SetEntityState, '/set_entity_state')

        # Actual gripper joint angle, for stall detection.
        self._grip_actual = 0.0
        # Live arm joint angles, for capture_pose.
        self._arm_actual = {}
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)

        # Finger contact sensors — declared in the adaptive-gripper URDF and published
        # by libgazebo_ros_bumper. They are the ONLY direct evidence of whether the
        # finger faces are really touching the box, independent of the grasp plugin.
        self._l_touch = False
        self._r_touch = False
        self.create_subscription(ContactsState, '/gripper_left1_bumper',
                                 self._left_cb, 10)
        self.create_subscription(ContactsState, '/gripper_right1_bumper',
                                 self._right_cb, 10)

        self._tf_buf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf, self)

        self._rows = []

    # ── param helper ─────────────────────────────────────────────────────────

    def _p(self, name):
        return self.get_parameter(name).value

    # ── callbacks ────────────────────────────────────────────────────────────

    def _joint_cb(self, msg: JointState):
        try:
            self._grip_actual = msg.position[msg.name.index('gripper_controller')]
        except ValueError:
            pass
        # Keep the live arm angles so report_capture() can print the spawn pose.
        for j in ARM_JOINTS:
            try:
                self._arm_actual[j] = msg.position[msg.name.index(j)]
            except ValueError:
                pass

    def _left_cb(self, msg: ContactsState):
        self._l_touch = len(msg.states) > 0

    def _right_cb(self, msg: ContactsState):
        self._r_touch = len(msg.states) > 0

    # ── Gazebo ground truth ──────────────────────────────────────────────────

    def _entity_pose(self, name):
        """World pose of a Gazebo entity (model or model::link), or None."""
        if not self._get_state.wait_for_service(timeout_sec=3.0):
            return None
        req = GetEntityState.Request()
        req.name = name
        req.reference_frame = 'world'
        fut = self._get_state.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        if not fut.done() or fut.result() is None or not fut.result().success:
            return None
        return fut.result().state.pose.position

    def _entity_pose_full(self, name):
        """World position AND orientation of a Gazebo entity, or None."""
        if not self._get_state.wait_for_service(timeout_sec=3.0):
            return None
        req = GetEntityState.Request()
        req.name = name
        req.reference_frame = 'world'
        fut = self._get_state.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        if not fut.done() or fut.result() is None or not fut.result().success:
            return None
        return fut.result().state.pose

    def reset_robot_to_spawn(self):
        """Put the whole robot back at its docked spawn pose, with zero velocity.

        WHY THIS MATTERS AS MUCH AS RESETTING THE BOX. The grasp point is a constant in
        base_link (0.240, 0, 0.039), which only lines up with the box while the base is
        exactly where it spawned. But the base is a free body on wheels: swinging the
        arm pushes it back slightly, and the wiggle test pushes it a lot. A few
        millimetres of drift per trial and the arm is no longer aiming at the box —
        so a later trial would fail for a reason that has nothing to do with the grip.

        Resetting both the box AND the robot makes every trial start from an identical
        state, which is what makes the success rate a number worth quoting.
        """
        yaw = float(self._p('robot_yaw'))
        st = EntityState()
        st.name = 'mbot'
        st.pose.position = Point(x=float(self._p('robot_x')),
                                 y=float(self._p('robot_y')),
                                 z=0.0)
        # yaw about Z only
        st.pose.orientation.z = math.sin(yaw / 2.0)
        st.pose.orientation.w = math.cos(yaw / 2.0)
        # Explicitly zero the twist: a teleport that keeps the old velocity makes the
        # base creep away again the moment physics resumes.
        st.twist.linear.x = st.twist.linear.y = st.twist.linear.z = 0.0
        st.twist.angular.x = st.twist.angular.y = st.twist.angular.z = 0.0
        st.reference_frame = 'world'
        req = SetEntityState.Request()
        req.state = st
        if not self._set_state.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/set_entity_state unavailable')
            return False
        fut = self._set_state.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        ok = fut.done() and fut.result() is not None and fut.result().success
        if not ok:
            self.get_logger().warn('could not reset the robot pose')
        return ok

    def reset_box_to_table(self):
        """Put the box back on the table at its exact known spot.

        NOTE THE DIRECTION. An earlier version of this bench teleported the box to
        wherever the gripper happened to be. That is wrong: if the arm has not reached
        the table yet (or the pose failed), the box is placed in mid-air and simply
        falls to the floor — which is exactly what was observed. The box's home is a
        fixed point ON THE TABLE, and it is the ARM that must travel to the box.
        """
        x = float(self._p('box_world_x'))
        y = float(self._p('box_world_y'))
        z = float(self._p('box_world_z'))
        st = EntityState()
        st.name = self._p('box_name')
        st.pose.position = Point(x=x, y=y, z=z)
        st.pose.orientation.w = 1.0
        # Explicitly zero the twist -- same reason as the robot reset above: a
        # teleport that keeps the box's old velocity (from being shoved/dropped in
        # the previous trial) launches it again the instant physics resumes, right
        # next to the arm. Missing here was the likely cause of the box "flying"
        # between trials. Diagnosed 2026-08-11.
        st.twist.linear.x = st.twist.linear.y = st.twist.linear.z = 0.0
        st.twist.angular.x = st.twist.angular.y = st.twist.angular.z = 0.0
        st.reference_frame = 'world'
        req = SetEntityState.Request()
        req.state = st
        if not self._set_state.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/set_entity_state unavailable')
            return False
        fut = self._set_state.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        ok = fut.done() and fut.result() is not None and fut.result().success
        if not ok:
            self.get_logger().error(f'failed to reset {st.name}')
        return ok

    def grasp_pose(self):
        """The grasp point in base_link — a fixed, already-calibrated number.

        An earlier version computed this by asking Gazebo for the robot base pose and
        converting the box's world position into base_link. That was unnecessary work
        with a single point of failure: the entity name `mbot::base_link` does not
        exist in Gazebo (URDF->SDF folds fixed-joint links into their parent), so the
        lookup returned nothing and the whole bench aborted before moving the arm.

        The robot is spawned pre-docked and never drives, so the grasp point in its own
        frame is a constant. It comes from the docked geometry already measured for
        calib_pick.py: box at base_link x=0.240, z=-0.021, grasped GRASP_ABOVE (0.06)
        higher.
        """
        return (float(self._p('pose_x')),
                float(self._p('pose_y')),
                float(self._p('pose_z')))

    def report_capture(self):
        """Print the current arm joint angles ready to paste into the URDF.

        This is how the arm's SPAWN pose is calibrated. Once these six numbers are in
        mycobot_ros2_control.xacro as each joint's <initial_value>, the arm is born at
        the table with the box already between its fingers — no approach motion, no
        planning, no waiting on MoveIt.
        """
        missing = [j for j in ARM_JOINTS if j not in self._arm_actual]
        if missing:
            self.get_logger().error(
                f'no /joint_states reading yet for: {", ".join(missing)}')
            return False

        # Prove the arm is actually AT the box before these angles are trusted.
        # MoveIt reporting success only means it reached the commanded base_link point;
        # it says nothing about whether that point is where the box is. Compare the two
        # in world coordinates, which is ground truth for both.
        tool = self._entity_pose('mbot::gripper_base')
        box = self._entity_pose(self._p('box_name'))
        if tool is not None and box is not None:
            dx, dy, dz = tool.x - box.x, tool.y - box.y, tool.z - box.z
            lateral_mm = 1000.0 * (dx * dx + dy * dy) ** 0.5
            self.get_logger().info(
                f'[capture] tool  world ({tool.x:+.4f}, {tool.y:+.4f}, {tool.z:+.4f})\n'
                f'[capture] box   world ({box.x:+.4f}, {box.y:+.4f}, {box.z:+.4f})\n'
                f'[capture] tool is {lateral_mm:.1f} mm sideways from the box centre, '
                f'and {dz * 1000:+.1f} mm above it')
            if lateral_mm > 15.0:
                self.get_logger().error(
                    f'[capture] REJECTED — the tool is {lateral_mm:.0f} mm off to the '
                    f'side. The fingers are NOT around the box, so these angles are '
                    f'useless as a spawn pose. Fix pose_x/pose_y first.')
                return False
            self.get_logger().info('[capture] tool is over the box — angles look sane.')
        else:
            self.get_logger().warn(
                '[capture] could not verify against Gazebo — trust these angles only '
                'after looking at the simulation.')
        lines = '\n'.join(
            f'  {j:<24}{self._arm_actual[j]:+.4f}' for j in ARM_JOINTS)
        self.get_logger().info(
            '\n╔═══ CAPTURED GRASP POSE ════════════════════════════════════\n'
            '║ Paste each value into the matching joint\'s <initial_value>\n'
            '║ in limo_car/gazebo/mycobot_ros2_control.xacro:\n'
            f'{lines}\n'
            '╚════════════════════════════════════════════════════════════')
        return True

    # ── gripper ──────────────────────────────────────────────────────────────

    def _publish_grip(self, t):
        """Command all 6 finger joints at interpolation factor t (0 open, 1 closed)."""
        msg = Float64MultiArray()
        msg.data = [float(o + t * (c - o))
                    for o, c in zip(GRIPPER_OPEN, GRIPPER_GRASP)]
        self._grip.publish(msg)
        self._last_grip = msg.data[:]

    def set_gripper(self, target, label, duration=0.8, steps=20):
        start = self._last_grip[:]
        for i in range(1, steps + 1):
            a = i / steps
            msg = Float64MultiArray()
            msg.data = [float(s + a * (v - s)) for s, v in zip(start, target)]
            self._grip.publish(msg)
            rclpy.spin_once(self, timeout_sec=duration / steps)
        self._last_grip = list(target)
        self.get_logger().info(f'gripper → {label}')
        time.sleep(0.3)


    def measure_tcp_offset(self):
        """Where are the FINGERTIPS relative to gripper_tcp? This calibrates the TCP.

        Kept as a REGRESSION CHECK. gripper_tcp used to sit at zero offset from
        gripper_base (the palm), so MoveIt aimed the palm at the target and the fingers
        landed an unknown distance beyond — an error GRASP_ABOVE = 0.06 silently
        absorbed. Since the 2026-08-01 calibration the TCP is the grasp point itself,
        so this now reads roughly (0, -0.035, 0) m: the knuckles sit ~35 mm BACK up the
        finger axis from the TCP. If that number drifts, the gripper geometry changed.

        Measured from TF, not Gazebo: gripper_base and gripper_tcp are BOTH folded into
        joint6_flange by the URDF->SDF conversion and cannot be queried from Gazebo at
        all, but robot_state_publisher publishes every one of them as a TF frame.

        Returns the midpoint of the two fingertip links expressed in gripper_tcp
        coordinates, or None. The dominant component is the offset to apply.
        """
        try:
            l = self._tf_buf.lookup_transform(
                TCP_LINK, 'gripper_left1', rclpy.time.Time()).transform.translation
            r = self._tf_buf.lookup_transform(
                TCP_LINK, 'gripper_right1', rclpy.time.Time()).transform.translation
        except Exception as e:
            self.get_logger().warn(f'[tcp] TF unavailable: {e}')
            return None
        mid = ((l.x + r.x) / 2.0, (l.y + r.y) / 2.0, (l.z + r.z) / 2.0)
        self.get_logger().info(
            f'[tcp] left1  ({l.x:+.4f}, {l.y:+.4f}, {l.z:+.4f})  '
            f'right1 ({r.x:+.4f}, {r.y:+.4f}, {r.z:+.4f})')
        self.get_logger().info(
            f'[tcp] fingertip MIDPOINT relative to {TCP_LINK}: '
            f'({mid[0] * 1000:+.1f}, {mid[1] * 1000:+.1f}, {mid[2] * 1000:+.1f}) mm')
        return mid

    def full_reset(self):
        """Put EVERYTHING back: fingers open, arm home, robot re-docked, box on the table.

        The arm matters as much as the box. A failed plan leaves the arm in whatever
        configuration it stopped in, and MoveIt finds it much harder to plan out of an
        awkward pose than out of a neutral one — so without this, each failed attempt makes
        the next one less likely to succeed, and a distance can look unreachable when it is
        only unreachable *from where the arm happens to be*.
        """
        self.set_gripper(GRIPPER_OPEN, 'open')
        self.go_joints(READY_J, 'home')
        self.reset_robot_to_spawn()
        self.reset_box_to_table()
        px, py, pz = self.grasp_pose()
        ok = self.go_pose(px, py, pz, 'back to the grasp pose')
        print(f'      full reset done — arm {"at the box" if ok else "COULD NOT reach the box"}')
        return ok

    def report_box_distance(self):
        """Print the TRUE robot-to-box distance, straight from Gazebo.

        Printed every trial so a manual distance sweep is self-documenting: whatever
        robot_y / pose_x were set to, this is what the geometry actually became.
        """
        box = self._entity_pose(self._p('box_name'))
        if box is None:
            return
        rx, ry = float(self._p('robot_x')), float(self._p('robot_y'))
        d = ((box.x - rx) ** 2 + (box.y - ry) ** 2) ** 0.5
        self.get_logger().info(
            f'[dist] robot at ({rx:.3f}, {ry:.3f})  box at ({box.x:.3f}, {box.y:.3f})  '
            f'--> TRUE DISTANCE {d:.3f} m   (arm limit ~0.24, camera needs ~0.35)')

    def close_and_watch(self):
        """Step-close the fingers while sampling both contact sensors.

        Returns (stall_angle_or_None, l_frames, r_frames, both_frames, samples).

        The contact counters are sampled once per close step (~12.5 Hz) rather than in
        the sensor callback, so that "both fingers touching AT THE SAME TIME" is a
        meaningful measurement instead of two independent message counts.
        """
        floor  = float(self._p('close_floor'))
        cmd    = self._last_grip[0]
        step_s = max(0.02, float(self._p('close_sec')) /
                     max(1.0, (GRIPPER_OPEN[0] - floor) / CLOSE_STEP_RAD))

        lag_run = 0
        stall = None
        l_frames = r_frames = both_frames = samples = 0

        while cmd > floor:
            cmd = max(floor, cmd - CLOSE_STEP_RAD)
            t = (GRIPPER_OPEN[0] - cmd) / (GRIPPER_OPEN[0] - GRIPPER_GRASP[0])
            self._publish_grip(t)
            rclpy.spin_once(self, timeout_sec=step_s)

            samples += 1
            if self._l_touch:
                l_frames += 1
            if self._r_touch:
                r_frames += 1
            if self._l_touch and self._r_touch:
                both_frames += 1

            if stall is None:
                lag = self._grip_actual - cmd
                if lag > LAG_THRESH:
                    lag_run += 1
                    if lag_run >= STALL_STEPS:
                        stall = self._grip_actual
                        self.get_logger().info(
                            f'[close] stalled at {stall:+.3f} rad (cmd {cmd:+.3f})')
                else:
                    lag_run = 0

        if stall is None:
            self.get_logger().warn(
                '[close] swept to the floor with no stall — nothing resisted')
        return stall, l_frames, r_frames, both_frames, samples

    # ── MoveIt ───────────────────────────────────────────────────────────────

    def _send(self, constraints, label):
        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = ARM_GROUP
        req.goal_constraints = [constraints]
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = VEL_SCALE
        req.max_acceleration_scaling_factor = ACC_SCALE
        goal.request = req
        fut = self._move.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        if not fut.done() or not fut.result().accepted:
            self.get_logger().error(f'[{label}] rejected')
            return False
        r = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self, r, timeout_sec=30.0)
        ok = r.result().result.error_code.val == MOVEIT_SUCCESS
        if not ok:
            self.get_logger().error(f'[{label}] FAILED')
        return ok

    def go_joints(self, joints, label):
        c = Constraints()
        for name, val in zip(ARM_JOINTS, joints):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(val)
            jc.tolerance_above = 0.02
            jc.tolerance_below = 0.02
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        return self._send(c, label)

    def wiggle_while_holding(self):
        """Swing and roll the wrist while the box is held, so the coupling is VISIBLE.

        This is the eye test for the whole switch from the kinematic weld to a real
        fixed joint. The weld copied POSITION ONLY — the box's orientation was frozen
        at pickup, so the dome kept pointing up no matter how the wrist turned. A real
        joint couples all six degrees of freedom, so the box must turn with the hand.

        Driven in JOINT space, deliberately: it starts from the arm's current angles and
        nudges one joint at a time, so every waypoint is reachable by construction and
        no IK can fail halfway through and abort the trial.
        """
        base = [self._arm_actual.get(j, 0.0) for j in ARM_JOINTS]
        if len(base) != len(ARM_JOINTS):
            self.get_logger().warn('[wiggle] no joint readings — skipping')
            return
        moves = [
            ('roll wrist right', 5, +0.9),
            ('roll wrist left',  5, -0.9),
            ('tilt wrist down',  4, +0.5),
            ('tilt wrist up',    4, -0.5),
            ('swing base right', 0, +0.35),
            ('swing base left',  0, -0.35),
        ]
        self.get_logger().info('[wiggle] watch the box — it should turn WITH the hand')
        for label, idx, delta in moves:
            j = list(base)
            j[idx] = base[idx] + delta
            self.go_joints(j, f'[wiggle] {label}')
            time.sleep(0.4)
        self.go_joints(base, '[wiggle] back to the lift pose')

    def go_pose(self, x, y, z, label, pos_tol=None, ori_tol=0.1):
        # Radius of the sphere MoveIt must land the TCP inside. Live-tunable so the
        # tolerance itself can be swept: `ros2 param set /grip_bench pos_tol 0.005`.
        if pos_tol is None:
            pos_tol = float(self._p('pos_tol'))
        tg = Pose()
        tg.position = Point(x=float(x), y=float(y), z=float(z))
        q = TOPDOWN_QUAT
        tg.orientation.x, tg.orientation.y = float(q[0]), float(q[1])
        tg.orientation.z, tg.orientation.w = float(q[2]), float(q[3])

        pc = PositionConstraint()
        pc.header.frame_id = PLANNING_FRAME
        pc.link_name = TCP_LINK
        bv = BoundingVolume()
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [pos_tol]
        bv.primitives.append(sp)
        bv.primitive_poses.append(tg)
        pc.constraint_region = bv
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = PLANNING_FRAME
        oc.link_name = TCP_LINK
        oc.orientation = tg.orientation
        oc.absolute_x_axis_tolerance = ori_tol
        oc.absolute_y_axis_tolerance = ori_tol
        oc.absolute_z_axis_tolerance = ori_tol
        oc.weight = 1.0

        c = Constraints()
        c.position_constraints.append(pc)
        c.orientation_constraints.append(oc)
        return self._send(c, label)

    # ── one trial ────────────────────────────────────────────────────────────

    def one_trial(self, n, _unused=None):
        """One grasp cycle, arm already parked at the grasp pose:

            open fingers → box appears between them → close → lift → hold →
            come back down → open → box is back on the table.

        The arm never returns to 'ready' and never re-plans an approach. Only the
        lift and the descent move, so a trial costs seconds instead of minutes.
        """
        # Re-read the grasp pose EVERY trial. pose_x/pose_y/pose_z and robot_x/robot_y
        # are live ROS parameters, so the docking distance can be changed with
        # `ros2 param set` between trials and the next one runs at the new distance —
        # no relaunch. That is what makes this usable for a manual distance sweep.
        px, py, pz = self.grasp_pose()
        lift = float(self._p('lift_m'))

        self.get_logger().info(
            f'\n┌── Trial {n} ───────────────────────────────────\n'
            f'│  close_sec  {float(self._p("close_sec")):.2f}   '
            f'floor {float(self._p("close_floor")):+.3f}   '
            f'backoff {bool(self._p("backoff"))}   '
            f'lift {lift:.2f} m\n'
            f'└───────────────────────────────────────────────')

        # 1. Open the fingers. Under the physics grasp plugin this also RELEASES
        #    anything still held from the previous trial.
        self.set_gripper(GRIPPER_OPEN, 'open')

        # 2. Reset the WHOLE SCENE, not just the box: the robot goes back to its docked
        #    spawn pose and the box back to its table spot. Both matter — the grasp point
        #    is fixed in base_link, so a base that has drifted means the arm is no longer
        #    aiming at the box, and the trial would fail for the wrong reason.
        self.reset_robot_to_spawn()
        if not self.reset_box_to_table():
            return None
        time.sleep(float(self._p('settle_sec')))

        # Go to the grasp pose for THIS trial. Normally a no-op because the arm is
        # already there, but if pose_x was changed live it moves to the new distance —
        # and if the new distance is out of reach, it says so instead of failing quietly.
        if not self.go_pose(px, py, pz, f'grasp pose (trial {n})'):
            # Retry from a neutral configuration before declaring it unreachable. A failed
            # plan usually means "not reachable FROM HERE", not "not reachable at all".
            self.get_logger().warn(
                '[grasp pose] failed — returning the arm home and retrying from there')
            self.go_joints(READY_J, 'home (recovery)')
            if not self.go_pose(px, py, pz, f'grasp pose retry (trial {n})'):
                self.get_logger().error(
                    f'CANNOT REACH x={px:.3f} even from the home configuration — '
                    f'genuinely outside the arm envelope.')
                return None

        before = self._entity_pose(self._p('box_name'))
        if before is None:
            self.get_logger().error('lost the box after resetting it')
            return None
        bx0, by0, bz0 = before.x, before.y, before.z

        # 3. Close, watching both finger contact sensors.
        self.report_box_distance()

        stall, lf, rf, bf, samples = self.close_and_watch()

        # Settle before the next MoveGroup plan. close_and_watch() ends the moment
        # the gripper stalls/finishes, but the ARM itself may still be micro-settling
        # (residual servo motion, /joint_states lag). The lift request below plans
        # from a "current state" snapshot; if that snapshot is stale by even a
        # fraction of a degree, MoveIt's execution-time check (0.01 rad tolerance)
        # rejects the whole trajectory and it silently retries at a different
        # clearance -- which looks like the arm "adapting"/wobbling for no reason
        # right after the grasp closes. Diagnosed 2026-08-11 from an actual reject:
        # "expected: -1.2959, current: -1.27459" on joint2_to_joint1.
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)
        time.sleep(0.3)

        # 4. Optional one-step back-off (suspect 4).
        if bool(self._p('backoff')):
            j_back = self._last_grip[0] + CLOSE_STEP_RAD
            t_back = max(0.0, (GRIPPER_OPEN[0] - j_back) /
                              (GRIPPER_OPEN[0] - GRIPPER_GRASP[0]))
            self._publish_grip(t_back)
            rclpy.spin_once(self, timeout_sec=0.15)

        # 5. How far did the closing fingers shove the box sideways?
        mid = self._entity_pose(self._p('box_name'))
        shove_mm = 0.0
        if mid is not None:
            shove_mm = 1000.0 * ((mid.x - bx0) ** 2 + (mid.y - by0) ** 2) ** 0.5

        # 6. Lift, hold, and ask Gazebo whether the box came with it.
        # Raising the table cost vertical headroom: from a higher box, a full 10 cm lift
        # can land outside the arm envelope and the whole trial fails on a PLANNING
        # error rather than on the grip — which is not what this bench is measuring.
        # Fall back to progressively smaller lifts so the grip still gets judged. Any
        # lift comfortably above min_rise is enough to prove the box left the table.
        lifted = False
        for frac in (1.0, 0.7, 0.5):
            attempt_lift = lift * frac
            if attempt_lift < float(self._p('min_rise')) * 1.2:
                break
            if self.go_pose(px, py, pz + attempt_lift,
                            f'lift {attempt_lift:.3f} m (trial {n})'):
                lifted = True
                if frac < 1.0:
                    self.get_logger().warn(
                        f'[lift] full {lift:.2f} m was out of reach — used '
                        f'{attempt_lift:.2f} m instead. Headroom is tight at this '
                        f'table height.')
                break
        time.sleep(float(self._p('hold_sec')))
        after = self._entity_pose(self._p('box_name'))
        rise = (after.z - bz0) if after is not None else float('nan')
        held = bool(lifted and after is not None
                    and rise >= float(self._p('min_rise')))

        # 6a. While it is still in the air: measure where the box actually sits
        #     relative to the palm. This calibrates gripper_tcp (see measure_tcp_offset).
        tcp_jaw = tcp_along = tcp_slide = float('nan')
        if held:
            m = self.measure_tcp_offset()
            if m is not None:
                tcp_jaw, tcp_along, tcp_slide = m

        # 6b. Optional: rotate the wrist around while holding, to SEE the box turn with
        #     it. Only meaningful if we actually have the box.
        held_after_wiggle = held
        if held and bool(self._p('wiggle')):
            self.wiggle_while_holding()
            post = self._entity_pose(self._p('box_name'))
            held_after_wiggle = bool(post is not None
                                     and (post.z - bz0) >= float(self._p('min_rise')))
            self.get_logger().info(
                f'[wiggle] after shaking it around: '
                f'{"STILL HELD ✓" if held_after_wiggle else "LOST IT ✗"}')

        # 7. Put it back down where it came from and let go — ready for the next trial.
        self.go_pose(px, py, pz, f'return to table (trial {n})')
        self.set_gripper(GRIPPER_OPEN, 'release')

        self.get_logger().info(
            f'\n└── Trial {n}: {"HELD ✓" if held else "DROPPED ✗"}   '
            f'rise {rise:+.4f} m   shoved {shove_mm:.1f} mm\n'
            f'    contact during close: left {lf}/{samples}  right {rf}/{samples}  '
            f'BOTH {bf}/{samples}   stall '
            f'{f"{stall:+.3f}" if stall is not None else "none"}')

        # Reading the contact counters, in words — but ONLY if the sensors are alive.
        # The finger bumper sensors have reported 0 in every trial ever run, including
        # trials that gripped successfully, so they are broken (almost certainly the same
        # wrong-name class of bug as the grasp plugin's palm link: the sensor references
        # collision "col", which may not be what Gazebo scopes it to). Until that is fixed
        # these counters must NOT be used to draw conclusions — saying "the fingers never
        # touched the box" on the strength of a dead sensor sends debugging in exactly the
        # wrong direction.
        if samples and (lf or rf):
            if bf == 0:
                self.get_logger().warn(
                    '    -> only ONE finger ever touched. The box is being pushed aside '
                    'instead of clamped.')
            elif bf < 0.25 * samples:
                self.get_logger().warn(
                    '    -> both fingers touched, but only briefly. Contact is marginal.')
        elif samples:
            self.get_logger().info(
                '    (contact sensors reported nothing — they are known broken, so this '
                'says NOTHING about whether the fingers touched. Judge by rise_m.)')

        # The honest signal, from Gazebo ground truth rather than a dead sensor:
        if shove_mm > 6.0:
            self.get_logger().warn(
                f'    -> the box was SHOVED {shove_mm:.1f} mm. The gripper is hitting it '
                f'rather than closing around it — an approach/alignment problem.')

        return {
            'trial': n,
            'held': held,
            'held_after_wiggle': held_after_wiggle,
            'rise_m': round(rise, 5),
            'shove_mm': round(shove_mm, 2),
            'tcp_jaw_mm': round(1000 * tcp_jaw, 2),
            'tcp_along_mm': round(1000 * tcp_along, 2),
            'tcp_slide_mm': round(1000 * tcp_slide, 2),
            'stall': None if stall is None else round(stall, 4),
            'l_frames': lf,
            'r_frames': rf,
            'both_frames': bf,
            'samples': samples,
            'close_sec': float(self._p('close_sec')),
            'close_floor': float(self._p('close_floor')),
            'backoff': bool(self._p('backoff')),
            'pose_x': float(self._p('pose_x')),
            'pose_y': float(self._p('pose_y')),
            'pose_z': float(self._p('pose_z')),
            'lift_m': float(self._p('lift_m')),
        }

    # ── run ──────────────────────────────────────────────────────────────────

    def interactive_loop(self):
        """Type commands, watch the robot. One sim, no relaunching.

        The automated trial loop answers "what is the success rate". This answers
        "what does it look like at 31 cm", which is a different and equally useful
        question — and one that is much better judged with your own eyes than by a
        number in a CSV.
        """
        print('\n' + '=' * 62)
        print('  GRIP BENCH — INTERACTIVE')
        print('=' * 62)
        print('  w        move the robot 1 cm CLOSER to the box')
        print('  s        move the robot 1 cm FURTHER from the box')
        print('  <number> jump straight to that distance,  e.g.  0.31')
        print('  p        PICK — arm goes down, closes, lifts, reports')
        print('  r        FULL reset: arm home, robot re-docked, box back, fingers open')
        print('  h        send the arm home only (recover from a failed plan)')
        print('  d        print the true distance')
        print('  q        quit')
        print('-' * 62)
        print('  measured arm limit  ~0.24 m')
        print('  camera needs        ~0.35 m to see the whole box')
        print('=' * 62 + '\n')

        self.set_gripper(GRIPPER_OPEN, 'open')
        self.reset_robot_to_spawn()
        self.reset_box_to_table()

        while True:
            d = float(self._p('pose_x'))
            try:
                cmd = input(f'  [{d:.3f} m] > ').strip().lower()
            except (EOFError, KeyboardInterrupt):
                break
            if cmd == 'q':
                break

            if cmd in ('w', 's'):
                step = -0.01 if cmd == 'w' else +0.01
                d = round(d + step, 3)
                cmd = str(d)          # fall through to the jump handler

            try:                       # a bare number = jump to that distance
                d_new = float(cmd)
                self.set_parameters([
                    rclpy.parameter.Parameter('pose_x', value=float(d_new)),
                    rclpy.parameter.Parameter('robot_y', value=float(4.0 + d_new)),
                ])
                self.reset_robot_to_spawn()
                self.reset_box_to_table()
                print(f'      robot repositioned — box is now {d_new:.3f} m away')
                self.report_box_distance()
                continue
            except ValueError:
                pass

            if cmd == 'd':
                self.report_box_distance()
            elif cmd == 'r':
                self.full_reset()
            elif cmd == 'h':
                self.go_joints(READY_J, 'home')
                print('      arm returned to the ready configuration')
            elif cmd == 'p':
                row = self.one_trial(len(self._rows) + 1)
                if row is None:
                    print('      >>> PICK FAILED — see the message above')
                else:
                    self._rows.append(row)
                    print(f'      >>> {"HELD" if row["held"] else "DROPPED"}   '
                          f'rise {row["rise_m"]:+.4f} m   shoved {row["shove_mm"]:.1f} mm')
            elif cmd:
                print('      ? use w / s / a number / p / r / d / q')

        self._write_csv()
        print('\n  bye\n')

    def run(self):
        self.get_logger().info('waiting for move_group…')
        if not self._move.wait_for_server(timeout_sec=90.0):
            self.get_logger().error('move_group never came up — aborting')
            return
        for cli, name in ((self._get_state, '/get_entity_state'),
                          (self._set_state, '/set_entity_state')):
            if not cli.wait_for_service(timeout_sec=30.0):
                self.get_logger().error(f'{name} never came up — aborting')
                return
        self.get_logger().info('move_group + Gazebo services ready')

        # ── Park the arm at the grasp pose ──────────────────────────────────
        # Done ONCE. Everything after this is close / lift / down / open, so a trial
        # costs seconds. Once the captured angles are baked into the URDF's
        # <initial_value> entries the arm already spawns here and this move is a no-op.
        if bool(self._p('interactive')):
            self.interactive_loop()
            return

        self.set_gripper(GRIPPER_OPEN, 'open')
        self.reset_box_to_table()
        px, py, pz = self.grasp_pose()
        self.get_logger().info(
            f'grasp pose (base_link): ({px:.3f}, {py:.3f}, {pz:.3f})')
        # Approach from above, but do NOT abort if the full clearance is unreachable.
        # At a longer dock the arm has little vertical headroom (measured: reach falls to
        # 0.22 m at base_link z=+0.10), so demanding lift_m of clearance here fails the
        # whole run before a single grasp is attempted — which is what happened at
        # dist:=0.28. Try decreasing clearances, then just go straight to the grasp pose.
        approached = False
        for clearance in (float(self._p('lift_m')), 0.06, 0.03):
            if self.go_pose(px, py, pz + clearance, f'approach from {clearance:.2f} m above'):
                approached = True
                break
        if not approached:
            self.get_logger().warn(
                'no clearance above the box was reachable — descending directly')
        if not self.go_pose(px, py, pz, 'park at grasp pose'):
            self.get_logger().error('cannot reach the grasp pose — aborting')
            return

        # ── capture_pose mode: print the six angles for the URDF, then stop ──
        if bool(self._p('capture_pose')):
            for _ in range(20):          # let /joint_states settle at the new pose
                rclpy.spin_once(self, timeout_sec=0.1)
            self.report_capture()
            self.get_logger().info(
                'capture_pose: LOOK AT GAZEBO NOW — the open fingers must be '
                'straddling the box. If they are not, these angles are wrong.')
            time.sleep(3.0)
            return

        self.get_logger().info(
            'parked at the box. The fingers are open around it; trials start now.')

        if bool(self._p('interactive')):
            self.interactive_loop()
            return

        loops = int(self._p('loops'))
        held_count = 0
        for n in range(1, loops + 1):
            row = self.one_trial(n)
            if row is None:
                continue
            self._rows.append(row)
            held_count += int(row['held'])
            self.get_logger().info(
                f'   running score: {held_count}/{len(self._rows)} held')

        self._write_csv()
        if self._rows:
            n = len(self._rows)
            self.get_logger().info(
                f'\n╔═══ GRIP BENCH DONE ════════════════════════════\n'
                f'║  held {held_count}/{n}  ({100.0 * held_count / n:.0f} %)\n'
                f'║  mean shove {sum(r["shove_mm"] for r in self._rows) / n:.1f} mm\n'
                f'║  trials with BOTH fingers in contact: '
                f'{sum(1 for r in self._rows if r["both_frames"] > 0)}/{n}\n'
                f'╚════════════════════════════════════════════════')

    def _write_csv(self):
        if not self._rows:
            return
        path = self._p('csv')
        if not path:
            data_dir = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                '..', '..', '..', '..', '..', 'data')
            os.makedirs(data_dir, exist_ok=True)
            path = os.path.join(
                data_dir, f'gripbench_{time.strftime("%Y%m%d_%H%M%S")}.csv')
        with open(path, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(self._rows[0].keys()))
            w.writeheader()
            w.writerows(self._rows)
        self.get_logger().info(f'wrote {len(self._rows)} trials → {path}')


def main(args=None):
    rclpy.init(args=args)
    node = GripBench()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('interrupted — writing what we have')
        node._write_csv()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

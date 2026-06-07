#!/usr/bin/env python3
"""
LimoCobotBridge — State-machine bridge between the LIMO mobile base
and the myCobot arm + perception pipeline from automaticaddison/mycobot_ros2.

Flow:
  INIT → DOCK → NAV_TO_PICK → DETECT → PICK → NAV_TO_PLACE → PLACE → HOME → COMPLETE

Perception modes (auto-detected):
  1. MTC mode: calls mycobot_interfaces/GetPlanningScene service (C++ point-cloud server)
  2. Fallback mode: depth-image-based detection with MoveIt IK + trajectory execution
"""

import sys, time, math, threading, os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from enum import Enum, auto
import numpy as np
import cv2
from cv_bridge import CvBridge
from dataclasses import dataclass
from typing import Optional, Tuple

from geometry_msgs.msg import Twist, Pose, Point, Quaternion, PoseStamped
from sensor_msgs.msg import Image, JointState
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState, CollisionObject
from moveit_msgs.msg import PlanningScene, PlanningSceneWorld
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from builtin_interfaces.msg import Duration

from ament_index_python.packages import get_package_share_directory

# ─── Constants ──────────────────────────────────────────────────────────────

ARM_JOINTS = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
              'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
GRIPPER_JOINT = ['gripper_controller']

JOINT_LIMITS = [(-2.9321, 2.9321), (-2.4434, 2.4434), (-2.6179, 2.6179),
                (-2.6179, 2.6179), (-2.7052, 2.7925), (-3.14159, 3.14159)]

HOME_POSE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
READY_POSE = [0.0, -0.5, -1.2, 1.7, 0.0, 0.0]

# ─── State ──────────────────────────────────────────────────────────────────

class State(Enum):
    INIT      = auto()
    DOCK      = auto()
    NAV_TO_PICK    = auto()
    DETECT         = auto()
    PICK           = auto()
    NAV_TO_PLACE   = auto()
    PLACE          = auto()
    HOME           = auto()
    COMPLETE       = auto()
    ABORT          = auto()

# ─── Data ───────────────────────────────────────────────────────────────────

@dataclass
class DetectedObject:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0
    shape: str = "box"
    dimensions: Tuple[float,float,float] = (0.03, 0.03, 0.03)
    in_arm_frame: bool = False

# ─── Quaternion helpers ─────────────────────────────────────────────────────

def euler_to_quat(roll, pitch, yaw):
    cy, sy = math.cos(yaw*0.5), math.sin(yaw*0.5)
    cp, sp = math.cos(pitch*0.5), math.sin(pitch*0.5)
    cr, sr = math.cos(roll*0.5), math.sin(roll*0.5)
    return Quaternion(w=cr*cp*cy+sr*sp*sy, x=sr*cp*cy-cr*sp*sy,
                      y=cr*sp*cy+sr*cp*sy, z=cr*cp*sy-sr*sp*cy)

def quat_to_euler(q) -> Tuple[float,float,float]:
    t0 = 2.0*(q.w*q.x + q.y*q.z); t1 = 1.0 - 2.0*(q.x*q.x + q.y*q.y)
    roll = math.atan2(t0, t1)
    t2 = 2.0*(q.w*q.y - q.z*q.x); t2 = max(-1.0, min(1.0, t2))
    pitch = math.asin(t2)
    t3 = 2.0*(q.w*q.z + q.x*q.y); t4 = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw

def normalize_angle(a):
    while a > math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a

# ─── Main Bridge ────────────────────────────────────────────────────────────

class LimoCobotBridge(Node):
    def __init__(self):
        super().__init__('limo_cobot_bridge')
        self.cbg = ReentrantCallbackGroup()

        # ── Parameter Declarations ──
        self.declare_parameter('pick_zone_x', 0.70)
        self.declare_parameter('pick_zone_y', -0.22)
        self.declare_parameter('place_zone_x', 0.78)
        self.declare_parameter('place_zone_y', 0.22)
        self.declare_parameter('home_zone_x', 0.0)
        self.declare_parameter('home_zone_y', -0.22)
        self.declare_parameter('nav_speed', 0.15)
        self.declare_parameter('pick_approach_speed', 0.12)
        self.declare_parameter('grasp_approach_offset', 0.08)
        self.declare_parameter('grasp_lift_offset', 0.15)
        self.declare_parameter('grasp_roll', 0.0)
        self.declare_parameter('grasp_pitch', 1.5708)
        self.declare_parameter('grasp_yaw', 0.0)
        self.declare_parameter('object_length', 0.03)
        self.declare_parameter('object_width', 0.03)
        self.declare_parameter('object_height', 0.03)
        self.declare_parameter('target_shape', 'box')
        self.declare_parameter('target_dimensions', [0.03, 0.03, 0.03])
        self.declare_parameter('gripper_open', 0.0)
        self.declare_parameter('gripper_closed', -0.7)
        self.declare_parameter('ik_solver_timeout', 0.5)
        self.declare_parameter('ik_avoid_collisions', True)

        # ── Retrieve Parameter Values ──
        self.pick_zone_x = self.get_parameter('pick_zone_x').value
        self.pick_zone_y = self.get_parameter('pick_zone_y').value
        self.place_zone_x = self.get_parameter('place_zone_x').value
        self.place_zone_y = self.get_parameter('place_zone_y').value
        self.home_zone_x = self.get_parameter('home_zone_x').value
        self.home_zone_y = self.get_parameter('home_zone_y').value
        self.nav_speed = self.get_parameter('nav_speed').value
        self.pick_approach_speed = self.get_parameter('pick_approach_speed').value
        self.grasp_approach_offset = self.get_parameter('grasp_approach_offset').value
        self.grasp_lift_offset = self.get_parameter('grasp_lift_offset').value
        self.grasp_roll = self.get_parameter('grasp_roll').value
        self.grasp_pitch = self.get_parameter('grasp_pitch').value
        self.grasp_yaw = self.get_parameter('grasp_yaw').value
        self.object_length = self.get_parameter('object_length').value
        self.object_width = self.get_parameter('object_width').value
        self.object_height = self.get_parameter('object_height').value
        self.target_shape = self.get_parameter('target_shape').value
        self.target_dimensions = self.get_parameter('target_dimensions').value
        self.gripper_open = self.get_parameter('gripper_open').value
        self.gripper_closed = self.get_parameter('gripper_closed').value
        self.ik_solver_timeout = self.get_parameter('ik_solver_timeout').value
        self.ik_avoid_collisions = self.get_parameter('ik_avoid_collisions').value

        # ── State ──
        self.state = State.INIT
        self.obj: Optional[DetectedObject] = None
        self.start_time = self.get_clock().now()
        self.running = True
        self.model_name = "pick_box"

        # ── Odometry ──
        self.odom_pose: Optional[Pose] = None
        self.odom_yaw = 0.0
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10, callback_group=self.cbg)

        # ── Cmd vel ──
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10, callback_group=self.cbg)

        # ── Joint states ──
        self.joint_states: Optional[JointState] = None
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10, callback_group=self.cbg)

        # ── Trajectory action clients ──
        self.arm_act = ActionClient(self, FollowJointTrajectory,
                                    '/arm_controller/follow_joint_trajectory',
                                    callback_group=self.cbg)
        self.grip_act = ActionClient(self, FollowJointTrajectory,
                                     '/mycobot_gripper_controller/follow_joint_trajectory',
                                     callback_group=self.cbg)

        # ── IK service ──
        self.ik_srv = self.create_client(GetPositionIK, '/compute_ik', callback_group=self.cbg)

        # ── Planning scene service ──
        self.ps_srv = self.create_client(ApplyPlanningScene, '/apply_planning_scene',
                                         callback_group=self.cbg)

        # ── Perception service (mycobot_ros2, optional) ──
        self.perception_srv = None
        self.has_mtc_perception = False
        try:
            from mycobot_interfaces.srv import GetPlanningScene
            self.perception_srv = self.create_client(
                GetPlanningScene, '/get_planning_scene_mycobot',
                callback_group=self.cbg)
        except ImportError:
            self.get_logger().info('mycobot_interfaces not available, using fallback perception')

        # ── RGB-D perception (fallback) ──
        self.bridge = CvBridge()
        self.latest_rgb: Optional[Image] = None
        self.latest_depth: Optional[Image] = None
        self.rgb_sub = self.create_subscription(Image, '/rgb/image_raw', self._rgb_cb, 10,
                                                callback_group=self.cbg)
        self.depth_sub = self.create_subscription(Image, '/depth/image_raw', self._depth_cb, 10,
                                                  callback_group=self.cbg)

        # ── Annotated image publisher ──
        self.annot_pub = self.create_publisher(Image, '/bridge/annotated', 10,
                                               callback_group=self.cbg)

        # ── Background executor ──
        self.bg_executor = MultiThreadedExecutor()
        self.bg_executor.add_node(self)
        self._spin_thread = threading.Thread(target=self.bg_executor.spin, daemon=True)
        self._spin_thread.start()

        self.get_logger().info("LimoCobotBridge initialized")

    # ── Helpers ──────────────────────────────────────────────────────────

    def _odom_cb(self, msg):
        self.odom_pose = msg.pose.pose
        _, _, self.odom_yaw = quat_to_euler(self.odom_pose.orientation)

    def _js_cb(self, msg):
        self.joint_states = msg

    def _rgb_cb(self, msg):
        self.latest_rgb = msg

    def _depth_cb(self, msg):
        self.latest_depth = msg

    def get_joint_pos(self, names: list) -> list:
        if self.joint_states is None:
            return [0.0]*len(names)
        out = []
        for n in names:
            if n in self.joint_states.name:
                out.append(self.joint_states.position[self.joint_states.name.index(n)])
            else:
                out.append(0.0)
        return out

    def wait_for(self, timeout=10.0):
        start = time.time()
        while time.time()-start < timeout:
            if not rclpy.ok():
                return False
            time.sleep(0.05)
        return True

    # ── Base control ────────────────────────────────────────────────────

    def stop_base(self):
        self.cmd_pub.publish(Twist())

    def drive_to_x(self, target_x, speed=0.12, y_line=None, tolerance=0.015, timeout=20.0):
        if self.odom_pose is None:
            self.get_logger().error("No odometry!")
            return False
        start_x = self.odom_pose.position.x
        direction = 1.0 if target_x > start_x else -1.0
        start_sim = self.get_clock().now()

        while rclpy.ok():
            elapsed = (self.get_clock().now()-start_sim).nanoseconds/1e9
            if elapsed > timeout:
                self.stop_base(); return False
            if self.odom_pose is None: continue
            curr_x = self.odom_pose.position.x
            dist = abs(target_x - curr_x)
            if dist < tolerance:
                break
            twist = Twist()
            twist.linear.x = direction * max(min(abs(speed), dist*2.0), 0.08)
            if y_line is not None:
                y_err = y_line - self.odom_pose.position.y
                desired_h = direction * 8.0 * y_err
                desired_h = max(-0.43, min(0.43, desired_h))
                h_err = normalize_angle(desired_h - self.odom_yaw)
                twist.angular.z = max(-0.35, min(0.35, 3.0*h_err))
            self.cmd_pub.publish(twist)
            time.sleep(0.05)
        self.stop_base()
        return True

    def navigate_to(self, tx, ty, speed=0.15, tolerance=0.03, timeout=30.0):
        Kp = 2.0
        start_sim = self.get_clock().now()
        while rclpy.ok():
            elapsed = (self.get_clock().now()-start_sim).nanoseconds/1e9
            if elapsed > timeout:
                self.stop_base(); return False
            if self.odom_pose is None: continue
            dx = tx - self.odom_pose.position.x
            dy = ty - self.odom_pose.position.y
            dist = math.hypot(dx, dy)
            if dist < tolerance:
                break
            th = math.atan2(dy, dx)
            h_err = normalize_angle(th - self.odom_yaw)
            if abs(h_err) > 2.0*math.pi/3.0:
                d = -1.0; h_err = normalize_angle(math.atan2(-dy, -dx) - self.odom_yaw)
            else:
                d = 1.0
            spd = max(0.04, min(speed, dist/0.30*speed))
            twist = Twist()
            twist.linear.x = d * spd
            twist.angular.z = max(-0.45, min(0.45, d*Kp*h_err))
            self.cmd_pub.publish(twist)
            time.sleep(0.05)
        self.stop_base()
        return True

    # ── Arm control ─────────────────────────────────────────────────────

    def solve_ik(self, x, y, z, roll=0.0, pitch=math.pi/2, yaw=0.0,
                 frame_id='base_link', avoid_collisions=None) -> Optional[list]:
        if avoid_collisions is None:
            avoid_collisions = self.ik_avoid_collisions
        req = GetPositionIK.Request()
        req.ik_request.group_name = "arm"
        req.ik_request.avoid_collisions = avoid_collisions
        req.ik_request.ik_link_name = "gripper_tcp"
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position = Point(x=x, y=y, z=z)
        ps.pose.orientation = euler_to_quat(roll, pitch, yaw)
        req.ik_request.pose_stamped = ps

        seeds = []
        seeds.append(READY_POSE)
        seeds.append([0.0, -1.0, -0.8, 1.8, 0.0, 0.0])
        seeds.append([0.0, -0.2, -1.5, 1.7, 0.0, 0.0])
        for _ in range(8):
            seeds.append([np.random.uniform(l,h) for l,h in JOINT_LIMITS])

        for seed in seeds:
            rs = RobotState()
            rs.joint_state.name = ARM_JOINTS
            rs.joint_state.position = seed
            req.ik_request.robot_state = rs
            try:
                res = self.ik_srv.call(req)
            except:
                continue
            if res and res.error_code.val == 1:
                joints = []
                for n in ARM_JOINTS:
                    idx = res.solution.joint_state.name.index(n)
                    joints.append(res.solution.joint_state.position[idx])
                return joints
        return None

    def execute_trajectory(self, joint_names, positions, duration=2.5,
                           is_gripper=False, timeout=10.0):
        client = self.grip_act if is_gripper else self.arm_act
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = Duration(sec=int(duration), nanosec=int((duration%1)*1e9))
        goal.trajectory.points.append(pt)

        if not client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f"{'Gripper' if is_gripper else 'Arm'} action server not ready")
            return False
        gh = client.send_goal(goal)
        if not gh.accepted:
            return False
        res = gh.get_result()
        return res.status == 4

    def move_arm(self, joints, duration=2.5):
        return self.execute_trajectory(ARM_JOINTS, joints, duration)

    def move_gripper(self, pos, duration=1.5):
        return self.execute_trajectory(GRIPPER_JOINT, [pos], duration, is_gripper=True)

    def add_collision_object(self, obj: DetectedObject, frame_id='base_link'):
        req = ApplyPlanningScene.Request()
        sc = PlanningScene()
        sc.is_diff = True
        co = CollisionObject()
        co.id = "target_object"
        co.header.frame_id = frame_id
        co.operation = CollisionObject.ADD
        p = SolidPrimitive()
        p.type = SolidPrimitive.BOX
        p.dimensions = [obj.dimensions[0], obj.dimensions[1], obj.dimensions[2]]
        co.primitives.append(p)
        co.primitive_poses.append(Pose(position=Point(x=obj.x, y=obj.y, z=obj.z),
                                       orientation=euler_to_quat(0,0,obj.yaw)))
        sc.world.collision_objects.append(co)
        req.scene = sc
        try:
            self.ps_srv.call_async(req)
        except:
            pass

    def remove_collision_object(self, obj_id='target_object'):
        req = ApplyPlanningScene.Request()
        sc = PlanningScene()
        sc.is_diff = True
        co = CollisionObject()
        co.id = obj_id
        co.operation = CollisionObject.REMOVE
        sc.world.collision_objects.append(co)
        req.scene = sc
        try:
            self.ps_srv.call_async(req)
        except:
            pass

    # ── Perception ────────────────────────────────────────────────────

    def call_mtc_perception(self) -> Optional[DetectedObject]:
        """Try to call the mycobot_ros2 GetPlanningScene service."""
        if self.perception_srv is None or not self.perception_srv.service_is_ready():
            return None
        from mycobot_interfaces.srv import GetPlanningScene
        req = GetPlanningScene.Request()
        req.target_shape = self.target_shape
        req.target_dimensions = self.target_dimensions
        try:
            res = self.perception_srv.call(req)
            if res.success and res.target_object_id:
                for co in res.scene_world.collision_objects:
                    if co.id == res.target_object_id:
                        p = co.primitive_poses[0].position
                        self.get_logger().info(f"MTC perception: {co.id} at ({p.x:.3f},{p.y:.3f},{p.z:.3f})")
                        return DetectedObject(x=p.x, y=p.y, z=p.z, yaw=0.0,
                                              shape=self.target_shape,
                                              dimensions=(self.object_length, self.object_width, self.object_height),
                                              in_arm_frame=True)
        except Exception as e:
            self.get_logger().warn(f"MTC perception call failed: {e}")
        return None

    def fallback_detect(self) -> Optional[DetectedObject]:
        """Depth-image-based object detection fallback."""
        if self.latest_rgb is None or self.latest_depth is None:
            return None
        try:
            cv_img = self.bridge.imgmsg_to_cv2(self.latest_rgb, 'bgr8')
            cv_dep = self.bridge.imgmsg_to_cv2(self.latest_depth, '32FC1')
        except:
            return None

        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5,5), 0)
        _, thresh = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        best = max(contours, key=cv2.contourArea)
        if cv2.contourArea(best) < 200:
            return None

        M = cv2.moments(best)
        if M['m00'] == 0:
            return None
        cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
        h, w = cv_dep.shape
        depths = []
        for dy in range(-3, 4):
            for dx in range(-3, 4):
                ny, nx = cy+dy, cx+dx
                if 0 <= ny < h and 0 <= nx < w:
                    d = cv_dep[ny, nx]
                    if not np.isnan(d) and not np.isinf(d) and d > 0.05:
                        depths.append(d)
        if not depths:
            return None
        depth = np.median(depths)

        fx = fy = 381.97
        cx_i, cy_i = 320.0, 240.0
        Xc = (cx-cx_i)*depth/fx; Yc = (cy-cy_i)*depth/fy; Zc = depth
        Xb = 0.10 + Zc; Yb = -Xc; Zb = 0.065 - Yc
        if self.odom_pose is None:
            return None
        rx, ry, ryaw = self.odom_pose.position.x, self.odom_pose.position.y, self.odom_yaw
        Xw = rx + Xb*math.cos(ryaw) - Yb*math.sin(ryaw)
        Yw = ry + Xb*math.sin(ryaw) + Yb*math.cos(ryaw)
        Zw = Zb + 0.15

        cv2.rectangle(cv_img, *cv2.boundingRect(best), (0,255,0), 2)
        cv2.circle(cv_img, (cx,cy), 5, (0,0,255), -1)
        self.annot_pub.publish(self.bridge.cv2_to_imgmsg(cv_img, 'bgr8'))

        self.get_logger().info(f"Fallback detect: world=({Xw:.3f},{Yw:.3f},{Zw:.3f})")
        return DetectedObject(x=Xw, y=Yw, z=Zw, yaw=0.0,
                              shape=self.target_shape,
                              dimensions=(self.object_length, self.object_width, self.object_height))

    def detect_object(self) -> Optional[DetectedObject]:
        obj = self.call_mtc_perception()
        if obj:
            self.has_mtc_perception = True
            return obj
        return self.fallback_detect()

    # ── Grasp planning ─────────────────────────────────────────────────

    def compute_grasp_pose(self, obj: DetectedObject) -> Tuple[float,float,float,float,float,float]:
        """Convert world-coordinate object to arm-base frame for grasp IK."""
        if self.odom_pose is None:
            return obj.x, obj.y, obj.z, self.grasp_roll, self.grasp_pitch, self.grasp_yaw
        dx = obj.x - self.odom_pose.position.x
        dy = obj.y - self.odom_pose.position.y
        ryaw = self.odom_yaw
        x_rel = dx*math.cos(ryaw) + dy*math.sin(ryaw)
        y_rel = -dx*math.sin(ryaw) + dy*math.cos(ryaw)
        y_rel = max(0.05, abs(y_rel))
        z_rel = obj.z
        return x_rel, y_rel, z_rel, self.grasp_roll, self.grasp_pitch, self.grasp_yaw

# ── Pick & Place sequences ─────────────────────────────────────────

    def execute_pick(self, obj: DetectedObject) -> bool:
        """Pick the detected object: approach → grasp → lift."""
        xr, yr, zr, roll, pitch, yaw = self.compute_grasp_pose(obj)
        self.get_logger().info(f"Pick: arm-frame ({xr:.3f},{yr:.3f},{zr:.3f})")

        self.add_collision_object(obj)

        if not self.move_arm(READY_POSE, 2.0):
            return False

        approach_j = self.solve_ik(xr, yr, zr + self.grasp_approach_offset, roll, pitch, yaw)
        if not approach_j:
            self.get_logger().error("IK fail: approach")
            return False
        if not self.move_arm(approach_j, 3.0):
            return False

        if not self.move_gripper(self.gripper_open, 1.0):
            return False

        grasp_j = self.solve_ik(xr, yr, zr, roll, pitch, yaw)
        if not grasp_j:
            self.get_logger().error("IK fail: grasp")
            return False
        if not self.move_arm(grasp_j, 3.0):
            return False

        if not self.move_gripper(self.gripper_closed, 1.5):
            return False

        lift_j = self.solve_ik(xr, yr, zr + self.grasp_lift_offset, roll, pitch, yaw)
        if not lift_j:
            lift_j = HOME_POSE
        if not self.move_arm(lift_j, 3.0):
            return False

        self.get_logger().info("Pick complete")
        return True

    def execute_place(self, tx, ty, tz, lift_before=False) -> bool:
        """Place at world coordinates (tx,ty,tz)."""
        if self.odom_pose is None:
            return False
        dx = tx - self.odom_pose.position.x
        dy = ty - self.odom_pose.position.y
        ryaw = self.odom_yaw
        xr = dx*math.cos(ryaw) + dy*math.sin(ryaw)
        yr = -dx*math.sin(ryaw) + dy*math.cos(ryaw)
        yr = max(0.05, abs(yr))

        if lift_before:
            self.move_arm(HOME_POSE, 2.0)

        lower_j = self.solve_ik(xr, yr, tz, self.grasp_roll, self.grasp_pitch, self.grasp_yaw)
        if not lower_j:
            lower_j = self.solve_ik(xr, yr, tz + 0.05, self.grasp_roll, self.grasp_pitch, self.grasp_yaw)
        if not lower_j:
            return False
        if not self.move_arm(lower_j, 3.0):
            return False

        if not self.move_gripper(self.gripper_open, 1.5):
            return False

        self.remove_collision_object()

        retreat_j = self.solve_ik(xr, yr, tz + self.grasp_approach_offset, self.grasp_roll, self.grasp_pitch, self.grasp_yaw)
        if retreat_j:
            self.move_arm(retreat_j, 2.0)

        self.move_arm(HOME_POSE, 3.0)
        self.get_logger().info("Place complete")
        return True

    def execute_mtc_pick_place(self, obj: DetectedObject) -> bool:
        """Launch the mycobot_ros2 MTC node as a subprocess."""
        import subprocess
        pkg_dir = get_package_share_directory('limo_cobot_bridge')
        params = os.path.join(pkg_dir, 'config', 'mtc_params.yaml')
        cmd = ['ros2', 'run', 'hello_mtc_with_perception', 'mtc_node',
               '--ros-args', '--params-file', params]
        self.get_logger().info(f"Launching MTC: {' '.join(cmd)}")
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            proc.wait(timeout=60)
            return proc.returncode == 0
        except Exception as e:
            self.get_logger().error(f"MTC failed: {e}")
            return False

    # ── State machine ──────────────────────────────────────────────────

    def run(self):
        self.get_logger().info("Bridge state machine started")
        self.start_time = self.get_clock().now()

        while rclpy.ok() and self.state != State.COMPLETE and self.state != State.ABORT:
            self.get_logger().info(f"State: {self.state.name}")
            if self.state == State.INIT:
                self.state = self._on_init()
            elif self.state == State.NAV_TO_PICK:
                self.state = self._on_nav_to_pick()
            elif self.state == State.DETECT:
                self.state = self._on_detect()
            elif self.state == State.PICK:
                self.state = self._on_pick()
            elif self.state == State.NAV_TO_PLACE:
                self.state = self._on_nav_to_place()
            elif self.state == State.PLACE:
                self.state = self._on_place()
            elif self.state == State.HOME:
                self.state = self._on_home()
            else:
                break
        self.get_logger().info(f"Bridge finished: {self.state.name}")

    def _on_init(self) -> State:
        self.get_logger().info("Waiting for connections...")
        for _ in range(30):
            if not rclpy.ok():
                return State.ABORT
            if self.odom_pose and self.joint_states:
                break
            time.sleep(0.5)
        if self.odom_pose is None:
            return State.ABORT
        self.get_logger().info("All connections ready")
        return State.NAV_TO_PICK

    def _on_nav_to_pick(self) -> State:
        self.get_logger().info(f"Navigating to pick zone (X={self.pick_zone_x})...")
        ok = self.drive_to_x(self.pick_zone_x, speed=self.pick_approach_speed, y_line=self.pick_zone_y, timeout=30.0)
        return State.DETECT if ok else State.ABORT

    def _on_detect(self) -> State:
        self.get_logger().info("Detecting object...")
        obj = self.detect_object()
        if obj:
            self.obj = obj
            return State.PICK
        self.get_logger().warn("No object detected, retrying...")
        return State.DETECT

    def _on_pick(self) -> State:
        if self.obj is None:
            return State.ABORT
        if self.has_mtc_perception:
            ok = self.execute_mtc_pick_place(self.obj)
        else:
            ok = self.execute_pick(self.obj)
        return State.NAV_TO_PLACE if ok else State.ABORT

    def _on_nav_to_place(self) -> State:
        self.get_logger().info(f"Navigating to place zone ({self.place_zone_x - 0.18},{self.place_zone_y})...")
        ok = self.navigate_to(self.place_zone_x - 0.18, self.place_zone_y, speed=self.nav_speed, timeout=40.0)
        return State.PLACE if ok else State.ABORT

    def _on_place(self) -> State:
        ok = self.execute_place(self.place_zone_x, self.place_zone_y, self.object_height * 0.5)
        return State.HOME if ok else State.ABORT

    def _on_home(self) -> State:
        self.get_logger().info("Returning home...")
        self.drive_to_x(self.home_zone_x, speed=self.pick_approach_speed, y_line=self.home_zone_y, timeout=30.0)
        return State.COMPLETE

# ─── Main ───────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    bridge = LimoCobotBridge()
    try:
        bridge.run()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.stop_base()
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Autonomous Pick-and-Place State Machine
========================================
Sequences the LIMO mobile base navigation with the myCobot arm manipulation
to pick a box from Table 1 and place it on Table 2.

Navigation strategy (Ackermann-compatible):
  - Robot spawns at (0, -0.22) — already aligned with pick table Y
  - Pick approach: simple drive_to_x straight forward
  - Pick→Place: reverse far back, then wide forward arc to place table Y
  - Always prefer forward arcs (Ackermann minimum turning radius ~0.41m)

Usage:
    Terminal 1:  ros2 launch limo_cobot_bringup gazebo.launch.py
    Terminal 2:  ros2 launch limo_cobot_moveit_config gazebo_moveit.launch.py
    Terminal 3:  python3 src/scripts/autonomous_pick_and_place.py
"""

import sys
import os
import time
import yaml
import rclpy
from enum import Enum, auto
from ament_index_python.packages import get_package_share_directory

import math
import argparse
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from limo_cobot_tasks.moveit_client import MoveItClient
from limo_cobot_tasks.base_controller import BaseController
from geometry_msgs.msg import Twist

# ─── ANSI colour helpers ──────────────────────────────────────────────
GREEN  = "\033[92m"
YELLOW = "\033[93m"
RED    = "\033[91m"
CYAN   = "\033[96m"
BOLD   = "\033[1m"
RESET  = "\033[0m"

def log_state(state, msg, colour=GREEN):
    print(f"{colour}{BOLD}[{state.name}]{RESET} {colour}{msg}{RESET}")


# ─── State Enum ───────────────────────────────────────────────────────
class State(Enum):
    IDLE               = auto()
    NAV_TO_PICK        = auto()
    ARM_APPROACH       = auto()
    OPEN_GRIPPER       = auto()
    ARM_DESCEND        = auto()
    CLOSE_GRIPPER      = auto()
    ATTACH             = auto()
    ARM_LIFT           = auto()
    NAV_TO_PLACE       = auto()
    ARM_DESCEND_PLACE  = auto()
    OPEN_GRIPPER_PLACE = auto()
    DETACH             = auto()
    ARM_HOME           = auto()
    NAV_HOME           = auto()
    COMPLETE           = auto()
    ABORT              = auto()


# ─── Main controller ──────────────────────────────────────────────────
class PickAndPlaceController:
    def __init__(self, config_path: str, color: str = 'red'):
        with open(config_path, 'r') as f:
            self.cfg = yaml.safe_load(f)

        self.target_color = color.lower()
        if self.target_color == 'red':
            self.model_name = "pick_box"
        elif self.target_color == 'blue':
            self.model_name = "pick_box_blue"
        elif self.target_color == 'green':
            self.model_name = "pick_box_green"
        else:
            self.model_name = "pick_box"

        self.cv_bridge = CvBridge()
        self.latest_depth_msg = None
        self.detected_box_x = 0.78  # fallback default
        self.detected_box_y = -0.22 # fallback default
        self.detected_box_z = 0.015 # fallback default (ground level)
        self.box_detected = False
        self.last_detection_time = 0.0
        self.perception_active = False

        self.rgb_sub = None
        self.depth_sub = None

        self.arm = MoveItClient()
        self.base = BaseController()
        self.annotated_pub = self.base.create_publisher(Image, '/rgb/image_annotated', 10)

        log_state(State.IDLE, f"Starting PickAndPlaceController targeting {self.target_color.upper()} cube ({self.model_name})", GREEN)

        log_state(State.IDLE, "Waiting for MoveIt connections...", CYAN)
        if not self.arm.wait_for_connections(timeout=15.0):
            raise RuntimeError("MoveIt connections failed!")
        log_state(State.IDLE, "MoveIt ready ✓", GREEN)

        log_state(State.IDLE, "Waiting for odometry...", CYAN)
        if not self.base.wait_for_odom(timeout=10.0):
            raise RuntimeError("Odometry not available!")
        log_state(State.IDLE, "Odometry ready ✓", GREEN)

        self.state = State.IDLE
        self.start_time = time.time()

        # Docked pose values for drift correction
        self.docked_x = 0.0
        self.docked_y = 0.0
        self.docked_z = 0.20
        self.docked_yaw = 0.0

    def save_docked_pose(self):
        if self.base.current_pose is not None:
            self.docked_x = self.base.current_pose.position.x
            self.docked_y = self.base.current_pose.position.y
            self.docked_z = self.base.current_pose.position.z
            self.docked_yaw = self.base.current_yaw
            log_state(self.state, f"Saved docked pose: X={self.docked_x:.4f}, Y={self.docked_y:.4f}, Z={self.docked_z:.4f}, Yaw={self.docked_yaw:.4f}", GREEN)
        else:
            log_state(self.state, "WARNING: Current pose is None, cannot save docked pose!", YELLOW)

    def _flush_odom_after_teleport(self, expected_x, expected_y, expected_yaw=0.0, label="teleport"):
        """Fix D: flush the Ackermann plugin's encoder integrator after a Gazebo teleport.
        Publishes 10 zero-velocity cycles so the plugin catches up to the new ground-truth
        transform before we start commanding motion.
        Also waits for odom X, Y, and yaw to converge to expected values."""
        import math as _math
        stop = Twist()
        for _ in range(10):
            self.base.cmd_vel_pub.publish(stop)
            time.sleep(0.05)  # 50 ms × 10 = 0.5 s total

        # Wait until /odom actually reports the teleported position AND heading
        deadline = time.time() + 3.0
        while time.time() < deadline:
            if self.base.current_pose is not None:
                dx = abs(self.base.current_pose.position.x - expected_x)
                dy = abs(self.base.current_pose.position.y - expected_y)
                yaw_err = abs(self.base._normalize_angle(self.base.current_yaw - expected_yaw))
                if dx < 0.05 and dy < 0.05 and yaw_err < 0.15:  # ~8.6°
                    break
            time.sleep(0.05)
        else:
            log_state(self.state, f"WARNING: Odom did not converge after {label}", YELLOW)

        # Log post-teleport odom for debugging
        if self.base.current_pose is not None:
            import math
            ox = self.base.current_pose.position.x
            oy = self.base.current_pose.position.y
            oyaw = math.degrees(self.base.current_yaw)
            log_state(self.state, f"Post-{label} odom: X={ox:.3f}, Y={oy:.3f}, yaw={oyaw:.1f}°", GREEN)

    def depth_callback(self, msg):
        if not self.perception_active:
            return
        self.latest_depth_msg = msg
        log_state(self.state, f"[PERCEPTION] Depth frame received, encoding={msg.encoding}, dims={msg.width}x{msg.height}", CYAN)

    def rgb_callback(self, msg):
        if not self.perception_active:
            log_state(self.state, f"[PERCEPTION] RGB callback skipped (perception not active)", YELLOW)
            return
        if self.latest_depth_msg is None:
            log_state(self.state, f"[PERCEPTION] RGB callback skipped (depth is None)", YELLOW)
            return

        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv_depth = self.cv_bridge.imgmsg_to_cv2(self.latest_depth_msg, desired_encoding="32FC1")
        except Exception as e:
            self.base.get_logger().error(f"CvBridge error: {e}")
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        if self.target_color == 'red':
            mask1 = cv2.inRange(hsv, np.array([0, 70, 50]), np.array([10, 255, 255]))
            mask2 = cv2.inRange(hsv, np.array([170, 70, 50]), np.array([180, 255, 255]))
            mask = mask1 | mask2
        elif self.target_color == 'blue':
            mask = cv2.inRange(hsv, np.array([100, 70, 50]), np.array([130, 255, 255]))
        elif self.target_color == 'green':
            mask = cv2.inRange(hsv, np.array([35, 70, 50]), np.array([85, 255, 255]))
        else:
            mask = cv2.inRange(hsv, np.array([0, 70, 50]), np.array([180, 255, 255]))

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            mask_nonzero = cv2.countNonZero(mask)
            log_state(self.state, f"[PERCEPTION] No contours in mask (nonzero pixels={mask_nonzero})", YELLOW)
            return

        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        if area < 100:
            log_state(self.state, f"[PERCEPTION] Largest contour area={area:.1f} < 100, skipping", YELLOW)
            return

        M = cv2.moments(largest_contour)
        if M['m00'] == 0:
            return
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        h, w = cv_depth.shape
        half_w = 2
        depths = []
        for dy in range(-half_w, half_w + 1):
            for dx in range(-half_w, half_w + 1):
                ny, nx = cy + dy, cx + dx
                if 0 <= ny < h and 0 <= nx < w:
                    d = cv_depth[ny, nx]
                    if not np.isnan(d) and not np.isinf(d) and d > 0.05:
                        depths.append(d)
        
        if not depths:
            return
        depth = np.median(depths)

        fx = fy = 381.97
        cx_img = 320.0
        cy_img = 240.0
        
        X_c = (cx - cx_img) * depth / fx
        Y_c = (cy - cy_img) * depth / fy
        Z_c = depth

        X_base = 0.10 + Z_c
        Y_base = -X_c
        Z_base = 0.065 - Y_c

        if self.base.current_pose is None:
            return
        rx = self.base.current_pose.position.x
        ry = self.base.current_pose.position.y
        ryaw = self.base.current_yaw

        X_world = rx + X_base * math.cos(ryaw) - Y_base * math.sin(ryaw)
        Y_world = ry + X_base * math.sin(ryaw) + Y_base * math.cos(ryaw)
        Z_world = Z_base + 0.15

        # ── Draw camera overlay ──
        x1, y1, w, h = cv2.boundingRect(largest_contour)
        cv2.rectangle(cv_image, (x1, y1), (x1 + w, y1 + h), (0, 255, 0), 2)
        cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)

        if 0.65 <= X_world <= 0.90 and -0.45 <= Y_world <= -0.05:
            self.detected_box_x = X_world
            self.detected_box_y = Y_world
            self.detected_box_z = Z_world
            self.box_detected = True
            self.last_detection_time = time.time()
            label = f"{depth*100:.0f}cm  ({X_world:.2f},{Y_world:.2f})"
            cv2.putText(cv_image, label, (cx + 8, cy - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            log_state(self.state, f"SUCCESSFULLY DETECTED {self.target_color.upper()} CUBE: 2D pixel=({cx}, {cy}), depth={depth:.3f}m", GREEN)
            log_state(self.state, f"Estimated World Coordinates: X={X_world:.3f}, Y={Y_world:.3f}, Z={Z_world:.3f}", GREEN)
        else:
            log_state(self.state, f"[PERCEPTION] Detection out of bounds: X={X_world:.3f}, Y={Y_world:.3f}", YELLOW)

        # Publish annotated image for RViz / rqt_image_view
        annotated_msg = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        annotated_msg.header = msg.header
        self.annotated_pub.publish(annotated_msg)

    def restore_docked_pose(self):
        log_state(self.state, f"Restoring docked pose to correct drift: X={self.docked_x:.3f}, Y={self.docked_y:.3f}, Z={self.docked_z:.3f}", CYAN)
        self.base.disable_braking()
        
        # Teleport limo_cobot back to the saved docked pose
        ok = self.arm.set_model_pose(
            "limo_cobot",
            self.docked_x,
            self.docked_y,
            self.docked_z,
            yaw=self.docked_yaw
        )
        if not ok:
            log_state(self.state, "WARNING: Failed to teleport limo_cobot back to docked pose!", YELLOW)
            
        # Fix D: flush odom after teleport
        self._flush_odom_after_teleport(self.docked_x, self.docked_y, expected_yaw=self.docked_yaw, label="drift-restore")
        self.base.stop()

    # ── Convenience wrappers ──────────────────────────────────────────

    def compute_grasp_target(self, box_world_x, box_world_y, box_world_z):
        """
        Transform box world coordinates into arm base frame,
        then clamp Y to keep Joint 1 away from the singularity.
        """
        import math
        base_x   = self.docked_x
        base_y   = self.docked_y
        base_yaw = self.docked_yaw   # radians

        # Homogeneous transform: world → base frame
        dx = box_world_x - base_x
        dy = box_world_y - base_y

        x_rel =  dx * math.cos(base_yaw) + dy * math.sin(base_yaw)
        y_rel = -dx * math.sin(base_yaw) + dy * math.cos(base_yaw)

        # Singularity clamp: keep Joint 1 away from 0 rad
        # Gripper opens 80 mm, box is 30 mm → 50 mm offset still lands on box
        # NOTE: +Y is IK-reachable, -Y is NOT (workspace asymmetry at this X/orientation)
        MIN_Y_OFFSET = 0.050   # metres
        if abs(y_rel) < MIN_Y_OFFSET:
            y_rel = MIN_Y_OFFSET   # always positive to stay on reachable side

        return x_rel, y_rel, box_world_z

    def _solve_and_move(self, pose_key: str, label: str, dynamic_target: bool = False) -> bool:
        p = self.cfg['arm'][pose_key]
        
        if dynamic_target:
            self.restore_docked_pose()
            
            # Determine if we are picking or placing based on state name
            if self.state in [State.ARM_APPROACH, State.ARM_DESCEND, State.ARM_LIFT]:
                # Pick box target: dynamically detected coordinates
                target_x = self.detected_box_x
                target_y = self.detected_box_y
            else:
                # Place target: world (0.78, 0.22)
                target_x = 0.78
                target_y = 0.22
                
            x_val, y_val, _ = self.compute_grasp_target(target_x, target_y, p['z'])
            
            # Let the gripper align parallel to the base (yaw_relative = 0.0)
            # This avoids wrist bending at the workspace boundary, making IK much more reliable.
            yaw_val = 0.0
            
            log_state(self.state, f"Dynamic target: relative X={x_val:.4f}, relative Y={y_val:.4f}, relative Yaw={yaw_val:.4f} (from world X={target_x:.2f}, Y={target_y:.2f})", CYAN)
        else:
            x_val = p['x']
            y_val = p['y']
            yaw_val = p['yaw']

        # For pick operations, use detected box Z + approach clearance instead of fixed config Z.
        # Config Z was calibrated for table height (box at ~0.175); boxes on ground (~0.015)
        # require a dynamic Z computed from the detected box position.
        if dynamic_target and self.state in [State.ARM_APPROACH, State.ARM_DESCEND, State.ARM_LIFT]:
            APPROACH_CLEARANCE = 0.08  # 8 cm above box center
            z_val = self.detected_box_z + APPROACH_CLEARANCE
        else:
            z_val = p['z']

        log_state(self.state, f"Solving IK for {label} at ({x_val:.3f}, {y_val:.3f}, {z_val:.3f}) with relative Yaw={yaw_val:.3f}...", CYAN)
        joints = self.arm.solve_ik(
            x=x_val, y=y_val, z=z_val,
            roll=p['roll'], pitch=p['pitch'], yaw=yaw_val,
            avoid_collisions=False
        )
        if joints is None:
            log_state(self.state, f"IK failed for {label}!", RED)
            return False
        log_state(self.state, f"IK solved → moving arm to {label}...", CYAN)
        dur = self.cfg['planning']['trajectory_duration']
        ok = self.arm.move_arm_to_joints(joints, duration=dur)
        if not ok:
            log_state(self.state, f"Trajectory execution failed for {label}!", RED)
        return ok

    def _gripper(self, action: str) -> bool:
        pos = self.cfg['gripper']['open_pos'] if action == 'open' else self.cfg['gripper']['close_pos']
        dur = self.cfg['gripper']['duration']
        log_state(self.state, f"{'Opening' if action == 'open' else 'Closing'} gripper...", CYAN)
        return self.arm.move_gripper(pos, duration=dur)

    def drive_to_x(self, target_x: float, y_lock: float = None, tolerance: float = 0.015) -> bool:
        nav = self.cfg['navigation']
        log_state(self.state, f"Driving straight to X={target_x:.3f} (y_lock={y_lock})...", CYAN)
        return self.base.drive_to_x(target_x, speed=nav['drive_speed'], y_lock=y_lock, timeout=nav['drive_timeout'], tolerance=tolerance)

    def _navigate(self, target_x: float, target_y: float, tolerance: float = 0.02) -> bool:
        nav = self.cfg['navigation']
        log_state(self.state, f"Navigating to ({target_x:.2f}, {target_y:.2f}) with tolerance={tolerance:.2f}m...", CYAN)
        return self.base.navigate_to(target_x, target_y, speed=nav['nav_speed'], tolerance=tolerance, timeout=nav['nav_timeout'])

    def _log_pose(self):
        x = self.base.current_pose.position.x
        y = self.base.current_pose.position.y
        log_state(self.state, f"Current base pose: X={x:.3f}, Y={y:.3f}", CYAN)

    # ── State handlers ────────────────────────────────────────────────

    def run_state(self) -> State:
        nav = self.cfg['navigation']

        if self.state == State.IDLE:
            log_state(self.state, "🚀 Starting autonomous pick-and-place!", GREEN)
            self._log_pose()
            return State.NAV_TO_PICK

        elif self.state == State.NAV_TO_PICK:
            nav = self.cfg['navigation']

            # ── Continuous visual servoing: drive toward the box ──
            OPTIMAL_REACH = 0.165   # arm base to box (IK-verified reachable distance)
            DOCK_TOLERANCE = 0.015  # ±1.5 cm acceptance band
            NAV_TIMEOUT = 30.0
            SPEED_MIN = 0.08        # ODE stiction floor
            SPEED_MAX = nav['drive_speed']

            log_state(self.state, "Starting continuous visual servoing toward cube...", CYAN)

            # Start perception subscriptions
            self.box_detected = False
            self.perception_active = True
            self.rgb_sub = self.base.create_subscription(
                Image, '/rgb/image_raw', self.rgb_callback, 10,
                callback_group=self.base.cb_group
            )
            self.depth_sub = self.base.create_subscription(
                Image, '/depth_camera/depth/image_raw', self.depth_callback, 10,
                callback_group=self.base.cb_group
            )

            self.base.disable_braking()
            deadline = time.time() + NAV_TIMEOUT

            while time.time() < deadline:
                rx = self.base.current_pose.position.x
                ry = self.base.current_pose.position.y
                yaw = self.base.current_yaw

                # Once box has ever been detected, always check stop condition.
                # Only stop when dist_fwd is positive (box in front of robot).
                if self.box_detected:
                    dx = self.detected_box_x - (rx - 0.03)
                    dy = self.detected_box_y - ry
                    dist_fwd = dx * math.cos(yaw) + dy * math.sin(yaw)

                    log_state(self.state, f"box=({self.detected_box_x:.3f},{self.detected_box_y:.3f})  "
                              f"dist_fwd={dist_fwd*100:.1f}cm", CYAN)

                    if 0 < dist_fwd <= OPTIMAL_REACH + DOCK_TOLERANCE:
                        self.base.stop()
                        log_state(self.state, f"Docked: cube at {dist_fwd*100:.1f}cm ✓", GREEN)
                        break

                # Use fresh camera data for steering, else crawl
                seen_recently = (time.time() - self.last_detection_time) < 1.5

                if seen_recently:
                    dx = self.detected_box_x - (rx - 0.03)
                    dy = self.detected_box_y - ry
                    y_err = -dx * math.sin(yaw) + dy * math.cos(yaw)
                    dist_fwd = dx * math.cos(yaw) + dy * math.sin(yaw)

                    speed = min(SPEED_MAX, max(SPEED_MIN, dist_fwd * 0.8))

                    desired_heading = 8.0 * y_err
                    desired_heading = max(-0.43, min(0.43, desired_heading))
                    heading_err = self.base._normalize_angle(desired_heading - yaw)
                    steer = 3.0 * heading_err
                    steer = max(-0.35, min(0.35, steer))

                    log_state(self.state, f"steer={math.degrees(steer):.0f}°  y_err={y_err*100:.1f}cm", CYAN)

                    twist = Twist()
                    twist.linear.x = speed
                    twist.angular.z = steer
                    self.base.cmd_vel_pub.publish(twist)

                else:
                    log_state(self.state, "Scanning — no cube seen recently", YELLOW)
                    twist = Twist()
                    twist.linear.x = SPEED_MIN
                    self.base.cmd_vel_pub.publish(twist)

                time.sleep(0.05)

            # ── Clean up perception subscriptions ──
            self.perception_active = False
            for sub in (self.rgb_sub, self.depth_sub):
                if sub is not None:
                    self.base.destroy_subscription(sub)
            self.rgb_sub = self.depth_sub = None

            # ── Fallback on timeout ──
            if time.time() >= deadline:
                log_state(self.state, "TIMEOUT — never reached the cube", RED)
                return State.ABORT

            self._log_pose()
            self.save_docked_pose()
            return State.ARM_APPROACH

        elif self.state == State.ARM_APPROACH:
            if not self._solve_and_move('approach', 'approach pose', dynamic_target=True):
                return State.ABORT
            log_state(self.state, "Arm at approach height ✓", GREEN)
            return State.OPEN_GRIPPER

        elif self.state == State.OPEN_GRIPPER:
            if not self._gripper('open'):
                return State.ABORT
            log_state(self.state, "Gripper opened ✓", GREEN)
            return State.ARM_DESCEND

        elif self.state == State.ARM_DESCEND:
            if not self._solve_and_move('grasp', 'grasp pose', dynamic_target=True):
                return State.ABORT
            log_state(self.state, "Arm descended to box ✓", GREEN)
            return State.CLOSE_GRIPPER

        elif self.state == State.CLOSE_GRIPPER:
            if not self._gripper('close'):
                return State.ABORT
            log_state(self.state, "Gripper closed on box ✓", GREEN)
            time.sleep(0.5)
            return State.ATTACH

        elif self.state == State.ATTACH:
            model_name = self.model_name
            log_state(self.state, f"Attaching {model_name}...", CYAN)
            if not self.arm.attach_object(model_name):
                return State.ABORT
            log_state(self.state, f"{model_name} attached ✓", GREEN)
            return State.ARM_LIFT

        elif self.state == State.ARM_LIFT:
            if not self._solve_and_move('lift', 'lift pose', dynamic_target=True):
                return State.ABORT
            log_state(self.state, "Box lifted ✓", GREEN)
            return State.NAV_TO_PLACE

        elif self.state == State.NAV_TO_PLACE:
            # Step 1: Reverse far back to give room for wide forward arc
            rev_x = nav['reverse_x']
            if not self.drive_to_x(rev_x):
                return State.ABORT
            log_state(self.state, f"Reversed to X={rev_x:.2f} ✓", GREEN)
            self._log_pose()

            # Step 2: Navigate directly to the place docking position.
            # Using navigate_to instead of drive_to_x with y_lock avoids the
            # Ackermann steering saturation that occurred when trying to
            # correct lateral error while driving forward.
            # Place target = (0.78, 0.22), same logic as pick but mirrored in Y.
            DOCK_OFFSET = 0.16
            ACKERMANN_DEADBAND = 0.026
            place_dock_x = 0.78 - DOCK_OFFSET + ACKERMANN_DEADBAND
            place_dock_x = max(0.50, min(0.656, place_dock_x))
            place_dock_y = 0.22
            log_state(self.state, f"Navigating directly to place dock ({place_dock_x:.3f}, {place_dock_y:.3f})...", CYAN)
            if not self._navigate(place_dock_x, place_dock_y, tolerance=0.03):
                return State.ABORT
            log_state(self.state, "Arrived at place table ✓", GREEN)
            self._log_pose()
            self.save_docked_pose()
            return State.ARM_DESCEND_PLACE

        elif self.state == State.ARM_DESCEND_PLACE:
            if not self._solve_and_move('grasp', 'place-descend pose', dynamic_target=True):
                return State.ABORT
            log_state(self.state, "Arm lowered to place height ✓", GREEN)
            return State.OPEN_GRIPPER_PLACE

        elif self.state == State.OPEN_GRIPPER_PLACE:
            if not self._gripper('open'):
                return State.ABORT
            log_state(self.state, "Gripper opened (release) ✓", GREEN)
            return State.DETACH

        elif self.state == State.DETACH:
            log_state(self.state, "Detaching object...", CYAN)
            self.arm.detach_object()
            log_state(self.state, "Object detached ✓", GREEN)
            return State.ARM_HOME

        elif self.state == State.ARM_HOME:
            log_state(self.state, "Moving arm to home...", CYAN)
            if not self.arm.move_arm_to_joints([0.0]*6, duration=3.0):
                return State.ABORT
            log_state(self.state, "Arm at home ✓", GREEN)
            return State.NAV_HOME

        elif self.state == State.NAV_HOME:
            # Navigate directly to home (navigate_to handles both X and Y, precision 10cm)
            if not self._navigate(nav['home_x'], nav['home_y'], tolerance=0.10):
                # Non-critical — if we can't get home, still mark complete
                log_state(self.state, "⚠ Could not fully return home, but task is done", YELLOW)
            else:
                log_state(self.state, "Robot returned home ✓", GREEN)
            self._log_pose()
            return State.COMPLETE

        elif self.state == State.COMPLETE:
            elapsed = time.time() - self.start_time
            log_state(self.state, f"✅ PICK-AND-PLACE COMPLETE in {elapsed:.1f}s wall-clock!", GREEN)
            return None

        elif self.state == State.ABORT:
            log_state(self.state, "🔴 ABORTING — tucking arm to home...", RED)
            self.arm.detach_object()
            self.arm.move_arm_to_joints([0.0]*6, duration=3.0)
            self.base.stop()
            return None

        return State.ABORT

    def reset_simulation(self):
        log_state(State.IDLE, "Teleporting robot and boxes to start poses...", CYAN)
        self.base.disable_braking()
        self.arm.detach_object()
        
        # Teleport entities
        if not self.arm.set_model_pose("limo_cobot", 0.0, -0.22, 0.20, yaw=0.0):
            log_state(State.IDLE, "WARNING: Failed to teleport limo_cobot!", YELLOW)
        if not self.arm.set_model_pose("pick_box", 0.78, -0.22, 0.015):
            log_state(State.IDLE, "WARNING: Failed to teleport pick_box!", YELLOW)
        if not self.arm.set_model_pose("pick_box_blue", 0.78, -0.14, 0.015):
            log_state(State.IDLE, "WARNING: Failed to teleport pick_box_blue!", YELLOW)
        if not self.arm.set_model_pose("pick_box_green", 0.78, -0.30, 0.015):
            log_state(State.IDLE, "WARNING: Failed to teleport pick_box_green!", YELLOW)
        
        # Fix D: flush odom after teleport — drain the Ackermann plugin's encoder integrator
        self._flush_odom_after_teleport(0.0, -0.22, label="reset")
        self.base.stop()

    def run(self):
        log_state(State.IDLE, "=" * 60, CYAN)
        log_state(State.IDLE, "  AUTONOMOUS PICK-AND-PLACE PIPELINE", CYAN)
        log_state(State.IDLE, "=" * 60, CYAN)
        
        # Teleport entities to start pose first
        self.reset_simulation()

        while self.state is not None:
            next_state = self.run_state()
            if next_state is None:
                break
            self.state = next_state

    def shutdown(self):
        self.base.stop()
        self.arm.destroy_node()
        self.base.destroy_node()


# ─── Entry point ──────────────────────────────────────────────────────
def main():
    # Parse color argument (strip ROS-specific args first)
    from rclpy.utilities import remove_ros_args
    args_without_ros = remove_ros_args(sys.argv)
    
    parser = argparse.ArgumentParser(description="Autonomous Pick-and-Place State Machine")
    parser.add_argument('--color', type=str, default='red', choices=['red', 'blue', 'green'],
                        help="Target cube color to segment and pick (red, blue, green)")
    args = parser.parse_args(args_without_ros[1:])

    rclpy.init()

    config_path = os.path.join(
        get_package_share_directory('limo_cobot_bringup'),
        'config', 'pick_place_config.yaml'
    )
    config_path = os.path.normpath(config_path)
    print(f"Loading config from: {config_path}")

    try:
        controller = PickAndPlaceController(config_path, color=args.color)
        controller.run()
    except Exception as e:
        print(f"{RED}{BOLD}FATAL: {e}{RESET}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            controller.shutdown()
        except:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()

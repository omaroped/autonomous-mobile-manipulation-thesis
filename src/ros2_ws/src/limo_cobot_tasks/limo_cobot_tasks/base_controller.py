#!/usr/bin/env python3
"""
BaseController: Python helper class to control the LIMO robot base
using /odom feedback and /cmd_vel publishing.
Supports both straight driving along X and general Go-To-Goal 2D navigation.
Uses ROS 2 simulation clock for robust timeout tracking.
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import threading
import time

def quaternion_to_euler(x, y, z, w):
    """Convert quaternion coordinates to Euler angles (roll, pitch, yaw)."""
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    
    return roll, pitch, yaw

class BaseController(Node):
    def __init__(self):
        super().__init__('base_controller_node')
        
        self.cb_group = ReentrantCallbackGroup()
        
        # Publishers & Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10, callback_group=self.cb_group)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10,
            callback_group=self.cb_group
        )
        
        self.current_pose = None
        self.current_yaw = 0.0
        
        self.cmd_vel_msg = Twist()
        self.cmd_vel_lock = threading.Lock()
        self.active_braking = False
        self.pub_timer = self.create_timer(0.05, self.publish_cmd_vel, callback_group=self.cb_group)
        
        # Start spinning in a background thread to allow blocking driving functions
        self.bg_executor = MultiThreadedExecutor()
        self.bg_executor.add_node(self)
        self.spin_thread = threading.Thread(target=self.bg_executor.spin, daemon=True)
        self.spin_thread.start()
        
        self.get_logger().info("BaseController initialized with background spin thread.")

    def publish_cmd_vel(self):
        if self.active_braking:
            with self.cmd_vel_lock:
                self.cmd_vel_pub.publish(self.cmd_vel_msg)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        q = self.current_pose.orientation
        _, _, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
        self.current_yaw = yaw

    def wait_for_odom(self, timeout=5.0):
        """Wait until the first odometry message is received."""
        start_time = time.time()
        while self.current_pose is None:
            if time.time() - start_time > timeout:
                self.get_logger().error("Timeout waiting for odometry feedback!")
                return False
            time.sleep(0.05)
        return True

    def drive_to_x(self, target_x, speed=0.12, target_y=None, y_lock=None, timeout=15.0, tolerance=0.015):
        """
        Drives the robot straight forward or backward along X to target_x.
        Keeps the robot aligned on a target Y line (either target_y or y_lock) using closed-loop line-following.
        Otherwise, keeps heading aligned parallel to the X-axis (yaw = 0.0).
        Uses simulation time for timeouts.
        """
        self.active_braking = False
        
        # Use target_y or y_lock
        y_line = target_y if y_lock is None else y_lock
        self.get_logger().info(f"drive_to_x: target_x={target_x}, speed={speed}, y_line={y_line}, tolerance={tolerance}")
        
        if not self.wait_for_odom():
            return False
            
        start_x = self.current_pose.position.x
        direction = 1.0 if target_x > start_x else -1.0
        
        rate = 0.05  # 20 Hz (50ms sleep)
        start_sim_time = self.get_clock().now()
        last_log_time = 0.0
        
        # Stuck detection: accept if no progress for 2 seconds
        STALL_SEC = 2.0
        best_dist = abs(target_x - self.current_pose.position.x)
        best_dist_time = 0.0
        
        try:
            while rclpy.ok():
                # Get elapsed simulation time
                elapsed_sim = (self.get_clock().now() - start_sim_time).nanoseconds / 1e9
                
                # Debug log every 5 sim seconds
                if elapsed_sim - last_log_time >= 5.0:
                    cx = self.current_pose.position.x
                    cy = self.current_pose.position.y
                    self.get_logger().info(
                         f"drive_to_x: pos=({cx:.3f},{cy:.3f}) "
                        f"target_x={target_x:.3f} dist={abs(target_x-cx):.3f} "
                        f"t={elapsed_sim:.1f}s"
                    )
                    last_log_time = elapsed_sim
                
                if elapsed_sim > timeout:
                    self.get_logger().error("drive_to_x: Simulation timeout reached!")
                    self.stop()
                    return False
                    
                curr_x = self.current_pose.position.x
                dist = abs(target_x - curr_x)
                
                # Symmetric tolerance check
                if dist < tolerance:
                    break
                
                # Stuck detection: if no progress for STALL_SEC, accept current position
                if dist < best_dist - 0.002:
                    best_dist = dist
                    best_dist_time = elapsed_sim
                elif elapsed_sim - best_dist_time > STALL_SEC:
                    self.get_logger().warn(
                        f"drive_to_x: Stalled at dist={dist:.3f} (best was {best_dist:.3f}). Accepting."
                    )
                    break
                    
                # Proportional steering control
                if y_line is not None:
                    curr_y = self.current_pose.position.y
                    y_err = y_line - curr_y
                    curr_yaw = self.current_yaw
                    
                    # Desired heading points towards the line proportional to Y error
                    # For backward motion, desired heading sign is inverted
                    desired_heading = direction * 8.0 * y_err
                    desired_heading = max(-0.43, min(0.43, desired_heading))
                    
                    heading_err = self._normalize_angle(desired_heading - curr_yaw)
                    Kp_yaw = 3.0
                    steer = Kp_yaw * heading_err
                    steer = max(-0.35, min(0.35, steer))
                else:
                    curr_yaw = self.current_yaw
                    heading_err = self._normalize_angle(0.0 - curr_yaw)
                    steer = 2.0 * heading_err
                    steer = max(-0.45, min(0.45, steer))
                
                # Slow down near target (speed is linear.x capped, proportional near target).
                # Floor at 0.08 to overcome ODE stiction — the Ackermann plugin won't
                # move at commanded speeds below ~0.07.
                twist_speed = min(abs(speed), max(dist * 2.0, 0.08))
                
                # Debug log steering every 10 cycles (0.5s)
                if int(elapsed_sim * 20) % 10 == 0:
                    import math
                    cy = self.current_pose.position.y
                    self.get_logger().info(f"drive_to_x line-follow: Y={cy:.3f}, dist_to_target={dist:.3f}, steer={math.degrees(steer):.1f}°")
                
                twist = Twist()
                twist.linear.x = direction * twist_speed
                twist.angular.z = steer
                self.cmd_vel_pub.publish(twist)
                
                time.sleep(rate)
                
        except Exception as e:
            self.get_logger().error(f"Error in drive_to_x: {e}")
            self.stop()
            return False
            
        self.stop()
        self.get_logger().info(f"drive_to_x: Target reached at X={self.current_pose.position.x:.3f}")
        return True

    def navigate_to(self, target_x, target_y, speed=0.15, tolerance=0.03, timeout=25.0):
        """
        Navigates the robot to a 2D pose (target_x, target_y) using Ackermann steering control.
        Determines whether to drive forward or backward based on target direction.
        Uses simulation time for timeouts.

        Includes three safeguards against the Ackermann limit-cycle problem:
          A. Distance-proportional speed reduction near the target
          B. Stuck/orbit detection — reverses to escape if no progress in 3s
          C. Heading-gate — at close range, creeps slowly when heading error is large
        """
        self.active_braking = False
        self.get_logger().info(f"navigate_to: target_x={target_x}, target_y={target_y}, speed={speed}, tolerance={tolerance}")

        if not self.wait_for_odom():
            return False

        rate = 0.05  # 20 Hz
        start_sim_time = self.get_clock().now()
        Kp_yaw = 2.0
        last_log_time = 0.0

        # --- Orbit detection state ---
        # Track the best (minimum) distance seen over a rolling window.
        # If the distance hasn't improved by at least PROGRESS_THRESHOLD
        # within STALL_WINDOW seconds, we declare a stall and reverse.
        STALL_WINDOW = 3.0          # seconds without progress → stall
        PROGRESS_THRESHOLD = 0.01   # must close at least 1 cm per window
        REVERSE_DURATION = 1.0      # seconds to reverse when escaping
        MAX_ESCAPES = 3             # give up after this many escape attempts

        best_dist = float('inf')
        best_dist_time = 0.0        # sim-time when best_dist was recorded
        escape_count = 0

        try:
            while rclpy.ok():
                elapsed_sim = (self.get_clock().now() - start_sim_time).nanoseconds / 1e9
                if elapsed_sim > timeout:
                    self.get_logger().error("navigate_to: Simulation timeout reached!")
                    self.stop()
                    return False

                curr_x = self.current_pose.position.x
                curr_y = self.current_pose.position.y
                curr_yaw = self.current_yaw

                dx = target_x - curr_x
                dy = target_y - curr_y
                dist = math.hypot(dx, dy)

                # Debug log every 5 sim seconds
                if elapsed_sim - last_log_time >= 5.0:
                    self.get_logger().info(
                        f"navigate_to: pos=({curr_x:.3f},{curr_y:.3f}) "
                        f"target=({target_x:.3f},{target_y:.3f}) "
                        f"dist={dist:.3f} yaw={math.degrees(curr_yaw):.1f}° "
                        f"t={elapsed_sim:.1f}s escapes={escape_count}"
                    )
                    last_log_time = elapsed_sim

                # If we are close enough to the target
                if dist < tolerance:
                    break

                # --- (B) Stuck / orbit detection ---
                if dist < best_dist - PROGRESS_THRESHOLD:
                    # Made meaningful progress — reset the window
                    best_dist = dist
                    best_dist_time = elapsed_sim
                elif elapsed_sim - best_dist_time > STALL_WINDOW:
                    # No progress for STALL_WINDOW seconds — likely in a limit cycle
                    escape_count += 1
                    if escape_count > MAX_ESCAPES:
                        self.get_logger().error(
                            f"navigate_to: Stuck after {MAX_ESCAPES} escape attempts "
                            f"(dist={dist:.3f}). Aborting."
                        )
                        self.stop()
                        return False

                    self.get_logger().warn(
                        f"navigate_to: Stall detected (dist={dist:.3f}, "
                        f"best={best_dist:.3f}). Reversing to escape (attempt {escape_count})..."
                    )
                    # Reverse straight back for REVERSE_DURATION seconds
                    reverse_start = (self.get_clock().now() - start_sim_time).nanoseconds / 1e9
                    while rclpy.ok():
                        rev_elapsed = (self.get_clock().now() - start_sim_time).nanoseconds / 1e9 - reverse_start
                        if rev_elapsed > REVERSE_DURATION:
                            break
                        twist = Twist()
                        twist.linear.x = -abs(speed)
                        twist.angular.z = 0.0
                        self.cmd_vel_pub.publish(twist)
                        time.sleep(rate)
                    self.stop()
                    time.sleep(0.2)

                    # Reset progress tracking after escape
                    best_dist = float('inf')
                    best_dist_time = (self.get_clock().now() - start_sim_time).nanoseconds / 1e9
                    continue

                # Compute target yaw
                target_heading = math.atan2(dy, dx)
                heading_err = self._normalize_angle(target_heading - curr_yaw)

                # Only reverse if target is clearly BEHIND the robot (>120°)
                # At 90° the controller oscillates between fwd/rev causing vibration!
                # Ackermann steering can handle up to ~90° arcs in forward gear.
                if abs(heading_err) > 2.0 * math.pi / 3.0:
                    drive_dir = -1.0
                    target_heading_back = math.atan2(-dy, -dx)
                    heading_err = self._normalize_angle(target_heading_back - curr_yaw)
                else:
                    drive_dir = 1.0

                # --- (A) Distance-proportional speed reduction ---
                # Start slowing down at 30 cm from target to prevent overshoot.
                # Minimum speed 4 cm/s so steering still has forward velocity to work with.
                SLOWDOWN_RADIUS = 0.30
                MIN_SPEED = 0.04
                speed_scale = min(1.0, dist / SLOWDOWN_RADIUS)
                twist_speed = max(MIN_SPEED, abs(speed) * speed_scale)

                # --- (C) Heading-gate for final approach ---
                # When close to the target and heading error is large, the robot
                # cannot physically correct fast enough (Ackermann min turning radius).
                # Creep very slowly so the steering has maximum effect per cm travelled.
                HEADING_GATE_DIST = 0.30      # activate within 30 cm
                HEADING_GATE_ANGLE = 0.52     # ~30 degrees
                if dist < HEADING_GATE_DIST and abs(heading_err) > HEADING_GATE_ANGLE:
                    twist_speed = MIN_SPEED    # creep to maximise steering authority

                # Control steering proportional to yaw error (capped for Ackermann limits)
                steer = drive_dir * Kp_yaw * heading_err
                steer = max(-0.45, min(0.45, steer))
                
                twist = Twist()
                twist.linear.x = drive_dir * twist_speed
                twist.angular.z = steer
                self.cmd_vel_pub.publish(twist)

                time.sleep(rate)

        except Exception as e:
            self.get_logger().error(f"Error in navigate_to: {e}")
            self.stop()
            return False

        self.stop()
        self.get_logger().info(f"navigate_to: Reached target at X={self.current_pose.position.x:.3f}, Y={self.current_pose.position.y:.3f}")
        return True

    def _normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def stop(self):
        """Stop the robot's motion and enable active braking."""
        with self.cmd_vel_lock:
            self.cmd_vel_msg = Twist()
            self.cmd_vel_pub.publish(self.cmd_vel_msg)
        self.active_braking = True

    def disable_braking(self):
        """Disable active braking."""
        self.active_braking = False

def main():
    rclpy.init()
    controller = BaseController()
    
    # Simple self-test
    print("Testing BaseController...")
    if controller.wait_for_odom():
        print(f"Current pose: {controller.current_pose.position.x}, {controller.current_pose.position.y}")
        # Drive forward 10cm
        controller.drive_to_x(controller.current_pose.position.x + 0.10, speed=0.10)
        # Drive backward 10cm
        controller.drive_to_x(controller.current_pose.position.x - 0.10, speed=0.10)
        
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

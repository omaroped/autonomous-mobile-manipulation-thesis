#!/usr/bin/env python3
import sys
import rclpy
import time

from limo_cobot_tasks.base_controller import BaseController

def main():
    rclpy.init()
    controller = BaseController()
    
    if not controller.wait_for_odom(timeout=5.0):
        print("Failed to get odometry!")
        controller.destroy_node()
        rclpy.shutdown()
        return
        
    print(f"Initial Pose: X={controller.current_pose.position.x:.3f}, Y={controller.current_pose.position.y:.3f}")
    
    # 1. Drive straight forward to pick position at X = 0.38 (assuming Y is already aligned, e.g. -0.22)
    current_y = controller.current_pose.position.y
    print(f"Step 1: Driving straight to pick position (X=0.38, Y={current_y:.3f})...")
    if not controller.drive_to_x(0.38, speed=0.15, timeout=20.0):
        print("Step 1 Failed!")
        controller.destroy_node()
        rclpy.shutdown()
        return
        
    time.sleep(1.0)
    
    # 2. Drive straight backward to transition point X = -0.10 (to clear the table before steering)
    print("Step 2: Reversing to clear table (X=-0.10)...")
    if not controller.drive_to_x(-0.10, speed=0.15, timeout=20.0):
        print("Step 2 Failed!")
        controller.destroy_node()
        rclpy.shutdown()
        return
        
    time.sleep(1.0)
    
    # 3. Navigate to placing table at (X=0.38, Y=0.22) using Ackermann steering
    print("Step 3: Navigating to placing table (X=0.38, Y=0.22) with Ackermann steering...")
    if not controller.navigate_to(0.38, 0.22, speed=0.15, timeout=30.0):
        print("Step 3 Failed!")
        controller.destroy_node()
        rclpy.shutdown()
        return
        
    time.sleep(1.0)
    
    # 4. Return to home at (X=0.0, Y=0.0)
    print("Step 4: Returning to home (X=0.0, Y=0.0)...")
    if not controller.navigate_to(0.0, 0.0, speed=0.15, timeout=30.0):
        print("Step 4 Failed!")
        
    print("Navigation test complete!")
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

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
        
    start_x = controller.current_pose.position.x
    print(f"Start X: {start_x:.4f}")
    
    # We will drive forward by 0.30 meters
    target_x = start_x + 0.30
    print(f"Driving to target X: {target_x:.4f} with speed 0.8...")
    
    # Run a manual drive loop to print coordinate updates every 0.5 seconds
    speed = 0.8
    timeout = 30.0
    start_time = time.time()
    
    from geometry_msgs.msg import Twist
    
    rate = 0.1
    try:
        while rclpy.ok():
            now = time.time()
            if now - start_time > timeout:
                print("TIMEOUT REACHED!")
                break
                
            curr_x = controller.current_pose.position.x
            print(f"Time: {now - start_time:.1f}s | Current X: {curr_x:.4f} | Target X: {target_x:.4f}")
            
            if curr_x >= target_x:
                print("TARGET X REACHED!")
                break
                
            # Publish twist
            twist = Twist()
            twist.linear.x = speed
            twist.angular.z = 0.0
            controller.cmd_vel_pub.publish(twist)
            
            time.sleep(rate)
    except KeyboardInterrupt:
        pass
        
    controller.stop()
    print(f"Final X after drive: {controller.current_pose.position.x:.4f}")
    
    # Drive back to original start_x
    print(f"Driving back to start X: {start_x:.4f}...")
    start_time = time.time()
    try:
        while rclpy.ok():
            now = time.time()
            if now - start_time > timeout:
                print("TIMEOUT REACHED!")
                break
                
            curr_x = controller.current_pose.position.x
            print(f"Time: {now - start_time:.1f}s | Current X: {curr_x:.4f} | Target X: {start_x:.4f}")
            
            if curr_x <= start_x:
                print("TARGET X REACHED!")
                break
                
            # Publish twist (backward)
            twist = Twist()
            twist.linear.x = -speed
            twist.angular.z = 0.0
            controller.cmd_vel_pub.publish(twist)
            
            time.sleep(rate)
    except KeyboardInterrupt:
        pass
        
    controller.stop()
    print(f"Final X after return: {controller.current_pose.position.x:.4f}")
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

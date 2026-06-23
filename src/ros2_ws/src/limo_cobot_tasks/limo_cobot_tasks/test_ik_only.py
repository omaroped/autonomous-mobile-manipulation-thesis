#!/usr/bin/env python3
import sys
import math
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState
from geometry_msgs.msg import PoseStamped

ARM_JOINT_NAMES = [
    'joint2_to_joint1',
    'joint3_to_joint2',
    'joint4_to_joint3',
    'joint5_to_joint4',
    'joint6_to_joint5',
    'joint6output_to_joint6'
]

class StandaloneIKTester(Node):
    def __init__(self):
        super().__init__('standalone_ik_tester')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
    def solve_ik(self, x, y, z, roll, pitch, yaw, avoid_collisions=False):
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            print("IK service not available!")
            return None
            
        req = GetPositionIK.Request()
        req.ik_request.group_name = "arm"
        req.ik_request.avoid_collisions = avoid_collisions
        req.ik_request.ik_link_name = "gripper_tcp"
        
        # Target Pose
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = z
        
        # Euler to Quaternion conversion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        pose_stamped.pose.orientation.w = cr * cp * cy + sr * sp * sy
        pose_stamped.pose.orientation.x = sr * cp * cy - cr * sp * sy
        pose_stamped.pose.orientation.y = cr * sp * cy + sr * cp * sy
        pose_stamped.pose.orientation.z = cr * cp * sy - sr * sp * cy
        
        req.ik_request.pose_stamped = pose_stamped
        
        # Seed State
        seed_state = RobotState()
        seed_state.joint_state.name = ARM_JOINT_NAMES
        seed_state.joint_state.position = [0.0, -0.5, -1.2, 1.7, 0.0, 0.0]
        req.ik_request.robot_state = seed_state
        
        print(f"Calling IK for x={x}, y={y}, z={z}, r={roll}, p={pitch}, y={yaw}...")
        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res is None:
            print("Failed to call service!")
            return None
        return res

def main():
    rclpy.init()
    tester = StandaloneIKTester()
    
    print("Testing pick and place poses (pitch=1.5708, roll=0, yaw=0)...")
    for z in [0.025, 0.065, 0.125]:
        print(f"--- Z = {z} ---")
        for x in [0.12, 0.13, 0.14, 0.15, 0.16]:
            res = tester.solve_ik(x, 0.0, z, 0.0, 1.5708, 0.0, avoid_collisions=False)
            if res:
                if res.error_code.val == 1:
                    sol = [round(pos, 4) for name, pos in zip(res.solution.joint_state.name, res.solution.joint_state.position) if name in ARM_JOINT_NAMES]
                    print(f"  x={x:.2f} -> SUCCESS. Joints: {sol}")
                else:
                    print(f"  x={x:.2f} -> FAILED ({res.error_code.val})")
            else:
                print(f"  x={x:.2f} -> Failed to call service.")
            
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

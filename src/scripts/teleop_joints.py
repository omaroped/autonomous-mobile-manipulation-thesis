#!/usr/bin/env python3
"""
myCobot 280 Joint Teleop for ROS 2 Gazebo/Simulation.
Allows selecting a joint (1-6) using number keys, and moving it using keyboard inputs.

Usage:
  python3 teleop_joints.py
"""

import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Joint names from arm_controllers.yaml
ARM_JOINTS = [
    'joint2_to_joint1',
    'joint3_to_joint2',
    'joint4_to_joint3',
    'joint5_to_joint4',
    'joint6_to_joint5',
    'joint6output_to_joint6'
]
GRIPPER_JOINTS = ['gripper_controller']

class TeleopJoints(Node):
    def __init__(self):
        super().__init__('teleop_joints')
        self.joint_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, '/mycobot_gripper_controller/joint_trajectory', 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        self.current_positions = {name: 0.0 for name in ARM_JOINTS}
        self.gripper_position = 0.0
        self.received_first_state = False
        self.selected_joint_idx = 0  # Default to joint 1 (idx 0)
        self.step_size = 0.05  # radians (~3 degrees)
        
    def joint_state_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in self.current_positions:
                self.current_positions[name] = pos
            elif name in GRIPPER_JOINTS:
                self.gripper_position = pos
        self.received_first_state = True

    def publish_arm_target(self, targets):
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINTS
        point = JointTrajectoryPoint()
        point.positions = [targets[name] for name in ARM_JOINTS]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 200000000  # 0.2s duration
        msg.points.append(point)
        self.joint_pub.publish(msg)

    def publish_gripper_target(self, target_val):
        msg = JointTrajectory()
        msg.joint_names = GRIPPER_JOINTS
        point = JointTrajectoryPoint()
        point.positions = [target_val]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 200000000  # 0.2s duration
        msg.points.append(point)
        self.gripper_pub.publish(msg)

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':
            key += sys.stdin.read(2)
    else:
        key = ''
    termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    node = TeleopJoints()
    
    settings = termios.tcgetattr(sys.stdin)
    
    # Wait for first joint state update to populate starting positions
    print("Waiting for /joint_states to initialize starting angles...")
    while rclpy.ok() and not node.received_first_state:
        rclpy.spin_once(node, timeout_sec=0.1)
        
    targets = {name: node.current_positions[name] for name in ARM_JOINTS}
    gripper_target = node.gripper_position
    
    def print_menu():
        # Clear terminal screen
        sys.stdout.write("\033[H\033[J")
        print("==================================================")
        print("      myCobot 280 Joint Teleop (ROS 2 Sim)        ")
        print("==================================================")
        print(" [1-6] Select Joint | [+ / -] Move Joint | [0] Reset to 0")
        print(" [O] Open Gripper   | [C] Close Gripper  | [Q] Quit")
        print("--------------------------------------------------")
        print("Current Joint Positions (Targets):")
        for i, name in enumerate(ARM_JOINTS):
            prefix = "* " if i == node.selected_joint_idx else "  "
            suffix = "  <-- SELECTED" if i == node.selected_joint_idx else ""
            print(f"{prefix}Joint {i+1} ({name}): {targets[name]:.3f} rad ({targets[name]*57.296:.1f}°){suffix}")
        print(f"  Gripper (gripper_controller): {gripper_target:.3f} rad")
        print("--------------------------------------------------")
        print("Press keys to control...")

    print_menu()
    
    try:
        while rclpy.ok():
            # Process ROS callbacks
            rclpy.spin_once(node, timeout_sec=0.01)
            
            # Read keypress
            key = get_key(settings)
            if not key:
                continue
                
            dirty = False
            dirty_gripper = False
            
            if key == 'q' or key == 'Q':
                break
            elif key in ['1', '2', '3', '4', '5', '6']:
                node.selected_joint_idx = int(key) - 1
                dirty = True
            elif key in ['+', '=', ']', 'd', 'D', '\x1b[C', '\x1b[A']:
                active_joint = ARM_JOINTS[node.selected_joint_idx]
                targets[active_joint] += node.step_size
                dirty = True
            elif key in ['-', '_', '[', 'a', 'A', '\x1b[D', '\x1b[B']:
                active_joint = ARM_JOINTS[node.selected_joint_idx]
                targets[active_joint] -= node.step_size
                dirty = True
            elif key == '0':
                active_joint = ARM_JOINTS[node.selected_joint_idx]
                targets[active_joint] = 0.0
                dirty = True
            elif key in ['o', 'O']:
                gripper_target = 0.0  # Open gripper
                dirty_gripper = True
            elif key in ['c', 'C']:
                gripper_target = -0.7  # Close gripper
                dirty_gripper = True
                
            if dirty:
                node.publish_arm_target(targets)
                print_menu()
            elif dirty_gripper:
                node.publish_gripper_target(gripper_target)
                print_menu()
                
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration
import curses
import sys
import time

class LimoMyCobotTeleop(Node):
    def __init__(self):
        super().__init__('limo_mycobot_teleop')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gripper_pub = self.create_publisher(Float64MultiArray, '/mycobot_gripper_controller/commands', 10)
        
        # Action Client for Arm
        self.arm_action_client = ActionClient(self, FollowJointTrajectory, '/mycobot_arm_controller/follow_joint_trajectory')
        
        # Subscriber for real-time feedback
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        # 10Hz Timer for steady-state control (prevents Gazebo drift)
        self.control_timer = self.create_timer(0.1, self.timer_callback)
        
        self.joint_names = [
            'joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
            'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6'
        ]
        
        self.current_positions = [0.0] * 6
        self.current_gripper = 0.0
        self.gripper_cmd = 0.0  # last ramped gripper command, for smooth open/close
        self.selected_joint = 0
        
        # Velocity State
        self.lin_vel = 0.0
        self.ang_vel = 0.0
        
        # Presets & Increments
        self.speed_steps = [0.0, 0.2, 0.4, 0.6]
        self.current_speed_idx = 0
        self.joint_inc = 0.05
        
        self.initialized = False
        self.status = "Searching for simulation..."

    def joint_state_callback(self, msg):
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_positions[i] = msg.position[idx]
        
        if 'gripper_controller' in msg.name:
            idx = msg.name.index('gripper_controller')
            self.current_gripper = msg.position[idx]
            
        self.initialized = True

    def timer_callback(self):
        # Continuous heartbeat prevents physics drifting in Gazebo
        msg = Twist()
        msg.linear.x = self.lin_vel
        msg.angular.z = self.ang_vel
        self.cmd_vel_pub.publish(msg)

    def send_arm_goal(self, increment):
        if not self.initialized: return
        target_positions = list(self.current_positions)
        target_positions[self.selected_joint] += increment
        
        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.time_from_start = Duration(sec=0, nanosec=500000000)
        trajectory.points = [point]
        goal_msg.trajectory = trajectory
        
        self.status = f"Moving Arm Joint {self.selected_joint+1}..."
        self._send_goal_future = self.arm_action_client.send_goal_async(goal_msg)

    def publish_gripper(self, target_cmd):
        # Ramp from the current command to the target so the gripper moves smoothly
        # instead of snapping (a one-shot jump imparts a large impulse in Gazebo).
        m = [1.0, 1.0, -1.0, -1.0, -1.0, 1.0]
        start = self.gripper_cmd
        steps = 15
        for i in range(1, steps + 1):
            cmd = start + (i / steps) * (target_cmd - start)
            msg = Float64MultiArray()
            msg.data = [float(cmd * x) for x in m]
            self.gripper_pub.publish(msg)
            time.sleep(0.04)
        self.gripper_cmd = target_cmd

    def update_base(self, lin_dir=None, ang_dir=None, stop=False):
        if stop:
            self.lin_vel = 0.0
            self.ang_vel = 0.0
        if lin_dir is not None:
            self.lin_vel += lin_dir * 0.1
        if ang_dir is not None:
            self.ang_vel += ang_dir * 0.2
            
        # Limits
        self.lin_vel = max(min(self.lin_vel, 1.0), -1.0)
        self.ang_vel = max(min(self.ang_vel, 1.0), -1.0)

def main(stdscr):
    node = LimoMyCobotTeleop()
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(50)
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            stdscr.clear()
            height, width = stdscr.getmaxyx()

            if not node.initialized:
                stdscr.addstr(0, 0, "WAITING FOR SIMULATION HEARTBEAT...", curses.A_REVERSE)
                stdscr.refresh()
                continue

            if height < 22 or width < 65:
                stdscr.addstr(0, 0, "TERMINAL TOO SMALL!", curses.A_REVERSE)
            else:
                try:
                    stdscr.addstr(0, 0, "=== LIMO + MyCobot Unified Teleop (Precision Base) ===", curses.A_BOLD)
                    
                    stdscr.addstr(2, 0, "--- LIMO MOBILE BASE ---")
                    stdscr.addstr(3, 2, f"Target: {node.lin_vel:.2f} m/s | Turn: {node.ang_vel:.2f} rad/s")
                    stdscr.addstr(4, 2, "[W/S] Lin +/- | [A/D] Turn +/- | [SPACE] Emergency Brake", curses.A_DIM)
                    
                    stdscr.addstr(6, 0, "--- MYCOBOT ARM JOINTS ---")
                    for i in range(6):
                        style = curses.A_REVERSE if i == node.selected_joint else curses.A_NORMAL
                        stdscr.addstr(7 + i, 2, f"Joint {i+1}: {node.current_positions[i]:.2f}", style)
                    stdscr.addstr(13, 2, "[1-6] Select | [Q/E] +/- Arm Angle", curses.A_DIM)
                    
                    stdscr.addstr(15, 0, f"--- GRIPPER: [O] Open | [P] Close | Current: {node.current_gripper:.2f}")
                    
                    stdscr.addstr(17, 0, "SYSTEM STATUS:", curses.A_BOLD)
                    stdscr.addstr(18, 2, f">>> {node.status}", curses.A_REVERSE)
                    stdscr.addstr(20, 0, "Hold [ESC] to Exit Controller", curses.A_DIM)
                except curses.error: pass

            char = stdscr.getch()
            if char == 27: break
            elif char == ord(' '):
                node.update_base(stop=True)
                node.status = "BASE LOCKED AT 0.0"
            elif char in [ord('w'), ord('W')]: node.update_base(lin_dir=1)
            elif char in [ord('s'), ord('S')]: node.update_base(lin_dir=-1)
            elif char in [ord('a'), ord('A')]: node.update_base(ang_dir=1)
            elif char in [ord('d'), ord('D')]: node.update_base(ang_dir=-1)
            elif ord('1') <= char <= ord('6'):
                node.selected_joint = char - ord('1')
                node.status = f"Arm Joint {node.selected_joint + 1} Selected"
            elif char in [ord('q'), ord('Q')]: node.send_arm_goal(node.joint_inc)
            elif char in [ord('e'), ord('E')]: node.send_arm_goal(-node.joint_inc)
            elif char in [ord('o'), ord('O')]:
                node.publish_gripper(0.15)
                node.status = "Commanding Gripper OPEN"
            elif char in [ord('p'), ord('P')]:
                node.publish_gripper(-0.5)
                node.status = "Commanding Gripper CLOSE"
            
            stdscr.refresh()

    finally:
        node.update_base(stop=True)
        node.destroy_node()

if __name__ == '__main__':
    rclpy.init()
    try:
        curses.wrapper(main)
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok(): rclpy.shutdown()

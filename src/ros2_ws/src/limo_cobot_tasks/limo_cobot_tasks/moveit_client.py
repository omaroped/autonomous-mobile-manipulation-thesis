#!/usr/bin/env python3
"""
MoveItClient: Python helper class to communicate with MoveIt 2 services 
and ros2_control action servers using a background spin thread to prevent deadlocks.
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState
from geometry_msgs.msg import PoseStamped, Quaternion, Pose, Twist
from gazebo_msgs.srv import GetEntityState, SetEntityState
from gazebo_msgs.msg import EntityState
import math
import threading
import time
import random

ARM_JOINT_NAMES = [
    'joint2_to_joint1',
    'joint3_to_joint2',
    'joint4_to_joint3',
    'joint5_to_joint4',
    'joint6_to_joint5',
    'joint6output_to_joint6'
]

# Joint limits for myCobot 280 arm joints from URDF (lower, upper)
JOINT_LIMITS = [
    (-2.9321, 2.9321),    # joint2_to_joint1
    (-2.4434, 2.4434),    # joint3_to_joint2
    (-2.6179, 2.6179),    # joint4_to_joint3
    (-2.6179, 2.6179),    # joint5_to_joint4
    (-2.7052, 2.7925),    # joint6_to_joint5
    (-3.14159, 3.14159)   # joint6output_to_joint6
]

GRIPPER_JOINT_NAMES = ['gripper_controller']

class MoveItClient(Node):
    def __init__(self):
        super().__init__('moveit_client_node')
        
        self.cb_group = ReentrantCallbackGroup()
        
        # Action Clients
        self.arm_action_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory', 
            callback_group=self.cb_group
        )
        self.gripper_action_client = ActionClient(
            self, FollowJointTrajectory, '/mycobot_gripper_controller/follow_joint_trajectory',
            callback_group=self.cb_group
        )
        
        # Service Clients
        self.ik_client = self.create_client(
            GetPositionIK, '/compute_ik', callback_group=self.cb_group
        )
        self.get_entity_state_client = self.create_client(
            GetEntityState, '/gazebo/get_entity_state', callback_group=self.cb_group
        )
        self.set_entity_state_client = self.create_client(
            SetEntityState, '/gazebo/set_entity_state', callback_group=self.cb_group
        )
        
        # Gazebo Attachment State
        self.attach_thread = None
        self.attach_running = False
        self.attached_model = None
        self.attached_relative_pose = None
        self.attached_reference_frame = None
        
        # Subscriber for current joint states (needed for IK seed state)
        self.joint_states = None
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10,
            callback_group=self.cb_group
        )
        
        # Start spinning in a background thread to prevent deadlocks with blocking calls
        self.bg_executor = MultiThreadedExecutor()
        self.bg_executor.add_node(self)
        self.spin_thread = threading.Thread(target=self.bg_executor.spin, daemon=True)
        self.spin_thread.start()
        
        self.get_logger().info("Background spin thread started. Waiting for connections...")
        
    def wait_for_connections(self, timeout=10.0):
        start_time = time.time()
        
        # Wait for IK service
        while not self.ik_client.service_is_ready():
            if time.time() - start_time > timeout:
                self.get_logger().error("IK service /compute_ik not available!")
                return False
            time.sleep(0.1)
            
        # Wait for Gazebo state services
        while not self.get_entity_state_client.service_is_ready():
            if time.time() - start_time > timeout:
                self.get_logger().error("Gazebo GetEntityState service not available!")
                return False
            time.sleep(0.1)
            
        while not self.set_entity_state_client.service_is_ready():
            if time.time() - start_time > timeout:
                self.get_logger().error("Gazebo SetEntityState service not available!")
                return False
            time.sleep(0.1)
            
        # Wait for arm action
        while not self.arm_action_client.server_is_ready():
            if time.time() - start_time > timeout:
                self.get_logger().error("Arm action server not available!")
                return False
            time.sleep(0.1)
            
        # Wait for gripper action
        while not self.gripper_action_client.server_is_ready():
            if time.time() - start_time > timeout:
                self.get_logger().error("Gripper action server not available!")
                return False
            time.sleep(0.1)
            
        # Wait for joint states
        while self.joint_states is None:
            if time.time() - start_time > timeout:
                self.get_logger().error("Timeout waiting for /joint_states!")
                return False
            time.sleep(0.1)
            
        self.get_logger().info("All connections established successfully!")
        return True

    def joint_state_callback(self, msg):
        self.joint_states = msg

    def get_current_joint_positions(self, joint_names):
        if self.joint_states is None:
            return None
        positions = []
        for name in joint_names:
            if name in self.joint_states.name:
                idx = self.joint_states.name.index(name)
                positions.append(self.joint_states.position[idx])
            else:
                positions.append(0.0)
        return positions
    def solve_ik(self, x, y, z, roll, pitch, yaw, frame_id="base_link", seed_joints=None, avoid_collisions=True, timeout=5.0):
        """Calls the /compute_ik service to solve Inverse Kinematics for a Cartesian pose using a multi-seed strategy."""
        self.get_logger().info(f"solve_ik: avoid_collisions={avoid_collisions}")
        req = GetPositionIK.Request()
        req.ik_request.group_name = "arm"
        req.ik_request.avoid_collisions = avoid_collisions
        req.ik_request.ik_link_name = "gripper_tcp"
        
        # Target Pose
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
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
        
        # Collect candidate seeds to try sequentially
        seeds = []
        if seed_joints is not None:
            seeds.append(list(seed_joints))
            
        # 1. Current joint positions
        current_positions = []
        for name in ARM_JOINT_NAMES:
            if name in self.joint_states.name:
                idx = self.joint_states.name.index(name)
                current_positions.append(self.joint_states.position[idx])
            else:
                current_positions.append(0.0)
        seeds.append(current_positions)
        
        # 2. Ready state posture
        ready_posture = [0.0, -0.5, -1.2, 1.7, 0.0, 0.0]
        seeds.append(ready_posture)
        
        # 3. Useful search postures (forward reach, high ready)
        seeds.append([0.0, -1.0, -0.8, 1.8, 0.0, 0.0])
        seeds.append([0.0, -0.2, -1.5, 1.7, 0.0, 0.0])
        
        # 3b. Known working joint postures from workspace reachability mapping
        known_seeds = [
            [-0.004, -2.019, 1.118, -1.132, 1.562, 2.679],  # x=0.21, z=0.14
            [-0.004, -1.034, -1.03, 0.04, 1.562, 2.689],    # x=0.21, z=0.14 (elbow-down)
            [-0.004, -1.741, 0.986, -1.272, 1.562, 2.686],  # x=0.21, z=0.18
            [-0.004, -1.366, 0.599, -1.264, 1.562, 2.681],  # x=0.21, z=0.22
            [-0.004, -1.082, -0.931, -0.005, 1.562, 2.694], # x=0.22, z=0.14
            [-0.004, -1.666, 0.782, -1.129, 1.562, 2.699],  # x=0.22, z=0.18
            [-0.004, -1.487, 0.606, -1.131, 1.562, 2.701],  # x=0.22, z=0.20
            [-0.004, -1.847, 0.701, -0.853, 1.562, 2.713],  # x=0.23, z=0.14
            [-0.004, -1.091, -0.491, -0.425, 1.562, 2.706], # x=0.23, z=0.18
            [-0.004, -1.321, 0.2, -0.87, 1.562, 2.721]      # x=0.23, z=0.20
        ]
        seeds.extend(known_seeds)
        
        # 4. Generate 8 random seeds within joint limits
        for _ in range(8):
            rand_seed = [random.uniform(limit[0], limit[1]) for limit in JOINT_LIMITS]
            seeds.append(rand_seed)
            
        # Try each seed until a valid solution is found
        for seed_idx, seed in enumerate(seeds):
            seed_state = RobotState()
            seed_state.joint_state.name = ARM_JOINT_NAMES
            seed_state.joint_state.position = seed
            req.ik_request.robot_state = seed_state
            
            try:
                res = self.ik_client.call(req)
            except Exception as e:
                self.get_logger().error(f"IK service call failed on seed {seed_idx}: {e}")
                continue
            
            if res is None:
                continue
                
            if res.error_code.val == 1:  # 1 = SUCCESS
                # Extract arm joint values
                joint_positions = []
                for name in ARM_JOINT_NAMES:
                    if name in res.solution.joint_state.name:
                        idx = res.solution.joint_state.name.index(name)
                        joint_positions.append(res.solution.joint_state.position[idx])
                    else:
                        self.get_logger().error(f"Missing joint {name} in IK solution!")
                        joint_positions = None
                        break
                
                if joint_positions is not None:
                    # Success! Return the first working joint solution
                    return joint_positions
            else:
                self.get_logger().info(f"Seed {seed_idx} failed with error code: {res.error_code.val}")
                    
        # If all seeds failed
        self.get_logger().warn(f"IK solver failed for x={x:.3f}, y={y:.3f}, z={z:.3f} after trying {len(seeds)} seeds.")
        return None

    def execute_trajectory(self, joint_names, target_positions, duration=2.5, is_gripper=False, timeout=10.0):
        """Sends a joint trajectory action goal and waits for execution using thread-safe polling."""
        client = self.gripper_action_client if is_gripper else self.arm_action_client
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = joint_names
        
        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        goal_msg.trajectory.points.append(point)
        
        self.get_logger().info(f"Sending trajectory goal to {'gripper' if is_gripper else 'arm'}...")
        send_goal_future = client.send_goal_async(goal_msg)
        
        # Wait for goal acceptance
        start_time = time.time()
        while not send_goal_future.done():
            if time.time() - start_time > timeout:
                self.get_logger().error("Send goal timed out!")
                return False
            time.sleep(0.05)
            
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory goal rejected by server!")
            return False
            
        self.get_logger().info("Goal accepted, waiting for result...")
        get_result_future = goal_handle.get_result_async()
        
        # Wait for execution result
        start_time = time.time()
        while not get_result_future.done():
            if time.time() - start_time > timeout + duration:
                self.get_logger().error("Result future timed out!")
                return False
            time.sleep(0.05)
            
        result = get_result_future.result()
        if result.status == 4: # 4 = STATUS_SUCCEEDED
            self.get_logger().info("Trajectory execution succeeded!")
            return True
        else:
            self.get_logger().error(f"Trajectory failed with status code: {result.status}")
            return False

    def move_arm_to_joints(self, target_joints, duration=2.5):
        return self.execute_trajectory(ARM_JOINT_NAMES, target_joints, duration, is_gripper=False)

    def move_gripper(self, target_pos, duration=1.5):
        return self.execute_trajectory(GRIPPER_JOINT_NAMES, [target_pos], duration, is_gripper=True)

    def attach_object(self, model_name, reference_frame="limo_cobot::joint6_flange", timeout=2.0):
        """Programmatically attaches a Gazebo model to a reference frame."""
        self.get_logger().info(f"Attaching {model_name} to {reference_frame}...")
        
        req = GetEntityState.Request()
        req.name = model_name
        req.reference_frame = reference_frame
        
        try:
            res = self.get_entity_state_client.call(req)
        except Exception as e:
            self.get_logger().error(f"GetEntityState service call failed: {e}")
            return False
            
        if res is None or not res.success:
            self.get_logger().error(f"Failed to get entity state for {model_name} relative to {reference_frame}")
            return False
            
        self.attached_model = model_name
        self.attached_reference_frame = reference_frame
        self.attached_relative_pose = res.state.pose
        self.attach_running = True
        
        # Start background thread to keep updating pose in Gazebo
        self.attach_thread = threading.Thread(target=self._publish_attached_pose, daemon=True)
        self.attach_thread.start()
        
        self.get_logger().info(f"Successfully attached {model_name}!")
        return True

    def _publish_attached_pose(self):
        """Background loop to continuously update the Gazebo entity state.
        
        Uses world-frame SetEntityState to freeze the box in world space,
        counteracting gravity between updates."""
        rate = 0.01  # 100 Hz
        
        box_req = SetEntityState.Request()
        box_req.state.name = self.attached_model
        box_req.state.twist = Twist()
        
        while self.attach_running and rclpy.ok():
            # Use joint6_flange reference frame — the box stays fixed relative
            # to the flange. At 100 Hz, gravity drift is only ~0.5mm between
            # updates, which is invisible.
            box_req.state.reference_frame = self.attached_reference_frame
            box_req.state.pose = self.attached_relative_pose
            self.set_entity_state_client.call_async(box_req)
            time.sleep(rate)

    def detach_object(self, timeout=2.0):
        """Detaches the currently attached object."""
        if not self.attach_running:
            self.get_logger().warn("No object currently attached!")
            return False
            
        self.get_logger().info(f"Detaching {self.attached_model}...")
        self.attach_running = False
        if self.attach_thread is not None:
            self.attach_thread.join(timeout=timeout)
            
        # Set final pose in world coordinates to reset dynamics
        req = SetEntityState.Request()
        req.state.name = self.attached_model
        req.state.reference_frame = "world"
        
        get_req = GetEntityState.Request()
        get_req.name = self.attached_model
        get_req.reference_frame = "world"
        
        try:
            get_res = self.get_entity_state_client.call(get_req)
        except Exception as e:
            self.get_logger().error(f"GetEntityState service call failed: {e}")
            get_res = None
            
        if get_res is not None and get_res.success:
            req.state.pose = get_res.state.pose
            req.state.twist = Twist()
            self.set_entity_state_client.call_async(req)
            
        self.attached_model = None
        self.attached_relative_pose = None
        self.attached_reference_frame = None
        self.get_logger().info("Object detached successfully!")
        return True

    def set_model_pose(self, model_name, x, y, z, roll=0.0, pitch=0.0, yaw=0.0, reference_frame="world", timeout=2.0):
        """Sets the pose of a Gazebo model directly."""
        req = SetEntityState.Request()
        req.state.name = model_name
        req.state.reference_frame = reference_frame
        req.state.pose.position.x = x
        req.state.pose.position.y = y
        req.state.pose.position.z = z
        
        # Euler to Quaternion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        req.state.pose.orientation.w = cr * cp * cy + sr * sp * sy
        req.state.pose.orientation.x = sr * cp * cy - cr * sp * sy
        req.state.pose.orientation.y = cr * sp * cy + sr * cp * sy
        req.state.pose.orientation.z = cr * cp * sy - sr * sp * cy
        req.state.twist = Twist()
        
        try:
            res = self.set_entity_state_client.call(req)
            return res is not None and res.success
        except Exception as e:
            self.get_logger().error(f"SetEntityState service call failed: {e}")
            return False

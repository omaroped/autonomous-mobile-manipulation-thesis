#!/usr/bin/python3
import csv
import os
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
from geometry_msgs.msg import PoseStamped

class ReachSweep(Node):
    def __init__(self):
        super().__init__('reach_sweep')
        self._client = self.create_client(GetPositionIK, '/compute_ik')
        
    def wait_for_ik_service(self, timeout=60.0):
        self.get_logger().info("Waiting for /compute_ik service...")
        if not self._client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("/compute_ik service not available!")
            return False
        self.get_logger().info("/compute_ik service is available.")
        return True

    def query_ik(self, x, y, z, qx, qy, qz, qw):
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'arm'
        req.ik_request.ik_link_name = 'gripper_tcp'
        req.ik_request.pose_stamped.header.frame_id = 'base_link'
        req.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        req.ik_request.pose_stamped.pose.position.x = float(x)
        req.ik_request.pose_stamped.pose.position.y = float(y)
        req.ik_request.pose_stamped.pose.position.z = float(z)
        req.ik_request.pose_stamped.pose.orientation.x = float(qx)
        req.ik_request.pose_stamped.pose.orientation.y = float(qy)
        req.ik_request.pose_stamped.pose.orientation.z = float(qz)
        req.ik_request.pose_stamped.pose.orientation.w = float(qw)
        req.ik_request.avoid_collisions = True
        
        # Call service synchronously
        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            res = future.result()
            # error_code.val == 1 is SUCCESS
            return res.error_code.val == 1
        return False

    def run_sweep(self):
        if not self.wait_for_ik_service():
            return
        
        # Grid settings
        xs = [round(0.12 + i * 0.02, 2) for i in range(9)] # 0.12 to 0.28 step 0.02
        ys = [round(-0.08 + i * 0.04, 2) for i in range(5)] # -0.08 to 0.08 step 0.04
        zs = [round(0.03 + i * 0.02, 2) for i in range(9)] # 0.03 to 0.19 step 0.02
        
        # Orientations
        orientations = [
            ('top-down', (-0.7071, 0.0, 0.0, 0.7071)),
            ('frontal', (0.707, 0.0, -0.707, 0.0)),
            ('tilt-75', (0.793, 0.0, -0.609, 0.0)),
            ('tilt-55', (0.891, 0.0, -0.454, 0.0)),
            ('side-grasp', (0.5, -0.5, 0.5, -0.5))
        ]
        output_path = '/home/omar/Desktop/Thesisorg/docs/references/data/reach_map.csv'
        self.get_logger().info(f"Starting reach sweep. Saving results to {output_path}...")
        
        results = []
        counts = {name: 0 for name, _ in orientations}
        total_queries = 0
        
        for name, q in orientations:
            self.get_logger().info(f"Sweeping orientation: {name}...")
            for x in xs:
                for y in ys:
                    for z in zs:
                        success = self.query_ik(x, y, z, q[0], q[1], q[2], q[3])
                        total_queries += 1
                        if success:
                            counts[name] += 1
                        results.append({
                            'x': x, 'y': y, 'z': z,
                            'orientation': name,
                            'qx': q[0], 'qy': q[1], 'qz': q[2], 'qw': q[3],
                            'success': 1 if success else 0
                        })
        
        # Write to CSV
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        with open(output_path, mode='w', newline='') as f:
            fieldnames = ['x', 'y', 'z', 'orientation', 'qx', 'qy', 'qz', 'qw', 'success']
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for r in results:
                writer.writerow(r)
                
        self.get_logger().info("=== Sweep Complete ===")
        for name, count in counts.items():
            self.get_logger().info(f"Orientation {name}: {count} reachable points")
        self.get_logger().info(f"Total queries: {total_queries}. Results written to {output_path}")

def main(args=None):
    rclpy.init(args=args)
    node = ReachSweep()
    try:
        node.run_sweep()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

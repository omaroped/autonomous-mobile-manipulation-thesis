#!/usr/bin/env python3
import os
import csv
import math
import rclpy
from limo_cobot_tasks.moveit_client import MoveItClient

def main():
    rclpy.init()
    client = MoveItClient()
    
    print("Waiting for MoveIt connections...")
    if not client.wait_for_connections(timeout=10.0):
        print("MoveIt connection timeout!")
        client.destroy_node()
        rclpy.shutdown()
        return
        
    print("MoveIt connected. Starting workspace grid search...")
    
    # Output file path
    output_dir = "/home/omar/Desktop/Thesis/data/processed"
    os.makedirs(output_dir, exist_ok=True)
    output_file = os.path.join(output_dir, "ik_workspace_map.csv")
    
    # Open CSV file
    with open(output_file, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'x', 'y', 'z', 
            'roll', 'pitch', 'yaw', 
            'reachable', 
            'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'
        ])
        
        # Targeted Grid parameters (focused on manipulation area)
        x_range = [round(0.10 + i * 0.02, 2) for i in range(10)] # 0.10 to 0.28
        y_range = [round(-0.10 + i * 0.05, 2) for i in range(5)] # -0.10, -0.05, 0.0, 0.05, 0.10
        z_range = [round(0.02 + i * 0.04, 3) for i in range(5)]  # 0.02, 0.06, 0.10, 0.14, 0.18
        pitch_range = [0.0, 0.7854, 1.5708]
        
        total_points = len(x_range) * len(y_range) * len(z_range) * len(pitch_range)
        print(f"Total points to evaluate: {total_points}")
        
        count = 0
        success_count = 0
        
        for x in x_range:
            for y in y_range:
                for z in z_range:
                    for pitch in pitch_range:
                        count += 1
                        
                        # We search for top-down or forward grasps with roll=0 and yaw=0
                        roll = 0.0
                        yaw = 0.0
                        
                        joints = client.solve_ik(
                            x=x, y=y, z=z,
                            roll=roll, pitch=pitch, yaw=yaw,
                            avoid_collisions=False
                        )
                        
                        if joints is not None:
                            reachable = 1
                            success_count += 1
                            row = [x, y, z, roll, pitch, yaw, reachable] + list(joints)
                        else:
                            reachable = 0
                            row = [x, y, z, roll, pitch, yaw, reachable] + [0.0]*6
                            
                        writer.writerow(row)
                        
                        if count % 100 == 0 or count == total_points:
                            print(f"Evaluated {count}/{total_points} points | Success rate: {success_count}/{count} ({success_count/count*100:.1f}%)")
                            
    print(f"Workspace mapping complete. Saved results to {output_file}")
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import sys
import rclpy
from limo_cobot_tasks.moveit_client import MoveItClient

def main():
    rclpy.init()
    client = MoveItClient()
    if not client.wait_for_connections(timeout=5.0):
        print("Failed to connect!")
        client.destroy_node()
        rclpy.shutdown()
        return

    print("Testing dense scan around failed pick pose (pitch=0.0):")
    print("Format: X | Y | Z | Reachable? | Joint Solution")
    print("-" * 50)
    
    for x in [0.18, 0.20, 0.21, 0.22, 0.23]:
        for y in [0.0, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06]:
            for z in [0.14, 0.18]:
                sol = client.solve_ik(
                    x=x, y=y, z=z,
                    roll=0.0, pitch=0.0, yaw=0.0,
                    avoid_collisions=False
                )
                if sol is not None:
                    sol_rounded = [round(j, 3) for j in sol]
                    print(f"x={x:.3f} | y={y:.3f} | z={z:.3f} | YES | {sol_rounded}")
                else:
                    print(f"x={x:.3f} | y={y:.3f} | z={z:.3f} | NO  | -")

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

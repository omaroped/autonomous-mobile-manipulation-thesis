#!/usr/bin/env python3
"""lidar_live_viewer.py — live polar plot of the robot's /scan, over rosbridge.

Runs entirely on the LAPTOP. Needs nothing ROS-related installed here — just
`roslibpy` and `matplotlib` (both pip-installable, no ROS required on this end).
Connects to rosbridge_server running on the ROBOT over a plain WebSocket, which
is why this works over Tailscale/home networks where regular ROS 2 topic
discovery does not (that needs multicast; a WebSocket is ordinary TCP).

On the robot, before running this:
    source /opt/ros/humble/setup.bash
    source ~/limo_ros2_ws/install/setup.bash
    ros2 launch ydlidar_ros2_driver ydlidar_launch.py \
        params_file:=/home/agilex/limo_ros2_ws/real_robot/ydlidar_config/limo_tminipro.yaml
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml

Then here:
    python3 lidar_live_viewer.py [robot-host]   # defaults to the Tailscale address
"""
import sys
import math

import roslibpy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

ROBOT_HOST = sys.argv[1] if len(sys.argv) > 1 else "100.106.125.32"
ROBOT_PORT = 9090

client = roslibpy.Ros(host=ROBOT_HOST, port=ROBOT_PORT)
client.run()
print(f"connected to rosbridge at {ROBOT_HOST}:{ROBOT_PORT}: {client.is_connected}")

listener = roslibpy.Topic(client, "/scan", "sensor_msgs/LaserScan")

latest = {"angles": np.array([]), "ranges": np.array([])}


def on_scan(msg):
    angle_min = msg["angle_min"]
    angle_inc = msg["angle_increment"]
    ranges = np.array(msg["ranges"])
    angles = angle_min + np.arange(len(ranges)) * angle_inc
    # drop invalid readings (0 / inf / nan), same convention as RViz
    valid = np.isfinite(ranges) & (ranges > msg["range_min"]) & (ranges < msg["range_max"])
    latest["angles"] = angles[valid]
    latest["ranges"] = ranges[valid]


listener.subscribe(on_scan)

fig = plt.figure(figsize=(7, 7))
ax = fig.add_subplot(111, projection="polar")
ax.set_theta_zero_location("N")
ax.set_rmax(6.0)
ax.set_title(f"Live /scan — {ROBOT_HOST}")
scatter = ax.scatter([], [], s=3)


def update(_frame):
    if latest["angles"].size:
        scatter.set_offsets(np.column_stack([latest["angles"], latest["ranges"]]))
    return (scatter,)


ani = animation.FuncAnimation(fig, update, interval=200, blit=True)

try:
    plt.show()
finally:
    listener.unsubscribe()
    client.terminate()

#!/usr/bin/env python3
"""map_live_viewer.py — live occupancy grid view of /map, over rosbridge.

Same idea as lidar_live_viewer.py and camera_live_viewer.py: runs entirely on the
LAPTOP, connects to rosbridge_server on the ROBOT over a plain WebSocket (TCP),
not raw ROS 2/DDS. Built 2026-08-11 because /map specifically would not arrive
over DDS on the laptop even though it published fine on the robot and small
topics (TF) crossed the network with no issue -- the classic signature of DDS
multicast dropping fragments of a large message over WiFi (an occupancy grid is
much bigger than one UDP datagram; TF transforms are not). rosbridge sidesteps
this because it's one ordinary TCP connection, not fragmented UDP multicast.

On the robot, before running this (if not already running):
    source /opt/ros/humble/setup.bash
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml

Then here:
    python3 map_live_viewer.py [robot-host]   # defaults to the Tailscale address
"""
import sys

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import roslibpy

ROBOT_HOST = sys.argv[1] if len(sys.argv) > 1 else "100.106.125.32"
ROBOT_PORT = 9090

client = roslibpy.Ros(host=ROBOT_HOST, port=ROBOT_PORT)
client.run()
print(f"connected to rosbridge at {ROBOT_HOST}:{ROBOT_PORT}: {client.is_connected}")

topic = roslibpy.Topic(client, "/map", "nav_msgs/OccupancyGrid")

latest = {"grid": None, "extent": None, "stamp": None}


def on_map(msg):
    info = msg["info"]
    w, h, res = info["width"], info["height"], info["resolution"]
    ox = info["origin"]["position"]["x"]
    oy = info["origin"]["position"]["y"]
    if w == 0 or h == 0:
        return
    grid = np.array(msg["data"], dtype=np.int16).reshape((h, w))
    # -1 = unknown, 0-100 = occupancy probability. Map to a greyscale image:
    # unknown -> mid grey, 0 -> white (free), 100 -> black (occupied).
    img = np.full(grid.shape, 128, dtype=np.uint8)
    known = grid >= 0
    img[known] = (255 - (grid[known] * 255 // 100)).astype(np.uint8)
    latest["grid"] = img
    latest["extent"] = [ox, ox + w * res, oy, oy + h * res]


topic.subscribe(on_map)

fig, ax = plt.subplots(figsize=(8, 8))
im = ax.imshow(np.full((10, 10), 128, dtype=np.uint8), cmap="gray", vmin=0, vmax=255,
                origin="lower")
ax.set_title(f"Live /map — {ROBOT_HOST}")
ax.set_xlabel("x (m)")
ax.set_ylabel("y (m)")


def update(_frame):
    grid = latest["grid"]
    if grid is not None:
        im.set_data(grid)
        im.set_extent(latest["extent"])
    return (im,)


ani = animation.FuncAnimation(fig, update, interval=500, blit=False)

print("window open — close it to quit")
try:
    plt.show()
finally:
    topic.unsubscribe()
    client.terminate()

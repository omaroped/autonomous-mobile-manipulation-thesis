#!/usr/bin/env python3
"""camera_live_viewer.py — live color feed from the robot's camera, over rosbridge.

Same idea as lidar_live_viewer.py: runs entirely on the LAPTOP, connects to
rosbridge_server on the ROBOT over a plain WebSocket. Uses the COMPRESSED image
topic specifically (already JPEG-encoded) rather than raw — a raw 640x480 stream
over a ~100ms/jittery link would be far heavier than it needs to be; compressed
frames are a fraction of the size and this is a view-only feed anyway.

On the robot, before running this (if not already running):
    source /opt/ros/humble/setup.bash
    source ~/limo_ros2_ws/install/setup.bash
    ros2 launch orbbec_camera dabai.launch.py
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml

Then here:
    python3 camera_live_viewer.py [robot-host]   # defaults to the Tailscale address

Press 'q' in the video window to quit.
"""
import sys
import base64

import numpy as np
import cv2
import roslibpy

ROBOT_HOST = sys.argv[1] if len(sys.argv) > 1 else "100.106.125.32"
ROBOT_PORT = 9090

client = roslibpy.Ros(host=ROBOT_HOST, port=ROBOT_PORT)
client.run()
print(f"connected to rosbridge at {ROBOT_HOST}:{ROBOT_PORT}: {client.is_connected}")

topic = roslibpy.Topic(
    client, "/camera/color/image_raw/compressed", "sensor_msgs/CompressedImage")

latest_frame = {"img": None}


def on_frame(msg):
    jpg_bytes = base64.b64decode(msg["data"])
    arr = np.frombuffer(jpg_bytes, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is not None:
        latest_frame["img"] = img


topic.subscribe(on_frame)

print("window open — press 'q' to quit")
try:
    while True:
        img = latest_frame["img"]
        if img is not None:
            cv2.imshow(f"live camera — {ROBOT_HOST}", img)
        if cv2.waitKey(50) & 0xFF == ord("q"):
            break
finally:
    topic.unsubscribe()
    client.terminate()
    cv2.destroyAllWindows()

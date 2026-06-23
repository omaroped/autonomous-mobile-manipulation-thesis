#!/bin/bash
# ============================================================
# DEMO 4: Perception (Box Detection + 3D Pose)
# ============================================================
# WHAT IT SHOWS:
#   - HSV colour segmentation detecting the blue box
#   - Depth camera reading the distance
#   - 3D deprojection (pixel + depth → world coordinates)
#   - TF2 transform from camera frame to base_link
#   - Published /box_pose topic with the target position
#
# WHAT TO TELL THE PROFESSOR:
#   "This is the perception pipeline. The depth camera sees
#    the blue box, segments it by colour (HSV), reads the
#    depth at the centroid, and deprojects it into a 3D point
#    using the camera intrinsics. Then it transforms that
#    point from the camera frame into the robot's base_link
#    frame using TF2. The result is a /box_pose topic that
#    tells the arm exactly where the box is.
#    One of the challenges I found: at close range (~0.18m),
#    the depth camera over-reports the distance as ~0.27m.
#    That's a key sim-to-real finding."
#
# PREREQ: Gazebo must be running (Demo 1 or Demo 3)
# TIME: ~2 min
# ============================================================

cd ~/Desktop/Thesisorg/src/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash

echo "============================================"
echo "  DEMO 4: Perception Pipeline"
echo "============================================"
echo ""
echo "  Starting the box pose estimator..."
echo "  It will print the detected box position."
echo ""

# Launch the perception node
ros2 run limo_car box_pose_estimator &
PERCEP_PID=$!
sleep 3

echo ""
echo "  Perception is running. Let's see what it reports:"
echo ""
echo "  --- /box_pose output (5 readings) ---"

# Show a few readings
for i in 1 2 3 4 5; do
    echo "  Reading $i:"
    timeout 3 ros2 topic echo /box_pose --once 2>/dev/null | grep -A4 "position"
    echo ""
done

echo ""
echo "  --- Camera intrinsics ---"
timeout 3 ros2 topic echo /depth_camera/depth/camera_info --once 2>/dev/null | grep -A1 "k:"
echo ""

echo "  --- Active TF frames (camera → base_link) ---"
timeout 3 ros2 run tf2_ros tf2_echo base_link depth_camera_link_optical 2>/dev/null | head -5
echo ""

echo "  To see the camera feed live, run in another terminal:"
echo "    ros2 run rqt_image_view rqt_image_view /depth_camera/image_raw"
echo ""
echo "  Press Ctrl+C to stop."
kill $PERCEP_PID 2>/dev/null
wait $PERCEP_PID 2>/dev/null

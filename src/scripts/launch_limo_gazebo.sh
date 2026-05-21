#!/bin/bash
# Launch LIMO in Gazebo — works around command-line URDF size limits
# by writing the processed URDF to a file and using a parameter file.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"

source /opt/ros/humble/setup.bash

if [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
    source "${WORKSPACE_DIR}/install/setup.bash"
elif [ -f ~/thesis_ws/install/setup.bash ]; then
    source ~/thesis_ws/install/setup.bash
fi

export DISPLAY=:0
export WAYLAND_DISPLAY=wayland-0
export XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir

PKG_DIR=$(ros2 pkg prefix limo_car)/share/limo_car
XACRO_FILE=$PKG_DIR/gazebo/ackermann_with_sensor.xacro
WORLD_FILE=$PKG_DIR/worlds/empty_world.model
RVIZ_FILE=$PKG_DIR/rviz/gazebo.rviz
URDF_FILE=/tmp/limo_generated.urdf
PARAMS_FILE=/tmp/limo_rsp_params.yaml

echo "=== Processing xacro ==="
xacro $XACRO_FILE > $URDF_FILE 2>&1
echo "URDF generated: $(wc -l < $URDF_FILE) lines"

# Write parameter file for robot_state_publisher
python3 -c "
import yaml
with open('$URDF_FILE', 'r') as f:
    urdf = f.read()
params = {'robot_state_publisher': {'ros__parameters': {'robot_description': urdf, 'use_sim_time': True}}}
with open('$PARAMS_FILE', 'w') as f:
    yaml.dump(params, f, default_flow_style=False)
print('Parameter file written')
"

echo "=== Starting Gazebo server ==="
gzserver $WORLD_FILE --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so &
GZSERVER_PID=$!
sleep 5

echo "=== Starting Gazebo client ==="
gzclient &
GZCLIENT_PID=$!
sleep 2

echo "=== Starting robot_state_publisher ==="
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args --params-file $PARAMS_FILE &
RSP_PID=$!
sleep 3

echo "=== Checking robot_state_publisher ==="
ros2 node list
echo ""
ros2 topic list

echo ""
echo "=== Spawning LIMO robot ==="
ros2 run gazebo_ros spawn_entity.py \
  -topic robot_description \
  -entity limo \
  -x 0.0 -y 0.0 -z 0.1

echo ""
echo "=== Checking models in Gazebo ==="
ros2 service call /get_model_list gazebo_msgs/srv/GetModelList '{}'

echo ""
echo "=== Starting RViz ==="
ros2 run rviz2 rviz2 -d $RVIZ_FILE --ros-args -p use_sim_time:=true &
RVIZ_PID=$!

echo ""
echo "============================================"
echo "  LIMO is running in Gazebo!"
echo "  Open another terminal and run:"
echo "    wsl -d Ubuntu-22.04 -- bash -c 'source /opt/ros/humble/setup.bash && source ${WORKSPACE_DIR}/install/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard'"
echo "============================================"
echo "Press Ctrl+C to stop everything."

# Wait for Ctrl+C
trap "echo 'Shutting down...'; kill $GZSERVER_PID $GZCLIENT_PID $RSP_PID $RVIZ_PID 2>/dev/null; exit 0" SIGINT SIGTERM
wait

#!/bin/bash
# ROS 2 Humble Installation Script for Ubuntu 22.04 WSL2
# This script installs ROS 2 Humble Desktop Full + Nav2 + Gazebo + SLAM Toolbox

set -e

echo "=== Step 1/5: Adding ROS 2 repository ==="
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

echo "=== Step 2/5: Installing ROS 2 Humble Desktop Full ==="
sudo DEBIAN_FRONTEND=noninteractive apt install -y ros-humble-desktop-full

echo "=== Step 3/5: Installing development tools ==="
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool python3-pip git

echo "=== Step 4/5: Installing Nav2, SLAM Toolbox, and Gazebo packages ==="
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-cartographer \
  ros-humble-cartographer-ros \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-joint-state-publisher-gui \
  ros-humble-rqt-robot-steering \
  ros-humble-teleop-twist-keyboard \
  ros-humble-xacro \
  ros-humble-robot-state-publisher

echo "=== Step 5/5: Setting up environment ==="
# Add ROS 2 to bashrc if not already there
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

# Initialize rosdep
sudo rosdep init 2>/dev/null || true
rosdep update

echo ""
echo "============================================"
echo "  ROS 2 Humble installation complete!"
echo "  Run 'source ~/.bashrc' or open a new shell"
echo "============================================"

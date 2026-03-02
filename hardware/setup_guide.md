# LIMO Cobot Setup Guide

Step-by-step instructions for getting the Agilex LIMO Cobot operational from unboxing to first autonomous run.

> **Note:** Fill this in as you go through the setup process. These are placeholders. Document everything you actually do — future you will thank present you.

---

## 1. Unboxing & Physical Inspection

- [ ] Inspect robot for shipping damage
- [ ] Verify all components present (LIMO base, Mycobot 280, Orin Nano, Orbbec camera, cables, charger)
- [ ] Charge the battery fully before first use
- [ ] Test the emergency stop button

## 2. Orin Nano Initial Setup

- [ ] Connect HDMI, keyboard, mouse
- [ ] Power on and complete Ubuntu initial setup
- [ ] Connect to Wi-Fi
- [ ] Update system packages: `sudo apt update && sudo apt upgrade`
- [ ] Verify JetPack version: `sudo apt show nvidia-jetpack`
- [ ] Set up SSH for headless access from your PC
- [ ] Note the IP address for future SSH connections

## 3. ROS 2 Installation

- [ ] Install ROS 2 (Foxy or Humble depending on JetPack / Ubuntu version)
- [ ] Source ROS 2 in `.bashrc`
- [ ] Verify: `ros2 topic list`

## 4. LIMO Base Drivers

- [ ] Clone Agilex LIMO ROS 2 driver
- [ ] Build with `colcon build`
- [ ] Test teleop: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
- [ ] Verify robot moves with keyboard commands

## 5. Mycobot 280 Setup

- [ ] Install pymycobot: `pip install pymycobot`
- [ ] Clone Mycobot ROS 2 driver
- [ ] Build with `colcon build`
- [ ] Test arm: move to home position via command

## 6. Depth Camera Setup

- [ ] Install Orbbec camera SDK / ROS 2 package
- [ ] Verify RGB and depth streams: `ros2 topic echo /camera/color/image_raw`
- [ ] Check point cloud: `ros2 topic echo /camera/depth/points`

## 7. Networking

- [ ] Set up a fixed IP or hostname for the Orin Nano
- [ ] Configure ROS 2 DDS discovery between robot and dev machine
- [ ] Test: run publisher on robot, subscriber on PC

## 8. First Autonomous Test

- [ ] Run SLAM (Cartographer or SLAM Toolbox)
- [ ] Save a map
- [ ] Launch Nav2 with the saved map
- [ ] Send a goal via RViz2
- [ ] Celebrate when it gets there

---

> Update this document with actual commands, version numbers, and troubleshooting notes as you work through each step.
> Link to `troubleshooting.md` for anything that goes wrong.

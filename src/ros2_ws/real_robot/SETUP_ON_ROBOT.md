# Rebuilding the robot side from scratch

Everything in this directory is authored on the laptop and deployed to the Jetson.
**If the Jetson is ever reflashed or lost, this file is how you get back.**

Written 2026-08-04 after the Jetson failed to boot (UEFI completes, then a blinking
cursor, no console, no network). At that moment four working files existed *only* on the
robot and were in no repository. They are reproduced here so that never costs anything again.

---

## What lives where

| file | deploy to |
|---|---|
| `arm_bridge.py` | `~/limo_ros2_ws/real_robot/` |
| `real_robot.launch.py` | `~/limo_ros2_ws/real_robot/` |
| `fov_scan.py` | `~/` (or anywhere) |
| `ydlidar_config/limo_tminipro.yaml` | `~/limo_ros2_ws/src/ydlidar_ros2_driver/params/` |
| `ydlidar_config/measure_fov.yaml` | `~/limo_ros2_ws/src/ydlidar_ros2_driver/params/` |

Deploy everything:

```bash
# from the laptop
cd ~/Desktop/Thesisorg/src/ros2_ws
scp -r real_robot agilex@<robot-ip>:~/limo_ros2_ws/
scp real_robot/ydlidar_config/*.yaml \
    agilex@<robot-ip>:~/limo_ros2_ws/src/ydlidar_ros2_driver/params/
```

---

## The one patch that is NOT a file: ydlidar_launch.py

`ydlidar_ros2_driver/launch/ydlidar_launch.py` ships with the **pre-Foxy** launch API
(`node_executable`, `node_name`, `node_namespace`), removed in Galactic. On Humble the
launch aborts before the driver ever starts:

```
TypeError: LifecycleNode.__init__() missing 2 required keyword-only arguments:
'name' and 'namespace'
```

Fix (idempotent — safe to re-run):

```bash
cd ~/limo_ros2_ws/src/ydlidar_ros2_driver/launch
cp -n ydlidar_launch.py ydlidar_launch.py.bak
sed -i 's/node_executable=/executable=/; s/node_name=/name=/; s/node_namespace=/namespace=/' \
    ydlidar_launch.py
grep -n -A8 "LifecycleNode(package" ydlidar_launch.py    # verify
cd ~/limo_ros2_ws && colcon build --packages-select ydlidar_ros2_driver
```

The `node_name = '...'` *variable assignment* near the top must stay as-is; the `sed`
patterns only match the no-space keyword form, so it is safe.

**Note:** the launch file's default `params_file` is `TminiPro.yaml`, which is the correct
model for this robot. Override it with `limo_tminipro.yaml` for the ±100° production config.

---

## ⚠️ THE MODULE IS AN ORIN **NX**, NOT AN ORIN NANO

Read from the UEFI setup screen 2026-08-04:

```
NVIDIA Jetson Orin NX Engineering Reference Developer Kit
Orin                     1.51 GHz
36.4.0-gcid-37537400     8005 MB RAM
```

**Jetson Orin NX, 8 GB. Firmware 36.4.0 (L4T 36.4).** Every earlier document in this
project — including docs/robot_setup/jetson_flash_guide.md — says "Orin Nano". That is
WRONG and was never verified against the hardware.

**In SDK Manager select Jetson Orin NX.** Choosing Orin Nano flashes the wrong device tree
and board config; at best it fails, at worst it leaves the module unbootable.

Storage is a **128 GB NVMe SSD** (`nvme0n1`), not eMMC and not SD. The rootfs is
`/dev/nvme0n1p1`, PARTLABEL `APP`, ext4.

## Full rebuild order after a reflash

1. Flash JetPack 6.x, install ROS 2 Humble.
2. `sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup \
   ros-humble-slam-toolbox ros-humble-teleop-twist-keyboard ros-humble-nav2-map-server`
3. `pip install pymycobot==3.3.4` — **pinned**; 4.x changed the API surface.
4. Restore driver sources from `~/Desktop/limo_backup.tar.gz` (laptop):
   `limo_ros2`, `ydlidar_ros2_driver`, `YDLidar-SDK`, `OrbbecSDK_ROS2` into
   `~/limo_ros2_ws/src/`. Do **not** restore `navigation2`, `teb_local_planner`,
   `costmap_converter`, `rf2o_laser_odometry` — the thesis uses stock Nav2 plugins.
5. Apply the `ydlidar_launch.py` patch above.
6. Copy the files in this directory to their destinations.
7. udev: YDLidar rule `10c4:ea60 -> /dev/ydlidar`; run
   `OrbbecSDK_ROS2/orbbec_camera/scripts/install_udev_rules.sh` for the camera.
8. `cd ~/limo_ros2_ws && rosdep install --from-paths src --ignore-src -r -y && colcon build`

## ⚠️ ALWAYS SHUT DOWN CLEANLY

```bash
sudo shutdown -h now      # then WAIT for it to power off
```

**Never cut power to a running Jetson.** Still good practice -- but note the 2026-08-04
boot failure was NOT caused by this, despite an earlier version of this file saying so.

**CORRECTED root cause:** the rootfs was never corrupted. It mounts clean and every file is
intact; the PARTUUID the initrd looks for matches `nvme0n1p1` exactly. UEFI and a generic
Ubuntu kernel both enumerate the SSD, but the L4T kernel intermittently does not -- a
**physical PCIe/M.2 connection fault**. A driven robot vibrates and the M.2 card backs out
of its slot slightly. Swapping the initrd in `extlinux.conf` did not help, which is
consistent: no software fix reaches a link that is not there.

**Fix: reseat the NVMe.** Remove the acrylic top plate (4 screws, the one carrying the
lidar), unscrew the M.2 retaining screw, pull the card, inspect the contacts, push it back
in firmly and re-tighten. See `docs/jetson_boot_failure_2026-08-04.md` for the full
evidence trail and every route already ruled out.

Backup discipline still applies regardless: `scp` anything created on the robot back to the
repo the same day.

## Hardware facts worth not rediscovering

| device | port / id | note |
|---|---|---|
| myCobot arm | `/dev/ttyACM0`, CH340 `1a86:55d4` | **verified empirically** — guessing from chip type got it backwards once |
| YDLidar T-mini Pro | `/dev/ttyUSB0` → `/dev/ydlidar`, CP2102 `10c4:ea60` | 230400 baud, dual-channel |
| Orbbec DaBai DC1 | `2bc5:0657` + `2bc5:0557` | two USB interfaces |
| LIMO base | `/dev/ttyTHS1` | publishes `odom` → **`base_link`** (no `base_footprint`) |

- The ATOM needs **~2 s** after the serial port opens before it answers `get_angles()`.
- The lidar publishes **Best Effort** QoS — RViz shows nothing on the default Reliable.
- Orbbec's container survives the first Ctrl-C and keeps a launch alive; press it twice
  or `pkill -f component_container`.

# Hardware & Software Requirements

Everything needed to execute this thesis, from hardware on the bench to software on the machine.

---

## Hardware

### Robot Platform: Agilex LIMO Cobot

| Component | Specification |
|-----------|--------------|
| Base | LIMO PRO, 4-wheel differential steering |
| Arm | Mycobot 280 M5, 6-DoF, 250 g payload, 280 mm reach |
| Compute | NVIDIA Jetson Orin Nano (8 GB LPDDR5, 40 TOPS) |
| Camera | Orbbec Dabai depth camera (RGB-D) |
| Load capacity | 4 kg (base) |
| Weight | ~4.8 kg |
| Runtime | ~2.5 hours on battery |
| Max speed | 1 m/s |
| Display | 7-inch onboard screen |
| Control | Mobile app, SSH, direct ROS commands |

### Additional Hardware (Check Availability)

| Item | Purpose | Status |
|------|---------|--------|
| **External M.2 NVMe SSD + USB 3.2 Enclosure** | **Native Ubuntu boot drive for full ROS 2 performance** | [ ] |
| USB flash drive (8 GB+) | To create the Ubuntu 22.04 installer | [ ] |
| USB keyboard + mouse | Direct interaction with Orin Nano | [ ] |
| HDMI monitor / cable | Initial setup and debugging | [ ] |
| USB-C hub | Peripheral connections | [ ] |
| Ethernet cable | Firmware updates, large data transfers | [ ] |
| MicroSD card (128 GB+) | Orin Nano OS if not using NVMe | [ ] |
| Spare batteries | Extended experiment sessions | [ ] |
| Charging station | Keep robot charged between sessions | [ ] |
| Colored LEGO Duplo blocks (5 colors) | Target objects per EAI-WS spec | [ ] |
| Storage shelf (40×90×90 cm) | Warehouse track simulation | [ ] |
| Grey Euro-boxes (30×40×12 cm) | Container handling experiments | [ ] |
| Workbench / table | Assembly task workspace | [ ] |
| Measuring tape + markers | Experiment setup, arena layout | [ ] |
| Safety equipment (e-stop tester) | Required for all autonomous operation | [ ] |

### Lab Space Requirements

- Flat floor, minimum ~3×4 m clear area for navigation experiments
- Power outlets accessible for robot charging and workbench station
- Wi-Fi network for SSH access to Orin Nano from development machine
- Ideally: fixed camera mount above workspace for recording experiments

---

## Software

### On the Orin Nano (Robot)

| Software | Version | Purpose |
|----------|---------|---------|
| Ubuntu | 20.04 or 22.04 (JetPack) | OS |
| JetPack SDK | 6.x | NVIDIA SDK for Orin Nano |
| ROS 2 | Foxy or Humble | Robotics middleware |
| Nav2 | Latest | Autonomous navigation |
| SLAM Toolbox or Cartographer | Latest | Mapping and localization |
| MoveIt 2 | Latest | Arm motion planning |
| Mycobot ROS 2 driver | Latest | Arm control via ROS |
| Agilex LIMO ROS 2 driver | Latest | Base control via ROS |
| OpenCV | 4.x | Image processing |
| PyTorch + TensorRT | Compatible with JetPack | Perception models |
| Python | 3.8+ | Scripting, perception, utilities |

### On the Development Machine

| Software | Purpose |
|----------|---------|
| **Native Ubuntu 22.04 (on External SSD)** | **Local ROS 2 development, Gazebo simulation, full GPU/USB access** |
| Windows 11 (Internal SSD) | General use, document writing |
| VS Code + Remote SSH | Code editing on the Orin Nano |
| Git | Version control |
| TeX Live / MiKTeX | Thesis writing (LaTeX) |
| Python 3.10+ | Local scripting, data analysis, plotting |
| Gazebo Harmonic (on Ubuntu SSD) | Simulation with native GPU performance |
| Blender (optional) | 3D visualization, URDF inspection |
| Isaac Sim (optional) | NVIDIA simulation platform |

### Accounts & Services

| Service | Purpose |
|---------|---------|
| GitHub | Repository hosting (private) |
| Overleaf (optional) | Collaborative LaTeX if professor wants access |
| Google Scholar alerts | Stay updated on related publications |

---

## What to Acquire First

Priority order for things you may not already have:

1. **External M.2 NVMe SSD + USB-C 3.2 Gen 2 Enclosure** — Essential for a high-performance, native ROS 2 development environment without touching your internal Windows drive. (Alternatively: Samsung T7 Portable SSD).
2. **Colored LEGO Duplo blocks** — 5 colors, at least 10 of each. These are the standardized objects from the EAI-WS paper. Cheap and easy to get.
3. **USB peripherals** — For initial Orin Nano setup (keyboard, mouse, HDMI cable).
4. **Storage shelf** — The standard Bauhaus shelf (40×90×90 cm) referenced in the paper. ~€20.
5. **Euro-boxes** — Grey, 30×40×12 cm. Optional but useful for container handling experiments. ~€5 each.

Everything else is software or things you likely already have access to through the university lab.

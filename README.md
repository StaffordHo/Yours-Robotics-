# Yours Robotics (优时机器人) — Central Staff Repository

Welcome to the central repository for **Yours Robotics**. This repository serves as the unified hub for documentation, software packages, web control applications, AI VLA agents, and engineering handbooks for staff reference and collaboration.

---

## 📁 Repository Structure

```text
Yours-Robotics/
├── README.md                                         # Main Organization README & Quickstart
├── docs/                                             # Official Manuals & Engineering Handbooks
│   ├── Yours_Robotics_XiaoChe_Engineering_Handbook.md # Comprehensive Engineering & Troubleshooting Handbook
│   ├── Robot_Mapping_Manual_Sales_Edition_English.md  # Translated English Operations Manual (建图教程_销售版)
│   └── 建图教程_销售版.pages                             # Original Chinese Sales Operations Manual
├── dashboard/                                        # WebGL Control Dashboard & AI Agents
│   ├── index.html                                    # WebGL 3D RViz Map, VLA Prompt Suite, 5-Step SLAM Wizard
│   ├── vla_agent/
│   │   └── yours_vla_agent.py                        # Vision-Language-Action (VLA) AI ROS Node
│   └── slam/
│       └── yours_slam_exploration.py                 # Autonomous SLAM Frontier Exploration Node
└── ros_packages/                                     # Onboard ROS Packages
    └── yours_navigation/                             # Yours Navigation, Obstacle Avoidance & twist_mux
```

---

## 🤖 Robot Specifications — 小车 (XiaoChe)
* **Robot Name**: **小车** (XiaoChe)
* **Organization**: **Yours Robotics**
* **Compute**: Nvidia Jetson AGX / Xavier (Ubuntu 20.04 ARM64, ROS Noetic)
* **Sensors**:
  * **3D LiDAR**: Livox Mid-360 (`/livox/lidar` & `/scan` @ 10 FPS)
  * **RGB-D Camera**: RealSense D455 (`/camera/color/image_raw/compressed` @ 15 FPS)
  * **Chassis Actuators**: Storage Door Motor (`/yours_base/refrigerator_door`), Solenoid Locks (`/yours_base/locks_ctrl`), Peltier Climate Control (`/yours_base/refrigerator_mode`).

---

## 🛠️ Quick Start Guide for Staff

### 1. Web Dashboard & WebGL 3D RViz Visualizer
Open `dashboard/index.html` in Chrome / Edge and click **Connect** (`ws://127.0.0.1:9090`):
* **Live 3D RViz Map**: Renders live occupancy grids (`/map`), cyan global plans, and emerald green DWA obstacle avoidance trajectories.
* **3D Obstacle Color-Coding**: Red ($<0.6\text{m}$), Yellow ($0.6\text{m}-1.2\text{m}$), Cyan ($>1.2\text{m}$).
* **Official Teleop Key Matrix**: Use `u i o / j k l / m , .` keys (or on-screen buttons) to drive at safe **0.20 m/s**.
* **Universal LiDAR Safety Guard**: Automatically blocks forward motion if an obstacle is within $0.55\text{m}$.

### 2. Official 5-Step SLAM Mapping Workflow
Follow the 5-step wizard built into the dashboard or read [`docs/Robot_Mapping_Manual_Sales_Edition_English.md`](docs/Robot_Mapping_Manual_Sales_Edition_English.md):
1. `sudo systemctl stop yours_robot_start.service` (Stop background nav)
2. `bash build_two_map.sh start` (Start dual-layer Cartographer / ORB_SLAM2)
3. `roslaunch yours_teleop keyboard_teleop.launch` (Teleop drive @ 0.20 m/s, trigger yellow loop closure)
4. Press `Enter` to save map (saving takes ~2x mapping duration)
5. `python3 map_deal.py` & `sudo systemctl start yours_robot_start.service` (Align layers & restore nav service)

### 3. Key ROS Launch Commands Reference
* **Manual Teleop**: `roslaunch yours_teleop keyboard_teleop.launch`
* **Obstacle Avoidance Nav**: `roslaunch yours_navigation followpath.launch`
* **Human Eye/Face Tracking**: `roslaunch yours_navigation lidar_test.launch`
* **3D Camera Depth Guard**: `roslaunch yours_navigation vision_test.launch`

---

## 🤝 Contribution Guidelines
All Yours Robotics staff are welcome to submit updates, documentation, product specs, and code improvements:
1. Clone the repository: `git clone https://github.com/StaffordHo/Yours-Robotics-.git`
2. Create a topic branch: `git checkout -b feature/your-feature-name`
3. Commit and push your changes for review.

---
*Yours Robotics (优时机器人) — Building Intelligent Autonomous Systems*

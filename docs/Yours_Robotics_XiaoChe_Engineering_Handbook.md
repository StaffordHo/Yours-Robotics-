# Yours Robotics — 小车 (XiaoChe) Engineering Handbook & Troubleshooting Guide

> **Target Audience**: New engineers, robotics operators, sales engineers, and developers working with **小车** (Yours Robotics).  
> **Purpose**: Standalone reference guide to troubleshoot issues, understand system architecture, prevent recurring mistakes, and operate the robot independently.

---

## 📑 Table of Contents
1. [System Architecture & Robot Specs](#1-system-architecture--robot-specs)
2. [Major Engineering Problems Overcome & Technical Solutions](#2-major-engineering-problems-overcome--technical-solutions)
   * [Problem 1: Robot Stationary During Waypoint & VLA Navigation](#problem-1-robot-stationary-during-waypoint--vla-navigation)
   * [Problem 2: Robot Spinning Continuously in Place](#problem-2-robot-spinning-continuously-in-place)
   * [Problem 3: Camera Feed Blank & 3D Pointcloud Not Rendering](#problem-3-camera-feed-blank--3d-pointcloud-not-rendering)
   * [Problem 4: Robot Colliding into Walls & Obstacles](#problem-4-robot-colliding-into-walls--obstacles)
   * [Problem 5: SLAM Mapping Stopping After < 1 Minute](#problem-5-slam-mapping-stopping-after--1-minute)
   * [Problem 6: Teleop Key Binding Mismatch](#problem-6-teleop-key-binding-mismatch)
3. [Official Yours Robotics 5-Step SLAM Mapping Workflow](#3-official-yours-robotics-5-step-slam-mapping-workflow)
4. [Software Stack & Priority Channel Architecture](#4-software-stack--priority-channel-architecture)
5. [Newcomer Quick Troubleshooting Matrix](#5-newcomer-quick-troubleshooting-matrix)

---

## 1. System Architecture & Robot Specs

### 🤖 Robot Profile
* **Robot Name**: **小车** (XiaoChe — represented strictly in Chinese characters)
* **Organization**: **Yours Robotics**
* **Chassis Type**: Omnidirectional / Differential Drive Service Robot
* **Onboard Compute**: Nvidia Jetson AGX / Xavier (Ubuntu 20.04 ARM64, ROS Noetic)
* **Onboard Sensors**:
  * **3D LiDAR**: Livox Mid-360 (`/livox/lidar` & `/scan` @ 10 FPS)
  * **RGB-D Camera**: RealSense D455 (`/camera/color/image_raw/compressed` @ 15 FPS)
  * **Mono Front Camera**: `/front_camera/image_raw` (6 FPS)
  * **Storage Compartment**: NTC Temperature Sensor (`/yours_base/sensors_celsius`), Peltier Climate Unit (`/yours_base/refrigerator_mode`), Electric Door Motor (`/yours_base/refrigerator_door`), Solenoid Latch Lock (`/yours_base/locks_ctrl`).
  * **BMS Battery**: Smart BMS Telemetry (`/yours_base/battery_status`).
  * **Voice & Expressions**: Speaker TTS (`/chat_audio_text`), Face Eye Position (`/yours_base/face_position`).

---

## 2. Major Engineering Problems Overcome & Technical Solutions

### Problem 1: Robot Stationary During Waypoint & VLA Navigation
* **Symptom**: Clicking 3D Waypoint buttons (e.g. `Lobby Station`) or issuing VLA natural language instructions logged messages, but the physical robot remained stationary.
* **Root Cause Analysis**: Pose target messages were published to `/move_base_simple/goal`. However, if the full ROS `move_base` planner node was not active on the Jetson, target pose messages sat unconsumed in RAM without generating wheel motor velocity commands (`/cmd_vel`).
* **Technical Solution**:
  Built a **Direct Active Velocity Driver (`driveRobotToTargetPose`)** inside `index.html`.
  When a target pose is received, the dashboard calculates the distance and heading vector from live odometry (`/yours_base/odom`) and dispatches velocity directly over `/yours_cmd_vel/keyop_ctrl` (the same working priority topic used by manual teleop). Once within 0.25m of the target, the driver automatically halts the motors.

---

### Problem 2: Robot Spinning Continuously in Place
* **Symptom**: When waypoint navigation or VLA goals were triggered, **小车** rotated continuously in circles without driving forward.
* **Root Cause Analysis**: The target angle calculation evaluated global map angle ($\text{atan2}(\Delta y, \Delta x)$) without subtracting `robotYaw` (the robot's current orientation heading from `/yours_base/odom` quaternion). Because `robotYaw` was missing, `angleError` remained constantly large, making the controller think the robot was misaligned and commanding max angular rotation (`angular.z`).
* **Technical Solution**:
  1. Extracted `robotYaw` from odometry quaternion ($w, x, y, z$):
     $$\text{robotYaw} = \text{atan2}\left(2(wq + xy), 1 - 2(y^2 + z^2)\right)$$
  2. Calculated normalized relative heading error:
     $$\text{angleError} = \text{targetAngle} - \text{robotYaw} \quad \in [-\pi, \pi]$$
  3. Control logic: Turns toward the target heading first, then drives straight forward to the destination.

---

### Problem 3: Camera Feed Blank & 3D Pointcloud Not Rendering
* **Symptom**: Camera viewport displayed fallback text; 3D spatial visualizer showed 0 points.
* **Root Cause Analysis**:
  1. Camera selector defaulted to `/front_camera/image_raw` (uncompressed raw `sensor_msgs/Image`), whereas ROSBridge WebSocket base64 encoding streams cleanly on `/camera/color/image_raw/compressed` (15 FPS RealSense JPEG).
  2. LiDAR LaserScan parser evaluated `msg.ranges[i]` with strict range checks where out-of-range returns (`NaN` / `Infinity`) caused JavaScript comparisons to silently evaluate to `false`. In addition, Three.js `BufferGeometry` was missing `setDrawRange(0, pointCount)`.
* **Technical Solution**:
  1. Defaulted camera selection to `/camera/color/image_raw/compressed` (15 FPS RealSense Base64 JPEG stream).
  2. Updated LaserScan validator: `isFinite(r) && r > 0.05 && r < 25.0`.
  3. Added `persistentPointCloud.geometry.setDrawRange(0, pointCount)`.
  4. Implemented dynamic obstacle color-coding:
     * **Red Points (`#ef4444`)**: Close obstacle danger ($< 0.60\text{m}$).
     * **Yellow Points (`#f59e0b`)**: Caution distance ($0.60\text{m} - 1.20\text{m}$).
     * **Cyan Points (`#06b6d4`)**: Safe clear space ($> 1.20\text{m}$).

---

### Problem 4: Robot Colliding into Walls & Obstacles
* **Symptom**: Robot bumped into walls during teleop, waypoint navigation, and SLAM mapping.
* **Root Cause Analysis**: Velocity publisher `publishVel(linearX, angularZ)` dispatched velocity directly without checking live 3D LiDAR obstacle clearance (`minDist`).
* **Technical Solution**:
  Built a **Universal Active LiDAR Collision Prevention Interlock** inside `publishVel()`:
  ```javascript
  if (linearX > 0 && typeof slamCurrentMinDist === 'number' && slamCurrentMinDist < 0.55) {
      logConsole(`⚠️ COLLISION PREVENTED! Obstacle detected at ${slamCurrentMinDist}m. Forward velocity interlocked.`);
      linearX = 0.0; // Block forward motion to prevent crash!
  }
  ```
  *When driving forward, if an obstacle is within 0.55m (0.90m during SLAM), forward motion is instantly overridden to `0.0`, while rotation remains enabled so the operator/robot can steer away safely.*

---

### Problem 5: SLAM Mapping Stopping After < 1 Minute
* **Symptom**: Autonomous SLAM mapping exploration loop stopped after moving for 30–50 seconds.
* **Root Cause Analysis**: The step counter cycle terminated after reaching its upper modulo bound.
* **Technical Solution**:
  Upgraded `startClientSideFrontierExploration()` into an unending spatial exploration loop (`cycle = step % 120`) that runs continuously until explicitly stopped by clicking `🛑 Pause SLAM Mapping` or `💾 Save Occupancy Map`.

---

### Problem 6: Teleop Key Binding Mismatch
* **Symptom**: Confusion when attempting to drive using standard WASD keys versus official terminal teleop.
* **Root Cause Analysis**: Official `yours_teleop` (`keyboard_teleop.launch`) uses the **`u i o / j k l / m , .` 3x3 key layout**.
* **Technical Solution**:
  Updated `index.html` UI and JS `keydown` listeners to strictly map the official `u, i, o, j, k, l, m, ,, .` key matrix matching `yours_teleop_keyboard`.

---

## 3. Official Yours Robotics 5-Step SLAM Mapping Workflow

*(Extracted from official manual `建图教程_销售版.pages` / `Robot_Mapping_Manual_Sales_Edition_English.md`)*

```text
Step 1: Stop Nav Service -> sudo systemctl stop yours_robot_start.service
Step 2: Start Dual SLAM Mode -> bash build_two_map.sh start
Step 3: Teleop Drive & Loop Closure -> roslaunch yours_teleop keyboard_teleop.launch (Drive at 0.2 m/s, trigger yellow loop closure)
Step 4: Save Dual Map -> Press Enter (Saving takes ~2x mapping duration)
Step 5: Align Layers & Restore Service -> python3 map_deal.py & sudo systemctl start yours_robot_start.service
```

---

## 4. Software Stack & Priority Channel Architecture

*(Extracted from `~/Downloads/yours_navigation/config/twist_mux.yaml`)*

### 🔀 `twist_mux` Velocity Priority Levels
| Channel | Topic Name | Priority Level | Description |
| :--- | :--- | :---: | :--- |
| **`e_stop`** | `e_stop` | **255** | Absolute highest priority hardware emergency lock. |
| **`teleop_twist_keyboard`** | `/cmd_vel` or `/yours_cmd_vel/keyop_ctrl` | **100** | High priority manual keyboard/joystick override. |
| **`follow_path`** | `/follow/cmd_vel` | **10** | Autonomous navigation path follower. |

### 🛠️ Key ROS Launch Commands Reference
* **Manual Teleop**: `roslaunch yours_teleop keyboard_teleop.launch`
* **Dual SLAM Mapping**: `bash /home/nvidia/nvidia_catkin_ws/tools/two_layer_map/build_two_map.sh start`
* **ORB_SLAM2 Mapping**: `roslaunch ORB_SLAM2 yours_slam.launch`
* **Obstacle Avoidance Nav**: `roslaunch yours_navigation followpath.launch`
* **LiDAR Human & Face Tracking**: `roslaunch yours_navigation lidar_test.launch`
* **3D Vision Depth Guard**: `roslaunch yours_navigation vision_test.launch`

---

## 5. Newcomer Quick Troubleshooting Matrix

| Issue | Typical Cause | Solution / Fix |
| :--- | :--- | :--- |
| **Robot does not move when pressing `i j k l` in terminal** | `yours_robot_start.service` is still running in background, blocking resource | Run `sudo systemctl stop yours_robot_start.service` first |
| **Robot spins in circles when goal sent** | Odometry heading orientation (`robotYaw`) not subtracted from target angle | Check odometry subscriber quaternion conversion in dashboard |
| **Forward drive blocked / log says COLLISION PREVENTED** | Object or wall within 0.55m of LiDAR sensors | Rotate robot away from obstacle using `J` or `L` keys to clear path |
| **E-STOP locked / robot unresponsive** | `e_stop` topic received `true` (Priority 255) | Publish `false` to `e_stop` or click Connect on dashboard to clear |
| **Dual-layer map save looks frozen** | Dual-layer saving requires ~2x mapping duration | **Do NOT reboot or close terminal**; wait for save completion |
| **Camera feed blank on web dashboard** | Dropdown set to uncompressed topic | Select `/camera/color/image_raw/compressed` (15 FPS JPEG) |

---
*Handbook Compiled for Yours Robotics — 小车 Engineering Suite*

# Robot Mapping Operational Manual
## — Beginner Hands-On Edition —

**Target Audience**: Teammates encountering robot mapping for the first time, including sales, clients, operations, and non-technical staff.  
**Reading Suggestion**: Read sequentially from top to bottom without skipping steps. Each step explains *Why we do it, How to do it, and What success looks like*.

---

## I. Important Concepts First

### 1.1 What is Mapping?
Simply put, mapping allows the robot—when arriving at a new venue for the first time—to drive around by itself once, drawing and saving a spatial map of the area.

*Analogy*: Just like moving into a new apartment, on day one you walk through every room to remember the layout. After that, you can walk from the living room to the bedroom even with your eyes closed. The robot is the same: without this map, it doesn't know where it is, doesn't know where the walls are, and cannot navigate to client-specified locations.

Therefore, **mapping is a mandatory first step** before any robot is put into service, and must be performed once for every new location.

### 1.2 One-Sentence Summary of the Entire Workflow
All you need to do is:
1. Put the robot into **"I am drawing a map"** mode.
2. Remote control the robot using your keyboard to walk a complete loop around the venue.
3. Save the map after finishing the loop.
4. Upload the map to the management backend platform.

*Total Duration*: Approximately **15–30 minutes** (depending on venue size).

---

### 1.3 Tools You Will Use (Get Familiar)

* **Web Terminal**: A chat-box-like black screen with white text. You type, it executes. Robot commands such as "Start Mapping" and "Stop Mapping" are entered here.
* **SSH**: A method for remotely logging into the robot. Think of the robot as a computer in the distance; your laptop connects over SSH, allowing you to operate it just like your own computer.
* **RViz**: A 3D visual window showing what the robot sees in real time. During mapping, you monitor this screen as the map gradually fills in.
* **Keyboard Teleop**: Uses key presses on your keyboard as a steering wheel to drive the robot.

---

### 1.4 How to Choose Between the Two Mapping Modes?

* **Dual-Layer Map (Recommended)**: Use when the venue has items at varying heights, such as glass doors, low cabinets, railings, or overhangs. The robot generates two map layers—one for localization and one for obstacle avoidance.
* **Single-Layer Map**: For small, simple venues with uniform wall structures.

> **Rule of Thumb**: If unsure, **always select Dual-Layer Map**. It rarely fails.

---

## II. Preparation Before Starting

### 2.1 Pre-Flight Checklist
Before starting, verify all of the following:
* [x] Robot is powered ON with sufficient battery (recommended **> 80%**).
* [x] Robot and your laptop are connected to the **same Wi-Fi network**.
* [x] Your laptop browser can access the robot's Web Terminal.
* [x] Environment is static (minimize moving people or objects during mapping).
* [x] Lighting is normal without heavy glass/mirror floor reflections.

---

### 2.2 How to Open the Web Terminal?
Access via noVNC in 4 steps:
1. Open browser (Chrome / Edge / Safari), enter Robot IP + Port: `192.168.1.168:6080`
2. Click `vnc.html` in the file list.
3. Click the **Connect** button on the noVNC page and enter password.
4. Open terminal on the robot desktop ("Web Terminal").

---

### 2.3 How to Connect via SSH?

### 2.3 How to Connect via SSH?

**Primary SSH Remote Connection (Cloud Port Forwarded)**:
Open Terminal (Mac Command + Space -> `Terminal` or Windows `cmd`/`PowerShell`) and run:
```bash
ssh nvidia@47.93.217.30 -p 10914
```
Type `yes` when prompted on first connection, then enter password `nvidia`.

**Local Wi-Fi Network Connection (LAN Direct)**:
If laptop and robot are connected to the same local Wi-Fi router (`192.168.1.x`), run:
```bash
ssh nvidia@192.168.1.168
```

---

## III. Formal Mapping Execution (Core Workflow)

> ⚠️ **Crucial Rule**: Keep robot movement slow during mapping (**speed capped at ~0.2 m/s**). Avoid sharp in-place turns or excessive reversing.

---

### Step 1 · Stop Automatic Navigation Service

* **Why**: The robot runs an automatic navigation service by default on boot. This service competes for hardware resources with the mapping program and must be stopped first.
* **How**: In Terminal 1, SSH into the robot and execute:
  ```bash
  sudo systemctl stop yours_robot_start.service
  ```
* **Success Indicator**: Enter password `nvidia` if prompted; terminal returns cleanly to a new prompt line without errors.

---

### Step 2 · Start Mapping Program

* **Why**: Puts the robot into "drawing map" mode.
* **How**: In the same Terminal 1 window, run:
  * **Dual-Layer Map (Recommended)**:
    ```bash
    bash /home/nvidia/nvidia_catkin_ws/tools/two_layer_map/build_two_map.sh start
    ```
  * **Single-Layer Map**:
    ```bash
    bash /home/nvidia/mapping_script.sh
    ```
* **Success Indicator**: Terminal displays the official mapping initialization logs and waits for completion:
  ```text
  [2026-08-05 15:44:18] ===== Mapping Service Start =====
  开始执行地图构建流程...
  [2026-08-05 15:44:19] 录包启动 PID=1612617
  wait 2DMappingInitialize...
  wait cartographer_ros...
  ====================================
  ==== 遥控完成后按回车 ====
  ```
  > 💡 *Keep Terminal 1 open in the background while driving.*

---

### Step 3 · Start Keyboard Teleoperation

* **Why**: Step 2 prepares the robot to receive the map, but the robot won't move by itself. You must drive it.
* **How**: **Open a 2nd Terminal Tab/Window** on your laptop:
  1. SSH into the robot:
     ```bash
     ssh nvidia@47.93.217.30 -p 10914
     ```
  2. Source ROS environment & launch teleop node:
     ```bash
     source /opt/ros/noetic/setup.bash
     roslaunch yours_teleop keyboard_teleop.launch
     ```
* **Success Indicator**: Terminal 2 displays the official key matrix:
  ```text
  Control Your Binobot!
  ---------------------------
  Moving around:
     u    i    o
     j    k    l
     m    ,    .

  currently:	speed 0.4	turn 0.4
  ```
  > 💡 *Keep Terminal 2 active and focused to receive your key presses.*

---

### Step 4 · Drive Robot Around Area & Loop Closure

* **Key Operation Guidance**:
  * Set speed to **0.2 m/s** (press `x` to reduce linear speed for map quality).
  * Drive slowly along wall edges, covering all hallways, rooms, and corners.
  * Turn slowly using `j` (left) / `l` (right); avoid abrupt in-place spins.
  * Periodically revisit previously mapped areas to trigger **Loop Closure (回环检测)**.

* **What is Loop Closure?**
  As the robot maps, small drift errors accumulate. When it recognizes a previously visited area, it automatically recalculates and corrects map alignment.
* **Success Indicator**: In 3D RViz / WebGL map, when revisiting an area, the map temporarily flashes **yellow** (loop closure active). Pause for a few seconds until normal colors return.

---

### Step 5 · Save Map

* **Why**: Saves the map from RAM to disk (`/home/nvidia/maps/map/`).
* **How**: Switch back to **Terminal 1** (from Step 2) and press **`Enter`**.
* **Success Indicator**: Terminal displays map saving progress:
  ```text
  保存地图中...
  [INFO]: Waiting for the map
  ```
* ⚠️ **Note for Dual-Layer Maps**: Saving takes approximately **2x the duration of mapping** (e.g., 10 min mapping = ~20 min saving). Terminal may appear paused; **do not close terminal or reboot until complete**.

---

## IV. Map Post-Processing & Alignment

### 4.1 Dual-Layer Map Alignment
1. Navigate to tool folder in Web Terminal:
   ```bash
   cd /home/nvidia/nvidia_catkin_ws/tools/two_layer_map
   ```
2. Launch alignment tool:
   ```bash
   python3 map_deal.py
   ```
3. Load map folder `/home/nvidia/maps/map/`.
4. Drag Low-Z layer to align with High-Z layer. Use eraser tool to remove reflection noise dots.
5. Click **Export Aligned Map**.
6. Verify 4 files generated in `/home/nvidia/maps/map/`:
   * `map.pgm` & `map.yaml` (Localization)
   * `map_nav.pgm` & `map_nav.yaml` (Navigation & Obstacle Avoidance)

---

## V. Upload Map to Backend & Task Setup

### 5.1 Upload Map Files
1. Open browser to `map.yours.xyz` (do NOT type `https://`).
2. Go to **Map Management** -> **Upload Map**.
3. Fill 4 file fields:
   * Navigation PGM: `map_nav.pgm`
   * Navigation YAML: `map_nav.yaml`
   * Localization PGM: `map.pgm`
   * Localization YAML: `map.yaml`

### 5.2 Restore Service & Set Initial Location
1. Restart robot navigation service in Web Terminal:
   ```bash
   sudo systemctl start yours_robot_start.service
   ```
2. In backend, under **Map View**, click **Initialize Position** -> pick robot (e.g. `A0088`) -> click robot's physical location on map -> **Save**.

### 5.3 Annotate Waypoints & Forbidden Zones
* **Task Points**: Click *Add Annotation* -> enter name -> pick location -> Save.
* **Charging Dock**: Click *Add Charge Point* -> pick dock coordinate -> Save.
* **Forbidden Zones**: Click *Add Forbidden Zone* -> draw polygon vertices -> Save.

---

## VI. Troubleshooting Quick Guide

| Problem | Cause | Solution |
| :--- | :--- | :--- |
| Pressing `i j k l` does not move robot | Service `yours_robot_start.service` still running | Run `sudo systemctl stop yours_robot_start.service` in Web Terminal first |
| SSH connection refused / timeout | Wi-Fi mismatch or wrong IP | Verify laptop & robot are on same Wi-Fi & IP `192.168.1.168` |
| Dual-layer save appears frozen | Dual-layer saving requires ~2x mapping duration | Wait patiently; do NOT close terminal or reboot |
| `python3 map_deal.py` file not found | Forgot to change directory | Run `cd /home/nvidia/nvidia_catkin_ws/tools/two_layer_map` first |

---
*— End of Operations Manual —*

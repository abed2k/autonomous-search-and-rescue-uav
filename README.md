# Autonomous Search and Rescue UAV

An autonomous indoor UAV exploration system built on **ROS 2 Jazzy** and **PX4 Autopilot**. The quadrotor operates in GPS-denied, hazardous environments (e.g., collapsed structures, smoke-filled buildings) using 3D LiDAR perception to incrementally construct an **OctoMap**, detect 3D frontiers, and plan collision-free navigation paths using the **BIT\*** (Batch Informed Trees) algorithm via **OMPL**.

> **Note on Naming:** While legacy code and parameter files inside `rrtstar_octomap_planner` reference `RRT*`, the underlying path planning logic is implemented using **BIT\*** for fast batch sampling, informed search, and asymptotic optimality.

---

## Team

This project was developed collaboratively by Abdul Karim (abed2k) and Hazem Abusalem (hazem-a17) under the supervision of **EMS Elektronik**.

---

## Table of Contents

- [Team](#-team)
- [Overview & Motivation](#overview--motivation)
- [System Architecture](#system-architecture)
- [Coordinate Frame Transformation (NED ↔ ENU)](#coordinate-frame-transformation-ned--enu)
- [Packages](#packages)
- [Hardware & Software Specifications](#hardware--software-specifications)
- [Installation & Build](#installation--build)
- [Terminal Workflow & Execution](#terminal-workflow--execution)
- [How It Works](#how-it-works)
  - [1. 3D Occupancy Mapping](#1-3d-occupancy-mapping)
  - [2. Frontier Extraction & Utility Ranking](#2-frontier-extraction--utility-ranking)
  - [3. BIT\* Path Planning](#3-bit-path-planning)
  - [4. Real-time Path Following & Safety Layer](#4-real-time-path-following--safety-layer)
- [ROS 2 Interface & Topics](#ros-2-interface--topics)
- [Estimated Hardware Budget](#estimated-hardware-budget)
- [Credits](#-credits)
- [License](#license)

---

## Overview & Motivation

Search-and-rescue (SAR) missions in GPS-denied, structurally compromised indoor spaces pose extreme dangers to human first responders. This project delivers a **fully autonomous UAV system** capable of:
- Autonomous takeoff, 3D exploration, mapping, obstacle avoidance, and landing without human intervention.
- Real-time 3D volumetric mapping using an onboard 3D LiDAR (Velodyne VLP-16 class sensor).
- Systematic exploration of unknown spaces using frontier extraction and utility scoring.
- Asymptotically optimal global path planning via BIT\* to navigate tight corridors and obstacles.
- Closed-loop safety checking against dynamic OctoMap updates to prevent collisions.

---

## System Architecture

```
                                  ┌───────────────────────────────┐
                                  │      PX4 Autopilot SITL       │
                                  │  (Offboard Trajectory Control)│
                                  └───────────────▲───────────────┘
                                                  │ /fmu/in/trajectory_setpoint
                                                  │ /fmu/in/offboard_control_mode
┌──────────────────────────────┐  /fmu/out/       │
│    Gazebo Harmonic / SITL    │  vehicle_odometry│
│  (X500 Quadcopter + 3D LiDAR)│─────────┐        │
└──────────────┬───────────────┘         │        │
               │ /lidar_3d/points        │        │
               ▼                         ▼        │
┌──────────────────────────────┐  ┌──────────────┴───────────────┐
│     Micro XRCE-DDS Agent     │  │      drone_tf_publisher      │
│  (PX4 ↔ ROS 2 Middleware)    │  │     (NED ↔ ENU Conversion)   │
└──────────────┬───────────────┘  └──────────────┬───────────────┘
               │                                 │ /tf (map ↔ base_link)
               ▼                                 │
┌──────────────────────────────┐                 │
│        OctoMap Server        │◄────────────────┘
│  (3D Probabilistic Voxel Grid)
└──────────────┬───────────────┘
               │ /octomap_full
               ▼
┌──────────────────────────────┐     /get_frontiers (SRV)
│     Frontier Extraction      │◄─────────────────────────┐
│   (3D Cluster Detection)     │                          │
└──────────────────────────────┘                          │
                                                          ▼
┌──────────────────────────────┐  /planned_path ┌───────────────────┐
│        Path Follower         │◄───────────────┤ Autonomous BIT*   │
│ (Segment Safety Check @ 20Hz)│                │      Planner      │
└──────────────────────────────┘                └───────────────────┘
```

---

## Coordinate Frame Transformation (NED ↔ ENU)

PX4 Autopilot operates natively in the **NED** (North-East-Down) coordinate frame, while ROS 2 uses **ENU** (East-North-Up).

The `drone_tf_publisher` node continuously handles rigid-body frame transformations:
- **Position**: $X_{\text{ENU}} = Y_{\text{NED}}$, $Y_{\text{ENU}} = X_{\text{NED}}$, $Z_{\text{ENU}} = -Z_{\text{NED}}$
- **Orientation**: Quaternion $[w, x, y, z]_{\text{ENU}} = [q_0, q_1, -q_2, -q_3]_{\text{NED}}$
- **TF Tree**: Broadcasts dynamic transform from `map` (parent) to `base_link` (child) and static transform from `base_link` to `lidar_3d`.

---

## Packages

| Package | Language | Function |
|---------|----------|----------|
| `rrtstar_octomap_planner` | C++ / OMPL | Implements the **BIT\*** path planner and the `autonomous_planner_node` that computes exploration goals and plans optimal paths using OctoMap collision queries. |
| `path_follower` | C++ | Low-latency control node executing planned paths by sending position setpoints to PX4 offboard mode while validating path segment safety against the live OctoMap at 20 Hz. |
| `drone_tf_publisher` | Python | Listens to PX4 odometry, converts NED to ENU, and broadcasts `/tf` frames for mapping and visualization. |
| `px4_teleop_tools` | Python | Manual teleoperation and structured waypoint flight scripts for testing and evaluation. |
| `px4_msgs` | C++ / ROS 2 | PX4 uORB message definitions bridged via Micro XRCE-DDS. |
| `robot_exploration` | C++ / Python | 3D frontier extraction service and exploration strategy management nodes. |

---

## Hardware & Software Specifications

### Software Stack
- **OS**: Ubuntu 24.04 LTS
- **Middleware**: ROS 2 Jazzy Jalisco
- **Autopilot**: PX4 Autopilot (v1.15+) with Offboard Control
- **Simulator**: Gazebo Harmonic (`gz_x500_walls`)
- **Bridge**: Micro XRCE-DDS Agent
- **Planning Library (OMPL)**: [Open Motion Planning Library (OMPL)](https://ompl.kavrakilab.org/) v1.5+ (installed via system package `libompl-dev`). Used by `rrtstar_octomap_planner` for 3D state space sampling (`ompl::base::RealVectorStateSpace`), custom OctoMap collision checking (`ompl::base::StateValidityChecker`), and optimal path generation via `ompl::geometric::BITstar`.
- **Frontier Exploration Framework**: Adapted from [ADVRHumanoids/robot_exploration](https://github.com/ADVRHumanoids/robot_exploration). Integrated into `src/robot_exploration` to provide 3D frontier extraction (`frontier_3d_extraction_node`), custom ROS 2 frontier services (`/get_frontiers`), and exploration management logic.
- **3D Mapping**: OctoMap 1.9+ (`octomap_server`)

### Robot & Sensor Model (Simulation)
- **Airframe**: X500 Quadcopter (`x500_lidar`)
- **Primary Range Sensor**: 3D LiDAR (16-channel Velodyne VLP-16 specification: 360° horizontal, 30° vertical FOV, 50m range)
- **Secondary Inspection Sensor**: Visual camera feed for situational awareness

---

## Installation & Build

```bash
# 1. Install ROS 2 dependencies & OMPL
sudo apt update && sudo apt install -y \
  ros-jazzy-octomap ros-jazzy-octomap-msgs ros-jazzy-octomap-server \
  ros-jazzy-tf2-ros ros-jazzy-nav-msgs ros-jazzy-geometry-msgs \
  ros-jazzy-sensor-msgs ros-jazzy-visualization-msgs \
  libompl-dev liboctomap-dev

# 2. Build the workspace
cd ~/autonomous-search-and-rescue-uav
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Terminal Workflow & Execution

Follow these step-by-step terminal steps to run the complete autonomous system:

### Terminal 1: Launch PX4 SITL & Gazebo World
```bash
cd ~/PX4-Autopilot
export PX4_SIM_HOST_ADDR=127.0.0.1
export PX4_MICRODDS_AGENT=udp://:8888
make px4_sitl gz_x500_walls
```

### Terminal 2: Start Micro XRCE-DDS Agent
```bash
micro-xrce-dds-agent udp4 -p 8888
```

### Terminal 3: Parameter Bridge for LiDAR & Clock
```bash
ros2 run ros_gz_bridge parameter_bridge \
  /clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock \
  /lidar_3d/points/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked
```

### Terminal 4: Launch Drone TF Publisher
```bash
source ~/autonomous-search-and-rescue-uav/install/setup.bash
ros2 launch drone_tf_publisher drone_tf_complete.launch.py
```

### Terminal 5: Launch OctoMap Server
```bash
source ~/autonomous-search-and-rescue-uav/install/setup.bash
ros2 run octomap_server octomap_server_node --ros-args \
  -r /cloud_in:=/lidar_3d/points/points \
  -p frame_id:=map \
  -p base_frame_id:=base_link \
  -p resolution:=0.15 \
  -p sensor_model.max_range:=50.0 \
  -p filter_ground_plane:=false
```

### Terminal 6: Launch Frontier Extraction
```bash
source ~/autonomous-search-and-rescue-uav/install/setup.bash
ros2 run frontier_extraction frontier_3d_extraction_node
```

### Terminal 7: Launch Path Follower (Safety Layer & Execution)
```bash
source ~/autonomous-search-and-rescue-uav/install/setup.bash
ros2 run path_follower path_follower
```

### Terminal 8: Launch Autonomous BIT\* Planner
```bash
source ~/autonomous-search-and-rescue-uav/install/setup.bash
ros2 run rrtstar_octomap_planner autonomous_planner_node --ros-args \
  -p autonomous_mode:=true
```

*(Optional)* Launch RViz2 in Terminal 9 to visualize `/octomap_full`, `/planned_path`, and `/tf`.

---

## How It Works

### 1. 3D Occupancy Mapping
`octomap_server` integrates 3D point clouds from the LiDAR into a 3D OcTree grid (voxel resolution 0.15m). Voxels are updated probabilistically as **Free**, **Occupied**, or **Unknown**.

### 2. Frontier Extraction & Utility Ranking
Frontiers are clusters of free voxels bordering unmapped unknown space. The `autonomous_planner_node` requests frontiers from `frontier_extraction` via `/get_frontiers` and ranks them using a utility score:

$$\text{Utility} = N_i \times D_i \times (1 + \text{ratio}_{\text{unknown}})$$

Where $N_i$ is the number of frontier points, $D_i$ is distance to the robot, and $\text{ratio}_{\text{unknown}}$ measures unknown volume within a 1.5m radius.

### 3. BIT\* Path Planning
The **Batch Informed Trees (BIT\*)** algorithm plans collision-free trajectories:
- **Batch Sampling**: Samples batch states within workspace bounds ($X \in [-9, 26]\text{m}$, $Y \in [-16, 17]\text{m}$, $Z \in [0.5, 5.5]\text{m}$).
- **Informed Search & Pruning**: Constrains search inside an ellipsoid defined by current solution cost, rapidly refining paths.
- **Safety Inflation**: Samples spherical clearance around voxels using $R_{\text{collision}} = 0.7\text{m} + \text{margin}_{\text{safety}} = 0.1\text{m}$.

### 4. Real-time Path Following & Safety Layer
The `path_follower` node streams `px4_msgs::msg::OffboardControlMode` and `px4_msgs::msg::TrajectorySetpoint` at 20 Hz. Before moving to the next waypoint, it performs a 15-step segment collision check against the live OctoMap, skipping or aborting unsafe paths dynamically.

---

## ROS 2 Interface & Topics

### Subscribed Topics
- `/fmu/out/vehicle_odometry` (`px4_msgs/msg/VehicleOdometry`) - PX4 odometry state
- `/octomap_full` (`octomap_msgs/msg/Octomap`) - 3D volumetric map
- `/planner_goal` (`geometry_msgs/msg/PointStamped`) - Manual target goals (when `autonomous_mode:=false`)

### Published Topics
- `/planned_path` (`nav_msgs/msg/Path`) - Computed 3D waypoints
- `/fmu/in/offboard_control_mode` (`px4_msgs/msg/OffboardControlMode`) - Offboard enable flags
- `/fmu/in/trajectory_setpoint` (`px4_msgs/msg/TrajectorySetpoint`) - Target position setpoints

---

## Estimated Hardware Budget

While evaluated in high-fidelity Gazebo simulation, the physical hardware Bill of Materials (BOM) for real-world deployment is estimated below:

| Component | Model / Spec | Est. Cost (USD) |
|-----------|--------------|-----------------|
| **Airframe Kit** | X500 V2 Carbon Fiber Frame Kit | $260.00 |
| **Propulsion** | 4x 2216 920KV Motors + 20A ESCs + 1045 Props | $148.00 |
| **Flight Controller** | Pixhawk 6C (PX4 compatible) | $180.00 |
| **Onboard Compute** | NVIDIA Jetson Orin Nano (ROS 2 / AI compute) | $300.00 |
| **3D Range Sensor** | Velodyne VLP-16 Puck LiDAR | $500.00 |
| **Telemetry & Radio** | SiK Telemetry Radio V3 + RadioMaster Receiver/Tx | $287.99 |
| **Power & Battery** | 2x 4S 5000mAh LiPo + PDB + Charger | $137.00 |
| **Inspection Camera** | GoPro Hero10 Black | $150.00 |
| **Total Hardware BOM** | | **~$2,046.99** |

---

## Credits

Special thanks and attribution to the following open-source libraries and projects:
- **[The Open Motion Planning Library (OMPL)](https://ompl.kavrakilab.org/)**: Used for 3D state space sampling and the BIT\* path planning algorithm implementation.
- **[robot_exploration](https://github.com/ADVRHumanoids/robot_exploration)**: Adapted for 3D frontier extraction and exploration management logic.

---

## License

This project is licensed under the [MIT License](LICENSE).
Submodules in `src/robot_exploration` retain their original licenses.
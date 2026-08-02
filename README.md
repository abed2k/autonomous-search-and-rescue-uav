# 🚁 Autonomous Search and Rescue UAV

An autonomous indoor UAV exploration system built on **ROS 2 Jazzy** and **PX4 Autopilot**. The drone uses a 3D LiDAR sensor to build an OctoMap of its environment, extracts frontiers from the map, and plans collision-free paths using the BIT\* (Batch Informed Trees) algorithm — enabling fully autonomous indoor exploration for search and rescue scenarios.

---

## 📋 Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Packages](#packages)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Building](#building)
- [Usage](#usage)
  - [Simulation Setup](#1-simulation-setup)
  - [Manual Navigation](#2-manual-navigation)
  - [Autonomous Exploration](#3-autonomous-exploration)
- [ROS 2 Topics](#ros-2-topics)
- [Configuration](#configuration)
- [How It Works](#how-it-works)
- [Results](#results)
- [License](#license)

---

## Overview

This project implements an end-to-end autonomous exploration pipeline for a PX4-based quadrotor (X500) equipped with a 3D LiDAR. The system is designed for GPS-denied indoor environments (e.g., collapsed buildings, warehouses) where a human operator cannot easily navigate.

**Key capabilities:**
- 🗺️ Real-time 3D occupancy mapping using OctoMap
- 🔍 Frontier-based exploration to systematically cover unknown space
- 🧭 Collision-free path planning with BIT\* (OMPL)
- 🛡️ Safety layer with octomap-based collision checking on every path segment
- 🎯 Autonomous goal selection with utility-based frontier ranking
- 🕹️ Manual teleoperation and waypoint navigation modes

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        PX4 SITL + Gazebo                        │
│  ┌──────────┐    ┌──────────┐    ┌──────────────────────────┐   │
│  │  X500     │    │  3D      │    │  PX4 Autopilot           │   │
│  │  Drone    │───▶│  LiDAR   │    │  (Offboard Control)      │   │
│  └──────────┘    └────┬─────┘    └──────────┬───────────────┘   │
└───────────────────────┼─────────────────────┼───────────────────┘
                        │                     │
              ┌─────────▼─────────┐   ┌───────▼────────┐
              │  Micro XRCE-DDS   │   │  Vehicle       │
              │  Bridge           │   │  Odometry      │
              └─────────┬─────────┘   └───────┬────────┘
                        │                     │
         ┌──────────────▼──────────────┐      │
         │  OctoMap Server             │      │
         │  (3D Occupancy Grid)        │      │
         └──────────────┬──────────────┘      │
                        │                     │
              ┌─────────▼─────────┐   ┌───────▼────────┐
              │  Frontier         │   │  Drone TF      │
              │  Extraction       │   │  Publisher      │
              └─────────┬─────────┘   │  (NED → ENU)   │
                        │             └───────┬────────┘
              ┌─────────▼─────────┐           │
              │  Exploration      │           │
              │  Manager          │           │
              └─────────┬─────────┘           │
                        │                     │
              ┌─────────▼─────────────────────▼────────┐
              │  Autonomous BIT* Planner               │
              │  (Goal Selection + Path Planning)      │
              └─────────┬─────────────────────────────┘
                        │
              ┌─────────▼─────────┐
              │  Path Follower    │
              │  (Safety Layer +  │
              │   Waypoint Track) │
              └───────────────────┘
```

---

## Packages

| Package | Language | Description |
|---------|----------|-------------|
| `rrtstar_octomap_planner` | C++ | BIT\* path planner with OctoMap collision checking. Contains both a manual planner node and an autonomous planner with frontier-based goal selection. |
| `path_follower` | C++ | Waypoint tracking node that follows planned paths, with an OctoMap-based safety layer that validates every path segment before execution. |
| `drone_tf_publisher` | Python | Publishes TF transforms from PX4 odometry (NED → ENU conversion) and static transforms for sensor frames. |
| `px4_teleop_tools` | Python | Manual teleoperation (velocity control) and waypoint navigation tools for PX4 drones via Micro XRCE-DDS. |
| `px4_msgs` | C++ | PX4 message definitions for ROS 2 communication. |
| `robot_exploration` | C++/Python | Third-party frontier extraction and exploration management framework (adapted from [robot_exploration](https://github.com/ADVRHumanoids/robot_exploration)). |

---

## Prerequisites

### Software

- **Ubuntu 24.04**
- **ROS 2 Jazzy** ([installation guide](https://docs.ros.org/en/jazzy/Installation.html))
- **PX4 Autopilot** (v1.15+) with SITL support
- **Gazebo Harmonic** (ships with ROS 2 Jazzy)
- **Micro XRCE-DDS Agent** for PX4 ↔ ROS 2 communication

### System Dependencies

```bash
# ROS 2 packages
sudo apt install ros-jazzy-octomap ros-jazzy-octomap-msgs ros-jazzy-octomap-server \
                 ros-jazzy-tf2-ros ros-jazzy-nav-msgs ros-jazzy-geometry-msgs \
                 ros-jazzy-sensor-msgs ros-jazzy-visualization-msgs

# OMPL (path planning library)
sudo apt install libompl-dev

# OctoMap development libraries
sudo apt install liboctomap-dev

# Micro XRCE-DDS Agent
sudo snap install micro-xrce-dds-agent --edge
```

---

## Installation

### 1. Clone the Repository

```bash
mkdir -p ~/uav_ws/src
cd ~/uav_ws/src
git clone https://github.com/<your-username>/autonomous-search-and-rescue-uav.git .
```

> **Note:** The `src/` directory of this repository is the ROS 2 workspace source folder. Clone accordingly.

### 2. Install ROS 2 Dependencies

```bash
cd ~/uav_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Set Up PX4 SITL

```bash
# Clone PX4 Autopilot (if not already installed)
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash Tools/setup/ubuntu.sh
```

---

## Building

```bash
cd ~/uav_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

> **Tip:** If you encounter OMPL-related build errors, ensure `libompl-dev` is installed and the include paths in `rrtstar_octomap_planner/CMakeLists.txt` match your system.

---

## Usage

### 1. Simulation Setup

**Terminal 1 — Start PX4 SITL with Gazebo:**

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500_lidar
```

**Terminal 2 — Start Micro XRCE-DDS Agent:**

```bash
micro-xrce-dds-agent udp4 -p 8888
```

**Terminal 3 — Launch OctoMap Server:**

```bash
source ~/uav_ws/install/setup.bash
ros2 launch octomap_server octomap_server.launch.py \
  remappings:="[('/cloud_in', '/lidar')]" \
  params:="[{'resolution': 0.15, 'frame_id': 'map'}]"
```

**Terminal 4 — Launch TF Publisher:**

```bash
source ~/uav_ws/install/setup.bash
ros2 launch drone_tf_publisher drone_tf_complete.launch.py
```

### 2. Manual Navigation

For teleoperation or manual waypoint navigation:

```bash
# Teleoperation with keyboard
ros2 run px4_teleop_tools microxrce_teleop

# OR waypoint navigation (predefined waypoints)
ros2 run px4_teleop_tools waypoint_navigator
```

For manual path planning (publish a goal, get a collision-free path):

```bash
# Start the planner
ros2 run rrtstar_octomap_planner planner_node

# Start the path follower
ros2 run path_follower path_follower

# Send a goal (ENU coordinates)
ros2 topic pub --once /planner_goal geometry_msgs/msg/PointStamped \
  "{header: {frame_id: 'map'}, point: {x: 10.0, y: 5.0, z: 2.0}}"
```

### 3. Autonomous Exploration

Launch the full autonomous exploration stack:

```bash
source ~/uav_ws/install/setup.bash
ros2 launch rrtstar_octomap_planner autonomous_exploration.launch.py
```

This launches:
- **Autonomous BIT\* Planner** — selects exploration goals from frontiers and plans paths
- **Frontier Extraction Node** — detects frontier clusters in the 3D OctoMap
- **Exploration Manager** — coordinates the exploration strategy

In a separate terminal, start the path follower:

```bash
ros2 run path_follower path_follower
```

The drone will autonomously:
1. Detect frontiers (boundaries between known-free and unknown space)
2. Rank frontiers by a utility function (size × distance × unknown ratio)
3. Plan a collision-free path to the best frontier using BIT\*
4. Follow the path while continuously checking for obstacles
5. Repeat until the environment is fully explored

---

## ROS 2 Topics

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` | Drone pose from PX4 (NED frame) |
| `/octomap_full` | `octomap_msgs/Octomap` | Full 3D occupancy map |
| `/planner_goal` | `geometry_msgs/PointStamped` | Manual goal input (ENU frame) |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/planned_path` | `nav_msgs/Path` | Collision-free path (ENU frame) |
| `/fmu/in/trajectory_setpoint` | `px4_msgs/TrajectorySetpoint` | Position commands to PX4 |
| `/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` | Offboard mode configuration |

### Service Clients

| Service | Type | Description |
|---------|------|-------------|
| `/get_frontiers` | `frontier_extraction_srvs/GetFrontiers` | Requests frontier clusters from the frontier extraction node |

---

## Configuration

Key parameters can be set via the launch file or command line:

### Planner Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `planning_time` | `2.0` | BIT\* solver timeout (seconds) |
| `collision_radius` | `0.8` | Drone collision sphere radius (meters) |
| `safety_margin` | `0.5` | Additional clearance around obstacles |
| `samples_per_batch` | `200` | BIT\* sampling density per batch |
| `max_attempts` | `5` | Maximum planning retries per cycle |
| `max_waypoints` | `25` | Maximum waypoints in output path |

### Exploration Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `autonomous_mode` | `true` | Enable/disable autonomous frontier exploration |
| `exploration_timeout` | `30.0` | Timeout before dropping an unreachable goal (seconds) |
| `min_frontier_utility` | `0.3` | Minimum utility score for a frontier to be selected |
| `frontier_update_interval` | `2.0` | How often to request new frontiers (seconds) |
| `visited_goal_radius` | `3.0` | Radius to mark a goal as already visited (meters) |

### Workspace Bounds

| Parameter | Default | Description |
|-----------|---------|-------------|
| `min_x` / `max_x` | `-50.0` / `50.0` | X-axis limits (meters) |
| `min_y` / `max_y` | `-50.0` / `50.0` | Y-axis limits (meters) |
| `min_z` / `max_z` | `0.5` / `10.0` | Z-axis (altitude) limits (meters) |

---

## How It Works

### 1. Mapping
The drone's 3D LiDAR produces a point cloud that is fed into an **OctoMap Server**, which maintains a probabilistic 3D occupancy grid. Each voxel is classified as free, occupied, or unknown.

### 2. Frontier Detection
The **Frontier Extraction** node scans the OctoMap for boundary voxels between known-free and unknown space. These frontier voxels are clustered into frontier regions, each with a centroid and point count.

### 3. Goal Selection
The **Autonomous Planner** evaluates each frontier using a utility function:

```
utility = num_points × distance_to_frontier × (1 + unknown_ratio)
```

- **`num_points`**: Larger frontiers are preferred (more area to explore)
- **`distance_to_frontier`**: Farther frontiers are preferred to avoid local oscillation
- **`unknown_ratio`**: Frontiers near more unknown space are preferred

Goals that are too close, already visited, or in occupied space are filtered out. If no valid frontier is found, a random collision-free goal within the exploration bounds is selected.

### 4. Path Planning
The **BIT\*** (Batch Informed Trees) planner from OMPL generates an asymptotically optimal, collision-free path through the OctoMap. The planner:
- Uses a 3D RealVectorStateSpace with configurable bounds
- Performs inflated collision checking (collision radius + safety margin)
- Validates the complete path post-planning
- Retries with increased sampling density if planning fails

### 5. Path Following
The **Path Follower** node tracks the planned path waypoint-by-waypoint:
- Converts ENU waypoints to NED setpoints for PX4
- Publishes `OffboardControlMode` and `TrajectorySetpoint` at 20 Hz
- Performs real-time safety checks on each path segment using the latest OctoMap
- Skips ahead or rejects unsafe path segments
- Holds position when no path is active

### 6. Continuous Replanning
The system runs in a continuous loop — the planner replans every 500ms with the latest odometry and map data, allowing the drone to adapt to newly discovered obstacles in real time.

---

## Results

The system has been tested in a Gazebo simulation environment featuring an indoor maze-like structure. Key results:

- ✅ **Full autonomous exploration** of a multi-room indoor environment with no human intervention
- ✅ **Real-time 3D mapping** with OctoMap at 0.15m resolution
- ✅ **Collision-free navigation** through narrow corridors and doorways using BIT\* with safety margins
- ✅ **Frontier-based coverage** systematically explores all reachable unknown areas
- ✅ **Dynamic replanning** adapts to newly mapped obstacles during flight
- ✅ **NED ↔ ENU frame handling** ensures consistent coordinate transforms between PX4 and ROS 2

### Performance Metrics (Simulation)

| Metric | Value |
|--------|-------|
| Planning frequency | ~2 Hz (500ms cycle) |
| Path follower rate | 20 Hz |
| OctoMap resolution | 0.15 m |
| Collision radius | 0.8 m |
| Goal tolerance | 0.8 m |
| Typical planning time | 1.5–3.0 s |

---

## License

This project is licensed under the [Apache 2.0 License](LICENSE).

The `robot_exploration` subpackages are adapted from [ADVRHumanoids/robot_exploration](https://github.com/ADVRHumanoids/robot_exploration) and retain their original license.

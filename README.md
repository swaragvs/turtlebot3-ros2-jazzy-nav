# Autonomous TurtleBot3 Navigation & Dynamic Obstacle Avoidance (ROS 2 Jazzy & Nav2)

![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy_Jalisco-informational?logo=ros)
![Simulation](https://img.shields.io/badge/Simulation-Gazebo_Harmonic-orange)
![Nav2](https://img.shields.io/badge/Navigation-Nav2-blue)
![Platform](https://img.shields.io/badge/Chassis-TurtleBot3_Waffle-success)

This repository contains the ROS 2 package, launch scripts, and configuration files for autonomous mobile robot navigation and real-time obstacle avoidance using the TurtleBot3 Waffle simulated in Gazebo Harmonic with the ROS 2 Navigation2 (Nav2) stack.

## 📽️ Video Demonstration

![TurtleBot3 Nav2 Demo](https://img.youtube.com/vi/YgTTjvmDnQk/0.jpg)

> 🔗 **Direct link:** [Watch on YouTube (1–2 min walkthrough)](https://youtu.be/YgTTjvmDnQk)

## 🛠️ System Architecture & Approach

```text
                 +--------------------------------+
                 |          Sensor Suite          |
                 |  (2D LiDAR /scan, IMU, Camera) |
                 +----------------+---------------+
                                  |
                                  v
 +----------------+    +-------------------+    +--------------------+
 |   Static Map   |    |   Local Costmap   |    |   Global Costmap   |
 |  (map_server)  |    |  (Obstacle Layer) |    |  (Inflation Layer) |
 +-------+--------+    +---------+---------+    +---------+----------+
         |                       |                        |
         v                       |                        v
 +----------------+              |              +--------------------+
 |  Localization  |              |              |   Global Planner   |
 |     (AMCL)     |              |              |   (NavFn / A*)     |
 +-------+--------+              |              +---------+----------+
         |                       |                        |
         +-----------+-----------+                        v
                     |                          +--------------------+
                     v                          |  Local Controller  |
          +-------------------+                 |       (DWB)        |
          |   TF Tree Chain   +---------------->+---------+----------+
          | map->odom->base   |                           |
          +-------------------+                           v
                                                +--------------------+
                                                |     Robot Base     |
                                                |     (/cmd_vel)     |
                                                +--------------------+
```

- **Localization:** AMCL uses an adaptive particle filter to match laser scans against the pre-built occupancy grid.
- **Costmap perception:** Layered 2D costmaps mark obstacles from laser returns, clear free space via ray-tracing, and pad obstacles with an inflation layer.
- **Local control:** The DWB controller scores candidate velocity commands in the dynamic window and discards trajectories that intersect inflated costmap obstacles.

## 💻 Environment

| Layer | Configuration |
|---|---|
| Host | HP EliteBook 840 G5, Intel Core i7-8650U, 24 GB RAM, Windows 11 Pro |
| Linux | Ubuntu 24.04 LTS on WSL2 with WSLg; replaced the previous Ubuntu 22.04 installation |
| Middleware | ROS 2 Jazzy Jalisco (`ros-jazzy-desktop`, Fast DDS) |
| Simulator | Gazebo Harmonic through `ros_gz`, with RViz2 |
| Navigation | Nav2 (`nav2_bringup` and `nav2_minimal_tb3_sim`) using the stock launch file; no custom planner was written |
| Robot | TurtleBot3 Waffle (`TURTLEBOT3_MODEL=waffle`) with LiDAR and camera links; costmap footprint follows the model |

## 📦 Installation & Setup

### 1. Install dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-turtlebot3* \
  ros-jazzy-ros-gz
```

### 2. Configure the environment

```bash
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
source ~/.bashrc
```

### 3. Clone and build

```bash
mkdir -p ~/antropi_ws/src
cd ~/antropi_ws/src
git clone https://github.com/swaragvs/turtlebot3-ros2-jazzy-nav.git antropi_tb3_nav

cd ~/antropi_ws
colcon build --symlink-install
source install/setup.bash
```

## 🚀 Usage

### 1. Launch the simulation

```bash
ros2 launch antropi_tb3_nav navigation.launch.py
```

This starts Gazebo Harmonic, RViz2, AMCL, and the Nav2 lifecycle server tree.

### 2. Send a navigation goal

1. In the RViz2 toolbar, select **Nav2 Goal**.
2. Click and drag on a free (white) area of the map to set the goal position and heading.
3. Watch the planned path in RViz2 and the robot navigate in Gazebo.

### 3. Test dynamic obstacle avoidance

While the robot is driving toward its goal, spawn an obstacle in its path:

```bash
ros2 run ros_gz_sim create \
  -name dynamic_block \
  -string '<sdf version="1.6"><model name="dynamic_block"><static>true</static><link name="link"><visual name="v"><geometry><box><size>0.35 0.35 0.4</size></box></geometry><material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material></visual><collision name="c"><geometry><box><size>0.35 0.35 0.4</size></box></geometry></collision></link></model></sdf>' \
  -x 0.0 -y -0.2 -z 0.2
```

## System Engineering Log
## Startup Sequence

During bringup, a dependent Nav2 node can remain inactive if the global costmap attempts to resolve the `map` frame before AMCL has established the initial pose. The launch configuration therefore supplies the initial pose at startup:

```text
initial_pose_x = -2.0
initial_pose_y = -0.5
initial_pose_yaw = 0.0
```

This seeds the localization state and gives the TF tree the required `map → odom → base_link` chain. Node activation is checked with:

```bash
ros2 lifecycle get /planner_server
ros2 lifecycle get /global_costmap/global_costmap
ros2 lifecycle get /controller_server
```

## Clock Synchronization

WSL2 wall-clock timing and Gazebo simulation time must be kept aligned. All Nav2 nodes are launched with simulated time enabled to prevent transform-extrapolation and dropped-message warnings during navigation:

```text
use_sim_time = True
```

## Localization Verification

The initial estimate was not assumed to be correct. TF and AMCL pose outputs were checked independently:

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo --once /amcl_pose
```

Agreement between the TF transform and AMCL estimate confirms that localization and the transform chain are operating consistently.

## 👤 Author

- **Name:** Swarag V S
- **Email:** swaragvs09@gmail.com
- **GitHub:** [@swaragvs](https://github.com/swaragvs)

## 📄 License

This project is licensed under the [Apache License 2.0](LICENSE).

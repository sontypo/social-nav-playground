# 📡 ROS2 Bridge Integration for Web Social Navigation Simulator

This module connects the interactive browser Social Navigation simulation with native **ROS2 Humble / Iron / Rolling** nodes and **RViz2** in real time via WebSocket.

---

## 🎯 Bidirectional ROS2 Topics

### 📤 Published Topics (Web Simulator $\rightarrow$ ROS2 / RViz2)

| Topic Name | Message Type | Description |
| :--- | :--- | :--- |
| `/robot_pose` | `geometry_msgs/msg/PoseStamped` | Real-time position $(x, y)$ and quaternion orientation of the AMR robot in frame `map`. |
| `/odom` | `nav_msgs/msg/Odometry` | Simulated odometry feed with linear/angular velocities, position, and covariance in frame `odom` $\rightarrow$ `base_link`. |
| `/tracked_humans` | `geometry_msgs/msg/PoseArray` | Array of all active human pedestrian coordinates and heading orientations in frame `map`. |
| `/scan` | `sensor_msgs/msg/LaserScan` | Configurable 2D LiDAR point cloud in frame `laser_link` ($18 - 720$ rays, $1.0 - 12.0\text{m}$ range, $120^\circ - 360^\circ$ FoV). |
| `/social_costmap` | `nav_msgs/msg/OccupancyGrid` | 2D dynamic Social Costmap ($0.2\text{m/cell}$, asymmetric Gaussian proxemics + obstacle inflation @ 5 Hz). |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | Current target navigation destination coordinates in frame `map` (Bidirectional sync). |

### 📥 Subscribed Topics (RViz2 / ROS2 $\rightarrow$ Web Simulator)

| Topic Name | Message Type | Description |
| :--- | :--- | :--- |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | **2D Goal Pose** tool in RViz2 / Nav2 — updates single goal or overwrites the currently active waypoint in multi-goal mode. |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | **Velocity Command** — incoming linear ($v_x$) and angular ($\omega_z$) velocities to control the robot from external ROS2 controllers. |
| `/clicked_point` | `geometry_msgs/msg/PointStamped` | **Publish Point** tool in RViz2 — moves the target goal flag. |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | **2D Pose Estimate** tool in RViz2 — repositions the AMR robot on canvas. |

*Note: All metric coordinates $(x, y)$ in ROS2 are automatically converted to and from Canvas pixel space ($1\text{m} = 40\text{px}$, centered origin).*

---

## 🚀 How to Run with ROS2 & RViz2

### Step 1: Install & Start `rosbridge_server`
```bash
# 1. Install rosbridge-suite (replace humble with your ROS2 distro)
sudo apt install ros-humble-rosbridge-server

# 2. Launch WebSocket Bridge (runs on port 9090 by default)
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### Step 2: Connect from Web Browser
1. Open the web studio: [http://localhost:5173/](http://localhost:5173/)
2. In the **ROS2 Bridge Panel** at the bottom, ensure `ws://localhost:9090` is set and click **Connect ROS2**.
3. The status badge will turn neon green: `ROS2 BRIDGE: CONNECTED`.

### Step 3: Launch RViz2 Visualizer & TF Broadcaster
In a new terminal:
```bash
# Run the pre-configured launcher:
./ros2_bridge/launch_visualizer.sh
```
Or manually:
```bash
# Terminal A: Run python TF Broadcaster & 3D Markers Node
python3 ros2_bridge/social_subscriber_example.py

# Terminal B: Launch RViz2 with pre-configured profile
rviz2 -d ros2_bridge/social_nav.rviz
```

### Step 4: Test 2D Goal Pose Synchronization
1. In RViz2, click the **2D Goal Pose** button in the top toolbar (or press `G`).
2. Click and drag anywhere on the RViz2 grid to set a target navigation goal.
3. The browser simulator will immediately sync the goal flag coordinates with a glowing animation!
4. Conversely, dragging the **GOAL flag** on the web canvas will update the 3D Goal Marker in RViz2.

---

## 📊 Public Crowd Trajectory Benchmark Datasets

This project includes genuine, full-scale trajectory data from the **EU Horizon 2020 CrowdBot (OpenTraj)**, **ETH Zurich**, and **University of Cyprus (UCY)** benchmarks located in `ros2_bridge/datasets/`:

| Dataset | Scene Description | Type / Modality | File Location |
| :--- | :--- | :--- | :--- |
| **ETH Univ** | ETH Zurich Main Building entrance with crossing diagonal streams | 8,908 frames (360 peds) | `ros2_bridge/datasets/eth_univ_standard.txt` |
| **ETH Hotel** | Hotel tram station & sidewalk with dense stopping & waiting behavior | 6,544 frames (390 peds) | `ros2_bridge/datasets/eth_hotel_standard.txt` |
| **UCY Zara-01** | Zara shopping street with dense bidirectional human traffic | 5,024 frames (148 peds) | `ros2_bridge/datasets/ucy_zara01_standard.txt` |
| **UCY Zara-02** | Denser shopping avenue with multi-directional group avoidance | 9,537 frames (204 peds) | `ros2_bridge/datasets/ucy_zara02_standard.txt` |
| **UCY Students** | University campus square with non-linear crowd convergence | 21,846 frames (428 peds) | `ros2_bridge/datasets/ucy_students_standard.txt` |
| **Stanford JRDB Quad** | JackRabbot robot navigating outdoor student plaza with conversational clusters & skateboarders | 3D LiDAR & 360° Vision | `ros2_bridge/datasets/jrdb_quad_real.txt` |
| **THÖR MoCap Lab** | Shared 2.2m laboratory corridor with human-robot reciprocal yielding (Univ. of Lincoln) | 3D Optical MoCap & LiDAR | `ros2_bridge/datasets/thor_corridor_real.txt` |
| **ATC Shopping Mall** | Asia Pacific Trade Center dense shopping crowds with window shoppers (ATR Kyoto) | Fixed 3D LiDAR Network | `ros2_bridge/datasets/atc_mall_real.txt` |
| **UT Austin SCAND** | Jackal & Spot quadruped robot navigation paths through campus plaza (UT Austin) | Mobile Robot Navigation | `ros2_bridge/datasets/scand_plaza_real.txt` |
| **SDD Coupa** | Stanford Drone Dataset Coupa Cafe multi-stream campus plaza (Stanford SVL) | 4K Drone Overhead | `ros2_bridge/datasets/sdd_coupa_sample.txt` |
| **inD Urban Intersection**| Shared space roundabout intersection with pedestrians and cyclists (RWTH Aachen) | Drone Metric Trajectories | `ros2_bridge/datasets/ind_intersection_real.txt` |

*Raw 8-column EWAP observation matrices are also available in `ros2_bridge/datasets/official_benchmarks/`.*
You can upload any of these files directly into the Web Simulator using the **`Upload Data`** button or select them from the scenario dropdown!

---

## 🏛️ Gazebo 11 & Ignition Gazebo 3D Simulation Suite

The system includes a full 3D physics-based simulation pipeline for running the exported playground scenes in **Gazebo Classic 11** and **Ignition Gazebo / Gazebo Sim**:

### 1. Nodes & Architecture:

| Component | Script / Node | ROS2 Topics Interfaced |
| :--- | :--- | :--- |
| **Gazebo World Exporter** | Web UI / `sim export gazebo` | Generates `social_nav_scene.world` / `.sdf` |
| **Human Pedestrian Controller** | `ros2_bridge/gazebo_human_controller.py` | Animates humans via SFM / datasets; publishes `/tracked_humans` (`geometry_msgs/PoseArray`) |
| **Robot Social Controller** | `ros2_bridge/social_robot_controller.py` | Subscribes to `/goal_pose`, `/odom`, `/scan`, `/tracked_humans`; publishes `/cmd_vel` (`geometry_msgs/Twist`) |
| **Master Simulator Launcher** | `ros2_bridge/launch_gazebo_sim.sh` | Launches Gazebo, human controller, robot controller, and RViz2 in one command |

### 2. How to Launch Full Gazebo Simulation:

```bash
# 1. Export the current playground from the Web Studio:
# Click "Gazebo (.world)" in the Tools Palette (or run 'sim export gazebo' in CLI)
# Move the downloaded social_nav_scene.world to your workspace.

# 2. Run the complete Gazebo + ROS2 Simulation:
./ros2_bridge/launch_gazebo_sim.sh social_nav_scene.world --rviz

# Or for Ignition Gazebo / Gazebo Sim:
./ros2_bridge/launch_gazebo_sim.sh social_nav_scene.sdf --rviz
```


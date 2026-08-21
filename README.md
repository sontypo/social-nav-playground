# 🤖 SocialNav Studio | Interactive Physics & Proxemics Simulation Engine

A dedicated, high-impact **Robotics Social Navigation Simulation Studio & Proxemics Lab Dashboard**. Focused 100% on the **Interactive Physics & Proxemics Engine (Live Social Navigation Playground)**, real-time telemetry analytics, ROS2 bridge streaming, and an enhanced interactive **Robotics CLI Terminal (`/dev/robot_cli`)**.

---

## ⚡ Features

1. **Interactive Social Navigation Simulator (Canvas Physics Engine)**:
   - Real-time multi-agent crowd navigation with dynamic pedestrians & AMR robot (Jackal platform).
   - Interactive **Hall's Proxemics** visualizer (Intimate, Personal, and Social zones with asymmetric Gaussian heatmaps).
   - Compare 6 Motion Planning Algorithms (Mostly mock for learning-based methods, will be updated):
     - **SFM (Social Force Model)** - Helbing anisotropic physical forces & right-hand passing bias.
     - **SARL (Relational Graph DRL)** - Spatio-temporal self-attention graph with 1.5s trajectory anticipation.
     - **CADRL (MIT DRL)** - Value-network reciprocal collision avoidance.
     - **Social MPC** - Receding horizon trajectory optimization with 12 waypoints.
     - **Social-ORCA (RVO)** - Geometric half-plane velocity obstacles.
     - **Non-Social A\*** - Baseline naive obstacle avoidance ignoring social zones.
    - Interactive Tools: **Drag / Select**, **Spawn Human**, **Place Pillar Obstacle**, **Set Goal**, **Move Robot**.
    - Public Benchmark Datasets:
      - **ETH Univ** (Diagonal Crossing Streams - ETH Zurich)
      - **ETH Hotel** (Tram Stop & Dense Waiting - ETH Zurich)
      - **UCY Zara-01 & Zara-02** (Two-Way Shopping Street - Univ. of Cyprus)
      - **UCY Students** (University Campus Plaza - Univ. of Cyprus)
      - **Stanford JRDB Quad & CS Atrium** (JackRabbot Mobile Robot - Stanford University)
      - **UT Austin SCAND Plaza** (Spot & Jackal AMR Navigation - UT Austin)
      - **THÖR MoCap Lab** (Shared Space Human-Robot Yielding - Univ. of Lincoln)
      - **ATC Shopping Center** (3D LiDAR High-Density Crowd - ATR Kyoto)
      - **Stanford Drone Dataset (SDD)** (Coupa Cafe Multi-Modal Flow - Stanford SVL)
      - **inD Urban Intersection** (LevelX Vehicle-Pedestrian Interaction - RWTH Aachen)
      - Plus custom `.txt` / `.csv` trajectory upload.
    - 360° 360-beam LiDAR raycasting ($1^\circ$ resolution) with real-time point cloud visualization.
    - Real-time Gauges: Social Compliance %, Personal Space Violations, Minimum Separation Distance, Comfort Index, Live Velocity, Metric Robot Pose $(x, y, \theta)$.
    - Dynamic Social Costmap Matrix layer ($100 \times 62$ grid @ $0.2\text{m}$ resolution) published live to ROS2 `/social_costmap`.

2. **Embedded Interactive Terminal CLI (`/dev/robot_cli`)**:
   - Full command-line control of the simulation loop (`sim algo <algo>`, `sim scenario <name>`, `sim peds <n>`, `sim speed <v>`, `sim courtesy <w>`, `sim pause`, `sim resume`, `sim reset`, `sim lidar <on|off>`, `sim heatmap <on|off>`, `sim status`).
   - ROS2 topic inspection (`ros2 topic list`, `ros2 topic echo <topic>`).
   - Mathematical theory & benchmark data inspector (`theory proxemics`, `theory sfm`, `theory sarl`, `theory benchmarks`, `theory datasets`).

3. **ROS2 Bridge Real-time Streaming**:
   - Live WebSocket bridge connector (`ws://localhost:9090`).
   - Real-time topics streaming: `/robot_pose`, `/tracked_humans`, `/scan`, `/goal_pose`, `/social_costmap`, `/cmd_vel`.

4. **Theory & Mathematical Formulations Modal**:
   - Edward T. Hall's Proxemics definitions & Asymmetric Gaussian potential formulas.
   - C++ and PyTorch implementation snippets for all algorithms.

---

## 🚀 How to Run Locally

### Method 1: Using Vite (Recommended)
```bash
# 1. Install dependencies
npm install

# 2. Start local development server
npm run dev
```
Open your browser at `http://localhost:5173`.

### Method 2: Python HTTP Server (Zero-install option)
```bash
# In the project directory:
python3 -m http.server 3000
```
Open your browser at `http://localhost:3000`.

---

## 🛠️ Tech Stack

- **Frontend**: HTML5, Vanilla CSS3 (Neon Glassmorphism & Cyber-Robotics HUD), Modern ES6+ JavaScript.
- **Simulation**: High-Performance 2D Canvas Physics Engine, 360° LiDAR Raycasting, Asymmetric Gaussian Proxemics.
- **Middleware**: ROS2 Humble / Iron / Rolling Bridge (WebSocket on port 9090).
- **Build Tool**: Vite.

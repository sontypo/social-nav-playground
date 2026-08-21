// Interactive ROS2 CLI Terminal & Simulation Controller Engine

import { simTheoryData } from './data.js';
import { ros2BridgeInstance } from './ros2Bridge.js';
import { telemetryAnalytics } from './analyticsCharts.js';
import { aiAssistantInstance } from './aiAssistant.js';
import { gazeboExporterInstance } from './gazeboExporter.js';

export function initInteractiveTerminal(getSimulatorInstance) {
  const termBody = document.getElementById('terminal-body');
  const termInput = document.getElementById('terminal-input');
  if (!termBody || !termInput) return;

  let history = [];
  let historyIndex = -1;

  function printOutput(htmlContent) {
    const outDiv = document.createElement('div');
    outDiv.className = 'term-output';
    outDiv.innerHTML = htmlContent;
    const inputLine = termInput.closest('.terminal-input-line');
    if (inputLine) {
      termBody.insertBefore(outDiv, inputLine);
    } else {
      termBody.appendChild(outDiv);
    }
    termBody.scrollTop = termBody.scrollHeight;
  }

  function handleCommand(rawCmd) {
    const sim = typeof getSimulatorInstance === 'function' ? getSimulatorInstance() : getSimulatorInstance;
    const parts = rawCmd.trim().split(/\s+/);
    const cmd = parts[0]?.toLowerCase();
    const sub = parts[1]?.toLowerCase();
    const arg = parts[2]?.toLowerCase();

    if (cmd === 'clear') {
      const outputs = termBody.querySelectorAll('.term-output');
      outputs.forEach(o => o.remove());
      return;
    }

    if (cmd === 'help') {
      printOutput(simTheoryData.terminalHelp);
      return;
    }

    if (cmd === 'sim') {
      if (!sim) {
        printOutput('<span class="term-coral">Error: Simulator instance not initialized.</span>');
        return;
      }

      if (!sub || sub === 'status') {
        const s = sim.getStatus();
        printOutput(`
<span class="term-cyan">=== [SIMULATION STATUS] ===</span>
  • Algorithm       : <span class="term-green">${s.algorithm.toUpperCase()}</span>
  • Scenario        : <span class="term-green">${s.scenario}</span>
  • Execution State : <span class="term-green">${s.isPaused ? 'PAUSED ⏸' : 'RUNNING ▶'}</span>
  • Crowd Density   : ${s.pedestrianCount} pedestrians
  • Max Speed       : ${s.robotSpeed} m/s (Current: ${s.currentVelocity} m/s)
  • Courtesy Weight : ${s.courtesyWeight}
  • Social Score    : <span class="term-green">${s.complianceScore}%</span> | Violations: ${s.violationsCount}
  • Min Separation  : <span class="term-cyan">${s.minDistanceToHuman}m</span> | Comfort: ${s.comfortIndex}%
  • Robot Metric Pos: (x: ${s.robotPose.x}m, y: ${s.robotPose.y}m, θ: ${s.robotPose.yawDeg}°)
        `);
        return;
      }

      if (sub === 'pause') {
        sim.pause();
        const pauseBtn = document.getElementById('btn-sim-pause');
        if (pauseBtn) pauseBtn.innerHTML = `<svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polygon points="5 3 19 12 5 21 5 3"></polygon></svg> Resume`;
        printOutput('<span class="term-green">Simulation execution paused.</span>');
        return;
      }

      if (sub === 'resume' || sub === 'play') {
        sim.resume();
        const pauseBtn = document.getElementById('btn-sim-pause');
        if (pauseBtn) pauseBtn.innerHTML = `<svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="6" y="4" width="4" height="16"></rect><rect x="14" y="4" width="4" height="16"></rect></svg> Pause`;
        printOutput('<span class="term-green">Simulation execution resumed.</span>');
        return;
      }

      if (sub === 'reset') {
        sim.reset();
        printOutput('<span class="term-green">Simulation world reset to initial coordinates.</span>');
        return;
      }

      if (sub === 'algo') {
        if (!arg) {
          printOutput('<span class="term-coral">Usage: sim algo &lt;sfm | sarl | cadrl | mpc | orca | nonsocial&gt;</span>');
          return;
        }
        let mappedAlgo = arg;
        if (arg === 'sarl' || arg === 'drl') mappedAlgo = 'drl';
        if (arg === 'mpc') mappedAlgo = 'social_mpc';
        if (arg === 'orca') mappedAlgo = 'orca_social';

        sim.setAlgorithm(mappedAlgo);
        
        // Sync UI tab buttons
        document.querySelectorAll('.algo-tab-btn').forEach(btn => {
          btn.classList.toggle('active', btn.getAttribute('data-algo') === mappedAlgo);
        });

        printOutput(`<span class="term-green">Planner algorithm switched to: <strong>${mappedAlgo.toUpperCase()}</strong></span>`);
        return;
      }

      if (sub === 'scenario') {
        if (!arg) {
          printOutput('<span class="term-coral">Usage: sim scenario &lt;eth_univ | eth_hotel | ucy_zara | jrdb_quad | jrdb_atrium | scand_plaza | thor_mocap | atc_mall | sdd_coupa | ind_urban | synthetic | bottleneck | doorway&gt;</span>');
          return;
        }
        sim.loadScenario(arg);
        const scenarioSelect = document.getElementById('dataset-scenario-select');
        if (scenarioSelect) scenarioSelect.value = arg;
        printOutput(`<span class="term-green">Benchmark scenario loaded: <strong>${arg}</strong></span>`);
        return;
      }

      if (sub === 'peds') {
        const count = parseInt(arg);
        if (isNaN(count)) {
          printOutput('<span class="term-coral">Usage: sim peds &lt;number (2-20)&gt;</span>');
          return;
        }
        sim.setPedestrianCount(count);
        const slider = document.getElementById('slider-pedestrians');
        const label = document.getElementById('val-pedestrians');
        if (slider) slider.value = count;
        if (label) label.textContent = count;
        printOutput(`<span class="term-green">Pedestrian crowd density set to: ${count}</span>`);
        return;
      }

      if (sub === 'speed') {
        const speed = parseFloat(arg);
        if (isNaN(speed)) {
          printOutput('<span class="term-coral">Usage: sim speed &lt;m/s (0.4 - 3.0)&gt;</span>');
          return;
        }
        sim.setRobotSpeed(speed);
        const slider = document.getElementById('slider-speed');
        const label = document.getElementById('val-speed');
        if (slider) slider.value = speed;
        if (label) label.textContent = `${speed.toFixed(1)} m/s`;
        printOutput(`<span class="term-green">Robot max velocity set to: ${speed.toFixed(1)} m/s</span>`);
        return;
      }

      if (sub === 'courtesy') {
        const c = parseFloat(arg);
        if (isNaN(c)) {
          printOutput('<span class="term-coral">Usage: sim courtesy &lt;weight (0.1 - 2.0)&gt;</span>');
          return;
        }
        sim.setCourtesyWeight(c);
        const slider = document.getElementById('slider-courtesy');
        const label = document.getElementById('val-courtesy');
        if (slider) slider.value = c;
        if (label) label.textContent = c.toFixed(1);
        printOutput(`<span class="term-green">Yielding courtesy weight set to: ${c.toFixed(1)}</span>`);
        return;
      }

      if (sub === 'spawn') {
        const x = parseFloat(parts[2]);
        const y = parseFloat(parts[3]);
        if (isNaN(x) || isNaN(y)) {
          sim.spawnPedestrian();
          printOutput('<span class="term-green">Spawned 1 pedestrian at random safe coordinates.</span>');
        } else {
          sim.spawnPedestrian(0, x, y);
          printOutput(`<span class="term-green">Spawned 1 pedestrian at canvas coordinates (${x}, ${y}).</span>`);
        }
        return;
      }

      if (sub === 'pillar') {
        const r = parseFloat(parts[2]);
        const x = parseFloat(parts[3]);
        const y = parseFloat(parts[4]);
        let obs;
        if (!isNaN(r) && isNaN(x)) {
          obs = sim.addCustomObject(null, null, { type: 'circle', radius: r });
          printOutput(`<span class="term-green">Placed cylinder pillar obstacle with radius R=${r}px.</span>`);
        } else if (!isNaN(x) && !isNaN(y)) {
          const radius = !isNaN(r) ? r : 22;
          obs = sim.addCustomObject(x, y, { type: 'circle', radius });
          printOutput(`<span class="term-green">Placed cylinder pillar (R=${radius}px) at (${x}, ${y}).</span>`);
        } else {
          obs = sim.addCustomObject(null, null, { type: 'circle' });
          printOutput('<span class="term-green">Placed static cylinder pillar obstacle.</span>');
        }
        return;
      }

      if (sub === 'box' || sub === 'rect') {
        const w = parseFloat(parts[2]);
        const h = parseFloat(parts[3]);
        const x = parseFloat(parts[4]);
        const y = parseFloat(parts[5]);
        let obs;
        if (!isNaN(w) && !isNaN(h) && !isNaN(x) && !isNaN(y)) {
          obs = sim.addCustomObject(x, y, { type: 'rect', width: w, height: h });
          printOutput(`<span class="term-green">Placed rectangular box (${w}x${h}px) at (${x}, ${y}).</span>`);
        } else if (!isNaN(w) && !isNaN(h)) {
          obs = sim.addCustomObject(null, null, { type: 'rect', width: w, height: h });
          printOutput(`<span class="term-green">Placed rectangular box obstacle (${w}x${h}px).</span>`);
        } else {
          obs = sim.addCustomObject(null, null, { type: 'rect', width: 60, height: 30 });
          printOutput('<span class="term-green">Placed static box barrier obstacle (60x30px).</span>');
        }
        return;
      }

      if (sub === 'poly' || sub === 'polygon') {
        const x = parseFloat(parts[2]);
        const y = parseFloat(parts[3]);
        let obs;
        if (isNaN(x) || isNaN(y)) {
          obs = sim.addRandomPolygonObstacle();
          printOutput(`<span class="term-green">Generated random geometric obstacle: <strong>${obs.label}</strong> (${obs.points.length} vertices).</span>`);
        } else {
          obs = sim.addRandomPolygonObstacle(x, y);
          printOutput(`<span class="term-green">Generated random geometric obstacle: <strong>${obs.label}</strong> at (${x}, ${y}).</span>`);
        }
        return;
      }

      if (sub === 'goal') {
        const x = parseFloat(parts[2]);
        const y = parseFloat(parts[3]);
        if (!isNaN(x) && !isNaN(y)) {
          sim.setGoalMode('single');
          sim.setGoal(x, y);
          printOutput(`<span class="term-green">Set single goal flag coordinates to (${x}, ${y}).</span>`);
        } else {
          printOutput(`<span class="term-cyan">Goal Mode: <strong>${sim.goalMode.toUpperCase()}</strong>. Active Target: (${sim.getActiveGoal().x.toFixed(0)}, ${sim.getActiveGoal().y.toFixed(0)}).</span>`);
        }
        return;
      }

      if (sub === 'wp' || sub === 'waypoint' || sub === 'waypoints') {
        const action = parts[2]?.toLowerCase();
        if (action === 'add') {
          const x = parseFloat(parts[3]);
          const y = parseFloat(parts[4]);
          if (!isNaN(x) && !isNaN(y)) {
            sim.setGoalMode('multi');
            sim.addWaypoint(x, y);
            printOutput(`<span class="term-green">Added Waypoint #${sim.waypoints.length} at (${x}, ${y}). Total: ${sim.waypoints.length} waypoints.</span>`);
          } else {
            printOutput('<span class="term-yellow">Usage: sim wp add &lt;x&gt; &lt;y&gt;</span>');
          }
        } else if (action === 'clear') {
          sim.clearWaypoints();
          printOutput('<span class="term-green">Cleared all waypoints from sequential patrol sequence.</span>');
        } else if (action === 'loop') {
          const state = parts[3]?.toLowerCase();
          sim.setWaypointLoop(state === 'on' || state === 'true');
          printOutput(`<span class="term-green">Waypoint Loop Patrol: <strong>${sim.waypointLoop ? 'ENABLED' : 'DISABLED'}</strong></span>`);
        } else {
          printOutput(`<span class="term-cyan">Multi-Waypoint Status: <strong>${sim.waypoints.length} points</strong> | Loop: <strong>${sim.waypointLoop ? 'ON' : 'OFF'}</strong> | Active Target: <strong>WP #${sim.activeWaypointIndex + 1}</strong></span>`);
        }
        return;
      }

      if (sub === 'clear') {
        if (arg === 'peds' || arg === 'humans') {
          sim.pedestrians = [];
          printOutput('<span class="term-green">Cleared all dynamic pedestrians from simulation.</span>');
        } else if (arg === 'obs' || arg === 'obstacles') {
          sim.clearStaticObstacles();
          printOutput('<span class="term-green">Cleared all static obstacles and polygons from simulation.</span>');
        } else {
          sim.pedestrians = [];
          sim.clearStaticObstacles();
          printOutput('<span class="term-green">Cleared all pedestrians and static obstacles from simulation.</span>');
        }
        return;
      }

      if (sub === 'lidar') {
        const parts = args.slice(1);
        if (parts[0] === 'rays' && parts[1]) {
          const r = sim.setLidarRays(parseInt(parts[1]));
          const slider = document.getElementById('slider-lidar-rays');
          const lbl = document.getElementById('val-lidar-rays');
          if (slider) slider.value = r;
          if (lbl) lbl.textContent = `${r} rays`;
          printOutput(`<span class="term-cyan">📡 LiDAR Beam Resolution updated: <strong>${r} rays</strong> (Angular step: ${(sim.lidarFovDeg / r).toFixed(2)}°)</span>`);
          return;
        }
        if (parts[0] === 'range' && parts[1]) {
          const rng = sim.setLidarMaxRange(parseFloat(parts[1]));
          const lbl = document.getElementById('val-lidar-range');
          if (lbl) lbl.textContent = `${rng.toFixed(1)} m`;
          printOutput(`<span class="term-cyan">📡 LiDAR Max Scan Range updated: <strong>${rng.toFixed(1)} meters</strong></span>`);
          return;
        }
        if (parts[0] === 'fov' && parts[1]) {
          const fov = sim.setLidarFov(parseInt(parts[1]));
          const lbl = document.getElementById('val-lidar-fov');
          if (lbl) lbl.textContent = `${fov}°`;
          printOutput(`<span class="term-cyan">📡 LiDAR Field of View (FoV) updated: <strong>${fov}°</strong></span>`);
          return;
        }
        if (parts[0] === 'beams') {
          const show = parts[1] === 'off' ? false : true;
          sim.toggleLidarRays(show);
          printOutput(`<span class="term-cyan">LiDAR Laser Rays Fan: <strong>${show ? 'ENABLED' : 'DISABLED'}</strong></span>`);
          return;
        }
        if (parts[0] === 'hits' || parts[0] === 'points') {
          const show = parts[1] === 'off' ? false : true;
          sim.toggleLidarPoints(show);
          printOutput(`<span class="term-cyan">LiDAR Return Point Cloud Hits: <strong>${show ? 'ENABLED' : 'DISABLED'}</strong></span>`);
          return;
        }
        if (arg === 'on') { sim.showLidar = true; }
        else if (arg === 'off') { sim.showLidar = false; }
        else { sim.toggleLidar(); }
        const btn = document.getElementById('btn-toggle-lidar');
        if (btn) btn.classList.toggle('active', sim.showLidar);
        printOutput(`<span class="term-green">LiDAR 360° Raycasting: <strong>${sim.showLidar ? 'ENABLED' : 'DISABLED'}</strong> (${sim.lidarRays || 360} rays, ${(sim.lidarMaxRangeM || 6.0).toFixed(1)}m range, ${sim.lidarFovDeg || 360}° FoV)</span>`);
        return;
      }

      if (sub === 'heatmap') {
        if (arg === 'on') { sim.showHeatmap = true; }
        else if (arg === 'off') { sim.showHeatmap = false; }
        else { sim.toggleHeatmap(); }
        const btn = document.getElementById('btn-toggle-heatmap');
        if (btn) btn.classList.toggle('active', sim.showHeatmap);
        printOutput(`<span class="term-green">Proxemics Gaussian Heatmap: <strong>${sim.showHeatmap ? 'ENABLED' : 'DISABLED'}</strong></span>`);
        return;
      }

      if (sub === 'export' || sub === 'gazebo' || sub === 'world' || sub === 'sdf') {
        const format = (arg === 'ign' || arg === 'ignition' || sub === 'sdf') ? 'ignition' : 'classic';
        const res = gazeboExporterInstance.downloadWorldFile(sim, format);
        const formatLabel = format === 'classic' ? 'Gazebo Classic 11 (.world)' : 'Ignition Gazebo / Gazebo Sim (.sdf)';
        printOutput(`
<span class="term-cyan">=== [GAZEBO SIMULATION EXPORT READY] ===</span>
  • Format: <span class="term-green">${formatLabel}</span>
  • Filename: <span class="term-amber">${res.filename}</span> (${(res.size / 1024).toFixed(1)} KB)
  • Includes: Arena Walls, Pillars, Box Barriers, Polygons, AMR Robot (Diff-Drive + LiDAR), Goals, and Pedestrian proxies.
  • Launch in terminal:
    <code>gazebo --verbose ${res.filename}</code> (Gazebo Classic)
    <code>ign gazebo ${res.filename}</code> (Ignition Gazebo)
        `);
        return;
      }

      printOutput(`<span class="term-coral">Unknown sim command: 'sim ${sub}'. Type 'help' for available commands.</span>`);
      return;
    }

    if (cmd === 'ros2') {
      if (sub === 'topic') {
        if (arg === 'list') {
          printOutput(`
<span class="term-cyan">=== [ACTIVE ROS2 TOPICS &amp; INTERFACES] ===</span>
  <span class="term-muted">PUBLISHED STREAMS (WebSocket @ 20 Hz):</span>
  • <span class="term-green">/robot_pose</span>         [geometry_msgs/msg/PoseStamped] (AMR 2D position &amp; orientation)
  • <span class="term-green">/odom</span>               [nav_msgs/msg/Odometry]         (Odometry pose &amp; twist velocities)
  • <span class="term-green">/tracked_humans</span>     [geometry_msgs/msg/PoseArray]   (Dynamic pedestrian array, N=${sim ? sim.pedestrians.length : 7})
  • <span class="term-green">/scan</span>               [sensor_msgs/msg/LaserScan]     (2D LiDAR: ${sim ? sim.lidarRays || 360 : 360} rays @ ${(sim ? sim.lidarMaxRangeM || 6.0 : 6.0).toFixed(1)}m, FoV: ${sim ? sim.lidarFovDeg || 360 : 360}°)
  • <span class="term-green">/goal_pose</span>          [geometry_msgs/msg/PoseStamped] (Target waypoint / Nav2 destination)
  • <span class="term-green">/social_costmap</span>     [nav_msgs/msg/OccupancyGrid]    (Hall's Proxemics costmap @ 5 Hz)

  <span class="term-muted">SUBSCRIBED COMMANDS (Incoming from Nav2 / RViz2):</span>
  • <span class="term-amber">/cmd_vel</span>            [geometry_msgs/msg/Twist]       (External velocity controller)
  • <span class="term-amber">/goal_pose</span>          [geometry_msgs/msg/PoseStamped] (2D Nav Goal input)
  • <span class="term-amber">/initialpose</span>        [geometry_msgs/msg/PoseWithCovarianceStamped] (2D Pose Estimate)
  • <span class="term-amber">/clicked_point</span>      [geometry_msgs/msg/PointStamped] (RViz2 Clicked Point)
          `);
          return;
        }

        const targetTopic = parts[3]?.toLowerCase() || arg;

        if (sub === 'topic' && (arg === 'echo' || parts[2] === 'echo')) {
          const topicName = parts[3] || parts[2];
          if (!topicName || topicName === 'echo') {
            printOutput('<span class="term-coral">Usage: ros2 topic echo &lt;/robot_pose | /odom | /tracked_humans | /scan | /goal_pose | /social_costmap | /cmd_vel&gt;</span>');
            return;
          }

          if (topicName.includes('robot_pose')) {
            const s = sim ? sim.getStatus() : { robotPose: { x: 0, y: 0, yawDeg: 0 } };
            const rad = s.robotPose.yawDeg * Math.PI / 180;
            printOutput(`
<span class="term-cyan">--- [TOPIC ECHO: /robot_pose] ---</span>
header:
  stamp: {sec: ${Math.floor(Date.now() / 1000)}, nanosec: ${(Date.now() % 1000) * 1000000}}
  frame_id: "map"
pose:
  position: {x: ${s.robotPose.x}, y: ${s.robotPose.y}, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: ${Math.sin(-rad / 2).toFixed(4)}, w: ${Math.cos(-rad / 2).toFixed(4)}}
            `);
            return;
          }

          if (topicName.includes('odom')) {
            const s = sim ? sim.getStatus() : { robotPose: { x: 0, y: 0, yawDeg: 0 } };
            const vx = sim ? +(sim.robot.vx / sim.scale).toFixed(3) : 0.0;
            const vy = sim ? +(-sim.robot.vy / sim.scale).toFixed(3) : 0.0;
            printOutput(`
<span class="term-cyan">--- [TOPIC ECHO: /odom] ---</span>
header:
  stamp: {sec: ${Math.floor(Date.now() / 1000)}, nanosec: ${(Date.now() % 1000) * 1000000}}
  frame_id: "odom"
child_frame_id: "base_link"
pose:
  pose:
    position: {x: ${s.robotPose.x}, y: ${s.robotPose.y}, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
twist:
  twist:
    linear: {x: ${vx}, y: ${vy}, z: 0.0}
    angular: {x: 0.0, y: 0.0, z: 0.0}
            `);
            return;
          }

          if (topicName.includes('tracked_humans') || topicName.includes('humans')) {
            const pCount = sim ? sim.pedestrians.length : 0;
            const samplePoses = sim ? sim.pedestrians.slice(0, 3).map((p, idx) => `  - position: {x: ${+((p.x - sim.canvas.width/2)/sim.scale).toFixed(2)}, y: ${+(-(p.y - sim.canvas.height/2)/sim.scale).toFixed(2)}, z: 0.0}`).join('\n') : '';
            printOutput(`
<span class="term-cyan">--- [TOPIC ECHO: /tracked_humans] ---</span>
header:
  stamp: {sec: ${Math.floor(Date.now() / 1000)}, nanosec: ${(Date.now() % 1000) * 1000000}}
  frame_id: "map"
poses: [Total ${pCount} tracked human proxies]
${samplePoses}
${pCount > 3 ? `  ... (${pCount - 3} more pedestrians)` : ''}
            `);
            return;
          }

          if (topicName.includes('goal_pose') || topicName.includes('goal')) {
            const g = sim ? (sim.goalMode === 'multi' && sim.waypoints.length > 0 ? sim.waypoints[sim.activeWaypointIndex] || sim.goal : sim.goal) : { x: 720, y: 260 };
            const gx = sim ? +((g.x - sim.canvas.width/2)/sim.scale).toFixed(2) : 7.20;
            const gy = sim ? +(-(g.y - sim.canvas.height/2)/sim.scale).toFixed(2) : 0.00;
            printOutput(`
<span class="term-cyan">--- [TOPIC ECHO: /goal_pose] ---</span>
header:
  stamp: {sec: ${Math.floor(Date.now() / 1000)}, nanosec: ${(Date.now() % 1000) * 1000000}}
  frame_id: "map"
pose:
  position: {x: ${gx}, y: ${gy}, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
            `);
            return;
          }

          if (topicName.includes('scan') || topicName.includes('lidar')) {
            const rays = sim ? sim.lidarRays || 360 : 360;
            const maxR = sim ? (sim.lidarMaxRangeM || 6.0).toFixed(1) : '6.0';
            const fov = sim ? sim.lidarFovDeg || 360 : 360;
            const fovRad = (fov * Math.PI / 180);
            printOutput(`
<span class="term-cyan">--- [TOPIC ECHO: /scan] ---</span>
header:
  stamp: {sec: ${Math.floor(Date.now() / 1000)}, nanosec: ${(Date.now() % 1000) * 1000000}}
  frame_id: "laser_link"
angle_min: ${+(-fovRad / 2).toFixed(5)}
angle_max: ${+(fovRad / 2).toFixed(5)}
angle_increment: ${+(fovRad / rays).toFixed(5)}
time_increment: 0.00000
scan_time: 0.05000
range_min: 0.10
range_max: ${maxR}
ranges: [${rays} real-time laser hit ranges streaming @ 20 Hz (Hits: ${sim ? sim.laserHits.length : 0})]
            `);
            return;
          }

          if (topicName.includes('costmap')) {
            const info = sim && sim.costmapInfo ? sim.costmapInfo : { width: 100, height: 62, resolution: 0.2 };
            printOutput(`
<span class="term-cyan">--- [TOPIC ECHO: /social_costmap] ---</span>
header:
  frame_id: "map"
info:
  resolution: ${info.resolution.toFixed(2)}
  width: ${info.width}
  height: ${info.height}
  origin: {position: {x: ${(-(info.width * info.resolution)/2).toFixed(2)}, y: ${(-(info.height * info.resolution)/2).toFixed(2)}, z: 0.0}}
data: [${info.width * info.height} cells of Hall's Proxemics Asymmetric Gaussian Costs]
            `);
            return;
          }

          if (topicName.includes('cmd_vel')) {
            printOutput(`
<span class="term-cyan">--- [TOPIC ECHO: /cmd_vel] ---</span>
linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
[Receiver listener active on WebSocket ws://localhost:9090]
            `);
            return;
          }

          printOutput(`Streaming mock frame on <span class="term-green">${topicName}</span> [OK]`);
          return;
        }
      }

      printOutput('<span class="term-coral">Usage: ros2 topic list | ros2 topic echo &lt;topic&gt;</span>');
      return;
    }

    if (cmd === 'theory') {
      if (!sub || sub === 'benchmarks') {
        let table = `
<span class="term-cyan">=== [BENCHMARK EVALUATION ON ETH / UCY DATASETS] ===</span>
Algorithm / Policy      | ADE ↓   | FDE ↓   | Compliance ↑ | Violations ↓ | Comfort ↑
----------------------------------------------------------------------------------
`;
        for (let b of simTheoryData.benchmarks) {
          table += `${b.name.padEnd(23)} | ${b.ade.padEnd(7)} | ${b.fde.padEnd(7)} | ${b.compliance.padEnd(12)} | ${b.violations.padEnd(12)} | ${b.comfort}\n`;
        }
        printOutput(`<pre style="font-family: inherit; font-size: 11px; margin: 0;">${table}</pre>`);
        return;
      }

      if (sub === 'proxemics') {
        let text = `<span class="term-cyan">=== [HALL'S PROXEMICS FORMULATION] ===</span><br>`;
        for (let z of simTheoryData.proxemicsTheory.zones) {
          text += `• <span style="color: ${z.color}; font-weight: bold;">${z.name}</span> (${z.radius}): ${z.description}<br>`;
        }
        text += `<br><span class="term-highlight">Gaussian Formulation:</span><br><code>${simTheoryData.proxemicsTheory.gaussianFormula}</code>`;
        printOutput(text);
        return;
      }

      if (sub === 'datasets') {
        let text = `<span class="term-cyan">=== [OFFICIAL PUBLIC CROWD & ROBOTICS BENCHMARK DATASETS] ===</span><br>`;
        if (simTheoryData.datasetInfo) {
          for (let d of simTheoryData.datasetInfo) {
            text += `• <span class="term-green">${d.name}</span> [<span class="term-cyan">${d.id}</span>] — <em>${d.venue}</em><br>`;
            text += `  <span class="term-muted">Type: ${d.type} | Env: ${d.env}</span><br>`;
            text += `  <span class="term-highlight">Key Features: ${d.features}</span><br><br>`;
          }
        }
        text += `To load any dataset in simulator: <code>sim scenario &lt;dataset_id&gt;</code>`;
        printOutput(text);
        return;
      }

      const algoObj = simTheoryData.algorithms.find(a => a.id === sub || a.id.includes(sub));
      if (algoObj) {
        let text = `
<span class="term-cyan">=== [ALGORITHM: ${algoObj.name}] ===</span>
Author / Venue: ${algoObj.author} (${algoObj.badge})
Description   : ${algoObj.description}

<span class="term-highlight">Equation:</span>
<code>${algoObj.equation}</code>

<span class="term-highlight">Key Properties:</span>
${algoObj.details.map(d => `• ${d}`).join('<br>')}
        `;
        printOutput(text);
        return;
      }

      printOutput('<span class="term-coral">Usage: theory &lt;sfm | sarl | cadrl | mpc | orca | proxemics | benchmarks | datasets&gt;</span>');
      return;
    }

    if (cmd === 'analytics') {
      if (!sub || sub === 'stats') {
        const statsComp = telemetryAnalytics.calcStats(telemetryAnalytics.history.compliance);
        const statsComf = telemetryAnalytics.calcStats(telemetryAnalytics.history.comfort);
        const statsDist = telemetryAnalytics.calcStats(telemetryAnalytics.history.minDistance);
        const statsViol = telemetryAnalytics.calcStats(telemetryAnalytics.history.violations);

        let table = `
<span class="term-cyan">=== [REAL-TIME TELEMETRY DATA ANALYTICS STATS] ===</span>
Metric Name              | Current  | Mean (μ) | [Min – Max]        | Std (σ)
-----------------------------------------------------------------------------
Social Compliance (%)    | ${(statsComp.cur.toFixed(1) + '%').padEnd(8)} | ${(statsComp.mean.toFixed(1) + '%').padEnd(8)} | [${statsComp.min.toFixed(1)}% – ${statsComp.max.toFixed(1)}%]`.padEnd(52) + `| ${statsComp.std.toFixed(2)}
Comfort Index (%)        | ${(statsComf.cur.toFixed(1) + '%').padEnd(8)} | ${(statsComf.mean.toFixed(1) + '%').padEnd(8)} | [${statsComf.min.toFixed(1)}% – ${statsComf.max.toFixed(1)}%]`.padEnd(52) + `| ${statsComf.std.toFixed(2)}
Min Distance to Human (m)| ${(statsDist.cur.toFixed(2) + 'm').padEnd(8)} | ${(statsDist.mean.toFixed(2) + 'm').padEnd(8)} | [${statsDist.min.toFixed(2)}m – ${statsDist.max.toFixed(2)}m]`.padEnd(52) + `| ${statsDist.std.toFixed(2)}
Personal Violations (ev) | ${(statsViol.cur.toString()).padEnd(8)} | ${(statsViol.mean.toFixed(1)).padEnd(8)} | [${statsViol.min} – ${statsViol.max}]`.padEnd(52) + `| ${statsViol.std.toFixed(2)}
-----------------------------------------------------------------------------
Samples Recorded: ${telemetryAnalytics.history.timestamps.length} / ${telemetryAnalytics.bufferSize} (${telemetryAnalytics.timeWindowSeconds}s rolling window)
Status: ${telemetryAnalytics.isPaused ? '<span class="term-coral">PAUSED</span>' : '<span class="term-green">LIVE STREAMING @ 12 Hz</span>'}
`;
        printOutput(`<pre style="font-family: inherit; font-size: 11px; margin: 0;">${table}</pre>`);
        return;
      }

      if (sub === 'pause') {
        telemetryAnalytics.isPaused = true;
        const btn = document.getElementById('btn-analytics-pause');
        if (btn) {
          btn.classList.add('active');
          btn.innerHTML = `<svg width="13" height="13" viewBox="0 0 24 24" fill="currentColor"><polygon points="5 3 19 12 5 21 5 3"></polygon></svg> Resume Data`;
        }
        printOutput('<span class="term-amber">Telemetry data logging paused.</span>');
        return;
      }

      if (sub === 'resume') {
        telemetryAnalytics.isPaused = false;
        const btn = document.getElementById('btn-analytics-pause');
        if (btn) {
          btn.classList.remove('active');
          btn.innerHTML = `<svg width="13" height="13" viewBox="0 0 24 24" fill="currentColor"><rect x="6" y="4" width="4" height="16"></rect><rect x="14" y="4" width="4" height="16"></rect></svg> Pause Data`;
        }
        printOutput('<span class="term-green">Telemetry data logging resumed.</span>');
        return;
      }

      if (sub === 'clear') {
        telemetryAnalytics.clearHistory();
        printOutput('<span class="term-green">Telemetry ring buffer cleared.</span>');
        return;
      }

      if (sub === 'export' || sub === 'csv') {
        telemetryAnalytics.exportCSV();
        printOutput('<span class="term-green">Exported telemetry run data to CSV.</span>');
        return;
      }

      if (sub === 'json') {
        telemetryAnalytics.exportJSON();
        printOutput('<span class="term-green">Exported telemetry run data to JSON.</span>');
        return;
      }

      printOutput('<span class="term-coral">Usage: analytics &lt;stats | pause | resume | clear | csv | json&gt;</span>');
      return;
    }

    if (cmd === 'theme') {
      if (!sub || sub === 'list') {
        printOutput(`
<span class="term-cyan">=== [STUDIO THEMES (${16} PRESETS)] ===</span>
  <span class="term-muted">🌙 CYBER &amp; SCI-FI DARK:</span>
  • <span class="term-magenta">dracula_vampire</span> : Dracula Pro (Midnight Violet &amp; Mint - Default)
  • <span class="term-green">obsidian</span>        : Obsidian Matrix (Cyberpunk Green &amp; Cyan)
  • <span class="term-amber">cyberpunk_neon</span>  : Cyberpunk 2077 (Electric Yellow &amp; Hyper Cyan)
  • <span class="term-magenta">tokyo</span>           : Tokyo Cyber (Synthwave Magenta &amp; Purple)
  • <span class="term-coral">synthwave_sunset</span>: Synthwave Sunset (Outrun Orange &amp; Fuchsia)
  • <span class="term-cyan">apollo</span>          : Apollo Cosmos (NASA Deep Space &amp; Gold)
  • <span class="term-green">emerald</span>         : Emerald CRT (Vintage Phosphor Terminal)
  • <span class="term-coral">crimson_void</span>    : Crimson Laser (Sith Abyss &amp; Ruby Fire)

  <span class="term-muted">📐 TECHNICAL &amp; MINIMALIST:</span>
  • <span class="term-cyan">nord_aurora</span>     : Nord Arctic Frost (Polar Slate &amp; Frost Cyan)
  • <span class="term-cyan">cobalt_blueprint</span>: CAD Blueprint (Architectural Cobalt &amp; Cyan)
  • <span class="term-muted">monochrome_minimal</span>: Stark Monochrome (High-Contrast Bauhaus White)

  <span class="term-muted">☀️ LIGHT &amp; WARM THEMES:</span>
  • <span class="term-cyan">light</span>           : Polar Laboratory (Clean Blue &amp; Slate White)
  • <span class="term-amber">solar_light</span>     : Solar Sand (Warm Amber &amp; Desert Cream)
  • <span class="term-amber">coffee_latte</span>    : Espresso Latte (Warm Coffee &amp; Caramel Cream)
  • <span class="term-coral">sakura_light</span>    : Neo Sakura (Lilac Mist &amp; Rose Crimson)
  • <span class="term-green">mint_light</span>      : Eco Mint (Nordic Fresh Emerald Frost)

Usage: <code>theme &lt;theme_id&gt;</code>
        `);
        return;
      }

      const validThemes = [
        'obsidian', 'cyberpunk_neon', 'tokyo', 'synthwave_sunset', 'apollo', 'emerald', 'crimson_void', 'dracula_vampire',
        'nord_aurora', 'cobalt_blueprint', 'monochrome_minimal',
        'light', 'solar_light', 'coffee_latte', 'sakura_light', 'mint_light'
      ];
      if (validThemes.includes(sub)) {
        if (typeof window.setStudioTheme === 'function') {
          window.setStudioTheme(sub, true);
          printOutput(`<span class="term-green">Studio theme successfully changed to: <strong>${sub.toUpperCase()}</strong></span>`);
        }
        return;
      }

      printOutput(`<span class="term-coral">Unknown theme: '${sub}'. Type 'theme list' for available themes.</span>`);
      return;
    }

    if (cmd === 'ai' || cmd === 'ask' || cmd === 'gemini' || cmd === 'chat') {
      const question = rawCmd.replace(/^(ai|ask|gemini|chat)\s*/i, '').trim();
      if (!question) {
        printOutput(`
<span class="term-cyan">=== [AI ROBOTICS ASSISTANT (ZERO-API / ON-DEVICE)] ===</span>
  • Ask questions about Social Navigation, Proxemics, ROS2, or analyze live state.
  • Usage examples:
    - <code>ai status</code> (Show active AI Core & privacy stats)
    - <code>ai analyze</code> (Real-time telemetry scene evaluation)
    - <code>ai explain hall's proxemics</code>
    - <code>ai compare sfm vs sarl</code>
    - <code>ai why is the robot slowing down?</code>
    - <code>ai how to connect rviz2?</code>
        `);
        return;
      }

      aiAssistantInstance.ask(question, sim).then(ans => {
        printOutput(ans);
      }).catch(err => {
        printOutput(`<span class="term-coral">AI Error: ${err.message}</span>`);
      });
      return;
    }

    printOutput(`<span class="term-coral">Command not found: '${rawCmd}'. Type 'help' to see available commands.</span>`);
  }

  termInput.addEventListener('keydown', (e) => {
    if (e.key === 'Enter') {
      const rawCmd = termInput.value.trim();
      termInput.value = '';

      if (!rawCmd) return;

      history.push(rawCmd);
      historyIndex = history.length;

      // Print prompt echo
      printOutput(`<span class="term-cyan">robot@socialnav:~$</span> ${rawCmd}`);
      handleCommand(rawCmd);
    } else if (e.key === 'ArrowUp') {
      e.preventDefault();
      if (history.length > 0 && historyIndex > 0) {
        historyIndex--;
        termInput.value = history[historyIndex];
      }
    } else if (e.key === 'ArrowDown') {
      e.preventDefault();
      if (historyIndex < history.length - 1) {
        historyIndex++;
        termInput.value = history[historyIndex];
      } else {
        historyIndex = history.length;
        termInput.value = '';
      }
    }
  });
}

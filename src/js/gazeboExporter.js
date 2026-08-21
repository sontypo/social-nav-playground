// Gazebo Classic 11 (.world) and Ignition Gazebo (.sdf) Exporter Engine
// Converts 2D Social Navigation playground state and dataset scenarios to 3D Gazebo SDF
// strictly containing the 3D world, obstacles, goals, and animated human <actor> models (NO robot embedded).

export class GazeboExporter {
  constructor() {}

  /**
   * Convert canvas coordinates to metric SI coordinates (m) matching ROS2 / Gazebo frame.
   */
  toMetricCoords(canvasX, canvasY, canvasW, canvasH, scale) {
    const x = (canvasX - canvasW / 2) / scale;
    const y = -(canvasY - canvasH / 2) / scale; // Gazebo ENU +Y is North / Top
    return { x: +x.toFixed(3), y: +y.toFixed(3) };
  }

  /**
   * Generate Animated Gazebo <actor> XML for a pedestrian based on dataset waypoints or SFM target.
   * Ensures physically realistic walking speeds (0.85 - 1.4 m/s) and coherent kinematic timestamp intervals.
   */
  generateActorXml(p, idx, canvasW, canvasH, scale, isIgnition = false) {
    const skinFile = isIgnition 
      ? 'https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/walk.dae'
      : 'walk.dae';
    const actorZ = isIgnition ? 1.0 : 0.0; // In Ignition Gazebo, actor origin is centered at torso, so +1.0m offset prevents sinking into ground plane

    // 1. Extract raw metric points list
    let metricPoints = [];
    const nominalSpeed = Math.min(1.4, Math.max(0.85, p.speed || 1.0)); // Natural human walking speed: 0.85 - 1.4 m/s

    if (p.rawMetricWaypoints && p.rawMetricWaypoints.length > 1) {
      metricPoints = p.rawMetricWaypoints.map(pt => ({
        x: +pt.x.toFixed(3),
        y: +pt.y.toFixed(3),
        time: pt.time
      }));
    } else if (p.customWaypoints && p.customWaypoints.length > 1) {
      metricPoints = p.customWaypoints.map(pt => this.toMetricCoords(pt.x, pt.y, canvasW, canvasH, scale));
    } else if (p.history && p.history.length > 5) {
      const step = Math.max(1, Math.floor(p.history.length / 24));
      const sampled = [];
      for (let i = 0; i < p.history.length; i += step) {
        sampled.push(p.history[i]);
      }
      if (sampled[sampled.length - 1] !== p.history[p.history.length - 1]) {
        sampled.push(p.history[p.history.length - 1]);
      }
      metricPoints = sampled.map(pt => this.toMetricCoords(pt.x, pt.y, canvasW, canvasH, scale));
    } else {
      const p0 = this.toMetricCoords(p.x, p.y, canvasW, canvasH, scale);
      const targetCanvasX = p.targetX !== undefined ? p.targetX : (canvasW - p.x);
      const targetCanvasY = p.targetY !== undefined ? p.targetY : (canvasH - p.y);
      const p1 = this.toMetricCoords(targetCanvasX, targetCanvasY, canvasW, canvasH, scale);
      metricPoints = [p0, p1];
    }

    // 2. Clean and deduplicate consecutive identical points (< 5cm)
    const cleanPts = [metricPoints[0]];
    for (let i = 1; i < metricPoints.length; i++) {
      const prev = cleanPts[cleanPts.length - 1];
      const cur = metricPoints[i];
      if (Math.hypot(cur.x - prev.x, cur.y - prev.y) > 0.05 || i === metricPoints.length - 1) {
        cleanPts.push(cur);
      }
    }

    let waypointsXml = '';

    // 3. Build paced, kinematic waypoints
    if (cleanPts.length <= 1) {
      const pt = cleanPts[0] || { x: 0, y: 0 };
      waypointsXml = `
          <waypoint>
            <time>0.000</time>
            <pose>${pt.x} ${pt.y} ${actorZ} 0 0 0.000</pose>
          </waypoint>
          <waypoint>
            <time>10.000</time>
            <pose>${pt.x} ${pt.y} ${actorZ} 0 0 0.000</pose>
          </waypoint>`;
    } else {
      let curTime = 0.0;
      for (let i = 0; i < cleanPts.length; i++) {
        const pt = cleanPts[i];
        let yaw = 0.0;

        if (i < cleanPts.length - 1) {
          yaw = +Math.atan2(cleanPts[i + 1].y - pt.y, cleanPts[i + 1].x - pt.x).toFixed(3);
        } else if (i > 0) {
          yaw = +Math.atan2(pt.y - cleanPts[i - 1].y, pt.x - cleanPts[i - 1].x).toFixed(3);
        }

        if (i > 0) {
          const prevPt = cleanPts[i - 1];
          const dist = Math.hypot(pt.x - prevPt.x, pt.y - prevPt.y);

          // If dataset provided valid timestamps and velocity is within realistic envelope (0.4 - 2.2 m/s)
          if (pt.time !== undefined && prevPt.time !== undefined && pt.time > prevPt.time) {
            const rawDt = pt.time - prevPt.time;
            const rawSpeed = dist / rawDt;
            if (rawSpeed >= 0.4 && rawSpeed <= 2.2) {
              curTime += rawDt;
            } else {
              curTime += Math.max(0.4, dist / nominalSpeed);
            }
          } else {
            // Paced by natural walking speed (dist / v)
            curTime += Math.max(0.4, dist / nominalSpeed);
          }
        }

        waypointsXml += `
          <waypoint>
            <time>${curTime.toFixed(3)}</time>
            <pose>${pt.x} ${pt.y} ${actorZ} 0 0 ${yaw}</pose>
          </waypoint>`;
      }

      // Smooth loop closure back to start position
      const startPt = cleanPts[0];
      const endPt = cleanPts[cleanPts.length - 1];
      const loopDist = Math.hypot(startPt.x - endPt.x, startPt.y - endPt.y);

      if (loopDist > 0.25) {
        const loopDt = Math.max(1.0, loopDist / nominalSpeed);
        const loopYaw = +Math.atan2(startPt.y - endPt.y, startPt.x - endPt.x).toFixed(3);
        const loopTime = +(curTime + loopDt).toFixed(3);

        waypointsXml += `
          <waypoint>
            <time>${loopTime.toFixed(3)}</time>
            <pose>${startPt.x} ${startPt.y} ${actorZ} 0 0 ${loopYaw}</pose>
          </waypoint>`;
      }
    }

    return `
    <!-- Animated Human Pedestrian Actor #${idx + 1} (${p.id || 'Agent_' + (idx + 1)}) -->
    <actor name="actor_pedestrian_${idx + 1}">
      <skin>
        <filename>${skinFile}</filename>
        <scale>1.0</scale>
      </skin>
      <animation name="walking">
        <filename>${skinFile}</filename>
        <scale>1.0</scale>
        <interpolate_x>true</interpolate_x>
      </animation>
      <script>
        <loop>true</loop>
        <delay_start>0.000000</delay_start>
        <auto_start>true</auto_start>
        <trajectory id="0" type="walking">${waypointsXml}
        </trajectory>
      </script>
    </actor>
`;
  }

  /**
   * Generate Gazebo Classic 11 (.world) SDF 1.6 / 1.7 XML content.
   * Only contains 3D world, obstacles, goals, and animated human <actor> models.
   */
  generateClassicWorld(sim) {
    const canvasW = sim.canvas.width;
    const canvasH = sim.canvas.height;
    const scale = sim.scale || 40;

    const arenaWidthM = +(canvasW / scale).toFixed(2);
    const arenaHeightM = +(canvasH / scale).toFixed(2);
    const halfWM = +(arenaWidthM / 2).toFixed(2);
    const halfHM = +(arenaHeightM / 2).toFixed(2);

    const activeGoal = sim.getActiveGoal();
    const goalPos = this.toMetricCoords(activeGoal.x, activeGoal.y, canvasW, canvasH, scale);

    let worldXml = '';

    // 1. Arena Perimeter Boundary Walls
    worldXml += `
    <!-- Arena Perimeter Bounding Walls (${arenaWidthM}m x ${arenaHeightM}m) -->
    <model name="arena_boundary_walls">
      <static>true</static>
      <link name="walls_link">
        <!-- North Wall -->
        <collision name="wall_north_col">
          <pose>0 ${halfHM} 0.75 0 0 0</pose>
          <geometry><box><size>${arenaWidthM + 0.3} 0.25 1.5</size></box></geometry>
        </collision>
        <visual name="wall_north_vis">
          <pose>0 ${halfHM} 0.75 0 0 0</pose>
          <geometry><box><size>${arenaWidthM + 0.3} 0.25 1.5</size></box></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material>
        </visual>

        <!-- South Wall -->
        <collision name="wall_south_col">
          <pose>0 -${halfHM} 0.75 0 0 0</pose>
          <geometry><box><size>${arenaWidthM + 0.3} 0.25 1.5</size></box></geometry>
        </collision>
        <visual name="wall_south_vis">
          <pose>0 -${halfHM} 0.75 0 0 0</pose>
          <geometry><box><size>${arenaWidthM + 0.3} 0.25 1.5</size></box></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material>
        </visual>

        <!-- East Wall -->
        <collision name="wall_east_col">
          <pose>${halfWM} 0 0.75 0 0 0</pose>
          <geometry><box><size>0.25 ${arenaHeightM + 0.3} 1.5</size></box></geometry>
        </collision>
        <visual name="wall_east_vis">
          <pose>${halfWM} 0 0.75 0 0 0</pose>
          <geometry><box><size>0.25 ${arenaHeightM + 0.3} 1.5</size></box></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material>
        </visual>

        <!-- West Wall -->
        <collision name="wall_west_col">
          <pose>-${halfWM} 0 0.75 0 0 0</pose>
          <geometry><box><size>0.25 ${arenaHeightM + 0.3} 1.5</size></box></geometry>
        </collision>
        <visual name="wall_west_vis">
          <pose>-${halfWM} 0 0.75 0 0 0</pose>
          <geometry><box><size>0.25 ${arenaHeightM + 0.3} 1.5</size></box></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material>
        </visual>
      </link>
    </model>
`;

    // 2. Static Obstacles (Pillars, Box Barriers, Polygons)
    sim.obstacles.forEach((obs, idx) => {
      const pos = this.toMetricCoords(obs.x, obs.y, canvasW, canvasH, scale);

      if (obs.type === 'circle') {
        const radiusM = +(obs.radius / scale).toFixed(3);
        worldXml += `
    <!-- Static Pillar Obstacle #${idx + 1} (${obs.label || 'Pillar'}) -->
    <model name="pillar_obs_${idx + 1}">
      <static>true</static>
      <pose>${pos.x} ${pos.y} 1.0 0 0 0</pose>
      <link name="link">
        <collision name="col">
          <geometry><cylinder><radius>${radiusM}</radius><length>2.0</length></cylinder></geometry>
        </collision>
        <visual name="vis">
          <geometry><cylinder><radius>${radiusM}</radius><length>2.0</length></cylinder></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/ZincYellow</name></script></material>
        </visual>
      </link>
    </model>
`;
      } else if (obs.type === 'rect') {
        const widthM = +(obs.width / scale).toFixed(3);
        const lengthM = +(obs.height / scale).toFixed(3);
        const centerPos = this.toMetricCoords(obs.x + obs.width / 2, obs.y + obs.height / 2, canvasW, canvasH, scale);
        worldXml += `
    <!-- Static Box Barrier #${idx + 1} (${obs.label || 'Box'}) -->
    <model name="box_barrier_${idx + 1}">
      <static>true</static>
      <pose>${centerPos.x} ${centerPos.y} 0.9 0 0 0</pose>
      <link name="link">
        <collision name="col">
          <geometry><box><size>${widthM} ${lengthM} 1.8</size></box></geometry>
        </collision>
        <visual name="vis">
          <geometry><box><size>${widthM} ${lengthM} 1.8</size></box></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Orange</name></script></material>
        </visual>
      </link>
    </model>
`;
      } else if (obs.type === 'polygon' && obs.points && obs.points.length > 2) {
        const pts = obs.points;
        const len = pts.length;
        let polySegmentsXml = '';

        for (let i = 0; i < len; i++) {
          const p1 = pts[i];
          const p2 = pts[(i + 1) % len];
          const segDxM = (p2.x - p1.x) / scale;
          const segDyM = -(p2.y - p1.y) / scale;
          const segLenM = +Math.hypot(segDxM, segDyM).toFixed(3);
          const segAng = +Math.atan2(segDyM, segDxM).toFixed(3);
          const midXM = +((p1.x + p2.x) / (2 * scale)).toFixed(3);
          const midYM = +-((p1.y + p2.y) / (2 * scale)).toFixed(3);

          polySegmentsXml += `
        <!-- Segment ${i + 1} -->
        <collision name="seg_col_${i}">
          <pose>${midXM} ${midYM} 0.9 0 0 ${segAng}</pose>
          <geometry><box><size>${segLenM} 0.22 1.8</size></box></geometry>
        </collision>
        <visual name="seg_vis_${i}">
          <pose>${midXM} ${midYM} 0.9 0 0 ${segAng}</pose>
          <geometry><box><size>${segLenM} 0.22 1.8</size></box></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Turquoise</name></script></material>
        </visual>`;
        }

        worldXml += `
    <!-- Procedural Polygon Obstacle #${idx + 1} (${obs.label || 'Polygon'}) -->
    <model name="polygon_obs_${idx + 1}">
      <static>true</static>
      <pose>${pos.x} ${pos.y} 0 0 0 0</pose>
      <link name="link">${polySegmentsXml}
      </link>
    </model>
`;
      }
    });

    // 3. Navigation Target Goals / Waypoints
    if (sim.goalMode === 'multi' && sim.waypoints.length > 0) {
      sim.waypoints.forEach((wp, idx) => {
        const wpPos = this.toMetricCoords(wp.x, wp.y, canvasW, canvasH, scale);
        const isActive = idx === sim.activeWaypointIndex;
        const mat = isActive ? 'Gazebo/RedBright' : 'Gazebo/Gold';
        worldXml += `
    <!-- Navigation Waypoint #${idx + 1} (${isActive ? 'ACTIVE TARGET' : 'PATROL STOP'}) -->
    <model name="waypoint_${idx + 1}">
      <static>true</static>
      <pose>${wpPos.x} ${wpPos.y} 0.05 0 0 0</pose>
      <link name="link">
        <visual name="beacon_pad">
          <geometry><cylinder><radius>0.45</radius><length>0.08</length></cylinder></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>${mat}</name></script></material>
        </visual>
        <visual name="beacon_pole">
          <pose>0 0 0.8 0 0 0</pose>
          <geometry><cylinder><radius>0.04</radius><length>1.6</length></cylinder></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/YellowGlow</name></script></material>
        </visual>
      </link>
    </model>
`;
      });
    } else {
      worldXml += `
    <!-- Navigation Target Goal Beacon -->
    <model name="nav_goal_target">
      <static>true</static>
      <pose>${goalPos.x} ${goalPos.y} 0.05 0 0 0</pose>
      <link name="link">
        <visual name="beacon_pad">
          <geometry><cylinder><radius>0.45</radius><length>0.08</length></cylinder></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Gold</name></script></material>
        </visual>
        <visual name="beacon_pole">
          <pose>0 0 0.8 0 0 0</pose>
          <geometry><cylinder><radius>0.04</radius><length>1.6</length></cylinder></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/YellowGlow</name></script></material>
        </visual>
      </link>
    </model>
`;
    }

    // 4. Animated Human Actors with Dataset / Playground Trajectories
    sim.pedestrians.forEach((p, idx) => {
      worldXml += this.generateActorXml(p, idx, canvasW, canvasH, scale, false);
    });

    // Complete SDF XML Document (Without Robot)
    return `<?xml version="1.0" ?>
<!-- ============================================================================== -->
<!-- Generated by Social Navigation Studio (3D World & Animated Human Actors)      -->
<!-- Target Engine: Gazebo Classic 11                                              -->
<!-- Scenario / Benchmark: ${sim.currentScenario || 'Custom Playground'}           -->
<!-- Coordinate System: SI Metric (Meters, Radians, Center-Origin ENU)             -->
<!-- ============================================================================== -->
<sdf version="1.6">
  <world name="social_nav_world">
    <!-- Directional Sun Light -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Concrete Ground Plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Standard ODE Physics Engine Setup -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Scene Lighting Configuration -->
    <scene>
      <ambient>0.4 0.4 0.4 1.0</ambient>
      <background>0.7 0.7 0.7 1.0</background>
      <shadows>true</shadows>
    </scene>
${worldXml}
  </world>
</sdf>
`;
  }

  /**
   * Generate Ignition Gazebo / Gazebo Sim (.sdf) SDF 1.8 / 1.9 XML content.
   * Only contains 3D world, obstacles, goals, and animated human <actor> models.
   */
  generateIgnitionWorld(sim) {
    const canvasW = sim.canvas.width;
    const canvasH = sim.canvas.height;
    const scale = sim.scale || 40;

    const arenaWidthM = +(canvasW / scale).toFixed(2);
    const arenaHeightM = +(canvasH / scale).toFixed(2);
    const halfWM = +(arenaWidthM / 2).toFixed(2);
    const halfHM = +(arenaHeightM / 2).toFixed(2);

    const activeGoal = sim.getActiveGoal();
    const goalPos = this.toMetricCoords(activeGoal.x, activeGoal.y, canvasW, canvasH, scale);

    let worldXml = '';

    // 1. Arena Perimeter Bounding Walls
    worldXml += `
    <!-- Arena Perimeter Bounding Walls (${arenaWidthM}m x ${arenaHeightM}m) -->
    <model name="arena_boundary_walls">
      <static>true</static>
      <link name="walls_link">
        <!-- North Wall -->
        <collision name="wall_north_col">
          <pose>0 ${halfHM} 0.75 0 0 0</pose>
          <geometry><box><size>${arenaWidthM + 0.3} 0.25 1.5</size></box></geometry>
        </collision>
        <visual name="wall_north_vis">
          <pose>0 ${halfHM} 0.75 0 0 0</pose>
          <geometry><box><size>${arenaWidthM + 0.3} 0.25 1.5</size></box></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material>
        </visual>

        <!-- South Wall -->
        <collision name="wall_south_col">
          <pose>0 -${halfHM} 0.75 0 0 0</pose>
          <geometry><box><size>${arenaWidthM + 0.3} 0.25 1.5</size></box></geometry>
        </collision>
        <visual name="wall_south_vis">
          <pose>0 -${halfHM} 0.75 0 0 0</pose>
          <geometry><box><size>${arenaWidthM + 0.3} 0.25 1.5</size></box></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material>
        </visual>

        <!-- East Wall -->
        <collision name="wall_east_col">
          <pose>${halfWM} 0 0.75 0 0 0</pose>
          <geometry><box><size>0.25 ${arenaHeightM + 0.3} 1.5</size></box></geometry>
        </collision>
        <visual name="wall_east_vis">
          <pose>${halfWM} 0 0.75 0 0 0</pose>
          <geometry><box><size>0.25 ${arenaHeightM + 0.3} 1.5</size></box></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material>
        </visual>

        <!-- West Wall -->
        <collision name="wall_west_col">
          <pose>-${halfWM} 0 0.75 0 0 0</pose>
          <geometry><box><size>0.25 ${arenaHeightM + 0.3} 1.5</size></box></geometry>
        </collision>
        <visual name="wall_west_vis">
          <pose>-${halfWM} 0 0.75 0 0 0</pose>
          <geometry><box><size>0.25 ${arenaHeightM + 0.3} 1.5</size></box></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material>
        </visual>
      </link>
    </model>
`;

    // 2. Static Obstacles (Pillars, Box Barriers, Polygons)
    sim.obstacles.forEach((obs, idx) => {
      const pos = this.toMetricCoords(obs.x, obs.y, canvasW, canvasH, scale);
      if (obs.type === 'circle') {
        const radiusM = +(obs.radius / scale).toFixed(3);
        worldXml += `
    <!-- Static Pillar Obstacle #${idx + 1} (${obs.label || 'Pillar'}) -->
    <model name="pillar_obs_${idx + 1}">
      <static>true</static>
      <pose>${pos.x} ${pos.y} 1.0 0 0 0</pose>
      <link name="link">
        <collision name="col"><geometry><cylinder><radius>${radiusM}</radius><length>2.0</length></cylinder></geometry></collision>
        <visual name="vis">
          <geometry><cylinder><radius>${radiusM}</radius><length>2.0</length></cylinder></geometry>
          <material>
            <ambient>0.9 0.7 0.1 1</ambient>
            <diffuse>0.9 0.7 0.1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>`;
      } else if (obs.type === 'rect') {
        const widthM = +(obs.width / scale).toFixed(3);
        const lengthM = +(obs.height / scale).toFixed(3);
        const centerPos = this.toMetricCoords(obs.x + obs.width / 2, obs.y + obs.height / 2, canvasW, canvasH, scale);
        worldXml += `
    <!-- Static Box Barrier #${idx + 1} (${obs.label || 'Box'}) -->
    <model name="box_barrier_${idx + 1}">
      <static>true</static>
      <pose>${centerPos.x} ${centerPos.y} 0.9 0 0 0</pose>
      <link name="link">
        <collision name="col"><geometry><box><size>${widthM} ${lengthM} 1.8</size></box></geometry></collision>
        <visual name="vis">
          <geometry><box><size>${widthM} ${lengthM} 1.8</size></box></geometry>
          <material>
            <ambient>0.9 0.4 0.1 1</ambient>
            <diffuse>0.9 0.4 0.1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>`;
      } else if (obs.type === 'polygon' && obs.points && obs.points.length > 2) {
        const pts = obs.points;
        const len = pts.length;
        let polySegmentsXml = '';

        for (let i = 0; i < len; i++) {
          const p1 = pts[i];
          const p2 = pts[(i + 1) % len];
          const segDxM = (p2.x - p1.x) / scale;
          const segDyM = -(p2.y - p1.y) / scale;
          const segLenM = +Math.hypot(segDxM, segDyM).toFixed(3);
          const segAng = +Math.atan2(segDyM, segDxM).toFixed(3);
          const midXM = +((p1.x + p2.x) / (2 * scale)).toFixed(3);
          const midYM = +-((p1.y + p2.y) / (2 * scale)).toFixed(3);

          polySegmentsXml += `
        <collision name="seg_col_${i}">
          <pose>${midXM} ${midYM} 0.9 0 0 ${segAng}</pose>
          <geometry><box><size>${segLenM} 0.22 1.8</size></box></geometry>
        </collision>
        <visual name="seg_vis_${i}">
          <pose>${midXM} ${midYM} 0.9 0 0 ${segAng}</pose>
          <geometry><box><size>${segLenM} 0.22 1.8</size></box></geometry>
          <material><ambient>0.0 0.8 0.8 1</ambient><diffuse>0.0 0.8 0.8 1</diffuse></material>
        </visual>`;
        }

        worldXml += `
    <!-- Procedural Polygon Obstacle #${idx + 1} (${obs.label || 'Polygon'}) -->
    <model name="polygon_obs_${idx + 1}">
      <static>true</static>
      <pose>${pos.x} ${pos.y} 0 0 0 0</pose>
      <link name="link">${polySegmentsXml}
      </link>
    </model>`;
      }
    });

    // 3. Navigation Target Goals / Sequential Waypoints
    if (sim.goalMode === 'multi' && sim.waypoints.length > 0) {
      sim.waypoints.forEach((wp, idx) => {
        const wpPos = this.toMetricCoords(wp.x, wp.y, canvasW, canvasH, scale);
        const isActive = idx === sim.activeWaypointIndex;
        const color = isActive ? '1 0 0.5 1' : '1 0.8 0 1';
        const emissive = isActive ? '0.8 0 0.4 1' : '0.6 0.4 0 1';
        worldXml += `
    <!-- Navigation Waypoint #${idx + 1} (${isActive ? 'ACTIVE TARGET' : 'PATROL STOP'}) -->
    <model name="waypoint_${idx + 1}">
      <static>true</static>
      <pose>${wpPos.x} ${wpPos.y} 0.05 0 0 0</pose>
      <link name="link">
        <visual name="beacon_pad">
          <geometry><cylinder><radius>0.45</radius><length>0.08</length></cylinder></geometry>
          <material>
            <ambient>${color}</ambient>
            <diffuse>${color}</diffuse>
            <emissive>${emissive}</emissive>
          </material>
        </visual>
        <visual name="beacon_pole">
          <pose>0 0 0.8 0 0 0</pose>
          <geometry><cylinder><radius>0.04</radius><length>1.6</length></cylinder></geometry>
          <material>
            <ambient>${color}</ambient>
            <diffuse>${color}</diffuse>
            <emissive>${emissive}</emissive>
          </material>
        </visual>
      </link>
    </model>
`;
      });
    } else {
      worldXml += `
    <!-- Navigation Target Beacon -->
    <model name="nav_goal_beacon">
      <static>true</static>
      <pose>${goalPos.x} ${goalPos.y} 0.05 0 0 0</pose>
      <link name="link">
        <visual name="beacon_pad">
          <geometry><cylinder><radius>0.45</radius><length>0.08</length></cylinder></geometry>
          <material>
            <ambient>1 0.8 0 1</ambient>
            <diffuse>1 0.8 0 1</diffuse>
            <emissive>0.6 0.4 0 1</emissive>
          </material>
        </visual>
        <visual name="beacon_pole">
          <pose>0 0 0.8 0 0 0</pose>
          <geometry><cylinder><radius>0.04</radius><length>1.6</length></cylinder></geometry>
          <material>
            <ambient>1 0.8 0 1</ambient>
            <diffuse>1 0.8 0 1</diffuse>
            <emissive>0.6 0.4 0 1</emissive>
          </material>
        </visual>
      </link>
    </model>
`;
    }

    // 4. Animated Human Actors with Dataset / Playground Trajectories (z = +1.0m offset)
    sim.pedestrians.forEach((p, idx) => {
      worldXml += this.generateActorXml(p, idx, canvasW, canvasH, scale, true);
    });

    return `<?xml version="1.0" ?>
<!-- ============================================================================== -->
<!-- Generated for Ignition Gazebo / Gazebo Sim (Fortress / Garden / Harmonic)       -->
<!-- Scenario / Benchmark: ${sim.currentScenario || 'Custom Playground'}           -->
<!-- Coordinate System: SI Metric (Meters, Radians, Center-Origin ENU)             -->
<!-- ============================================================================== -->
<sdf version="1.8">
  <world name="social_nav_ignition_world">
    <!-- Ignition System Plugins -->
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"></plugin>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"></plugin>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"></plugin>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <!-- Directional Sun Light -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground Plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="col"><geometry><plane><normal>0 0 1</normal><size>50 50</size></plane></geometry></collision>
        <visual name="vis"><geometry><plane><normal>0 0 1</normal><size>50 50</size></plane></geometry></visual>
      </link>
    </model>
${worldXml}
  </world>
</sdf>
`;
  }

  /**
   * Trigger browser file download for .world or .sdf file.
   */
  downloadWorldFile(sim, format = 'classic') {
    const isClassic = format === 'classic';
    const content = isClassic ? this.generateClassicWorld(sim) : this.generateIgnitionWorld(sim);
    const scenarioTag = (sim.currentScenario || 'custom').replace(/[^a-zA-Z0-9_]/g, '_');
    const filename = isClassic ? `social_nav_${scenarioTag}.world` : `social_nav_${scenarioTag}.sdf`;
    const mimeType = 'text/xml;charset=utf-8';

    const blob = new Blob([content], { type: mimeType });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);

    return { filename, size: content.length, scenario: sim.currentScenario || 'custom' };
  }
}

export const gazeboExporterInstance = new GazeboExporter();

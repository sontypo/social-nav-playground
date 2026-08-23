// SocialNav Studio — Live ROS2 Robot Stream & Hardware Visualizer Engine

import { ROS2Bridge } from './ros2Bridge.js';
import { decodePointCloud2, turboColormap, intensityColormap, OrbitCamera3D } from './pointcloudDecoder.js';
import { SSHManager } from './sshManager.js';

export class LiveStreamManager {
  constructor() {
    this.bridge = new ROS2Bridge({ isIngestMode: true, autoReconnect: false, wsUrl: 'ws://localhost:9091' });
    
    // Canvas & Viewport Transform
    this.canvas = null;
    this.ctx = null;
    this.canvasWidth = 900;
    this.canvasHeight = 650;
    this.scale = 35; // Pixels per meter (default 35px = 1m)
    this.cameraX = 0; // World meters at screen center
    this.cameraY = 0;
    this.autoCenter = true;
    this.isDragging = false;
    this.dragStartX = 0;
    this.dragStartY = 0;
    this.camStartX = 0;
    this.camStartY = 0;
    this.navGoalMode = false;
    this.navGoalDraft = null;

    // View Mode & 3D Orbit Camera
    this.viewMode = '2D'; // '2D' Top-Down or '3D' PointCloud Orbit
    this.orbitCamera = new OrbitCamera3D();
    this.isOrbiting = false;
    this.isPanning = false;

    // Visualizer Layer Toggles
    this.layers = {
      lidar: true,
      pointcloud: true,
      map: true,
      navPath: true,
      proxemics: true,
      trail: true,
      grid: true
    };

    // 3D PointCloud2 State & Decay Buffer
    this.pointCloud = {
      points: [], // Current scan [{x, y, z, intensity}]
      history: [], // [{x, y, z, intensity, time}]
      colormap: 'height', // 'height' | 'intensity' | 'cyan'
      decaySeconds: 1.0,
      pointSize: 2.5,
      hasData: false,
      frameMode: 'auto', // 'auto' | 'robot' | 'world' | 'optical'
      yawOffsetDeg: 0,   // 0, 90, 180, 270 (LiDAR mount orientation offset)
      flipZ: false,      // Invert Z for upside-down mounted LiDAR / Camera
      sensorHeight: 0.35 // Height of LiDAR on robot
    };

    // Standalone Mock Stream Generator (for zero-config testing)
    this.mockMode = false;
    this.mockTimer = null;
    this.mockAngle = 0;

    // Live Telemetry State from Real Robot / ROS2
    this.robot = {
      x: 0,
      y: 0,
      yaw: 0,
      linearV: 0,
      angularW: 0,
      lastStamp: 0,
      hasData: false
    };

    this.robotTrail = []; // [{x, y, time}]
    this.maxTrailPoints = 300;

    this.laserScan = {
      points: [], // [{x, y, range, intensity}]
      angleMin: -Math.PI,
      angleMax: Math.PI,
      rangeMax: 12.0,
      lastStamp: 0,
      hasData: false
    };

    this.occupancyMap = {
      width: 0,
      height: 0,
      resolution: 0.05,
      originX: 0,
      originY: 0,
      data: null,
      offscreenCanvas: null,
      hasData: false
    };

    this.navPaths = {
      global: [], // [{x, y}]
      local: []
    };

    this.trackedHumans = []; // [{id, x, y, vx, vy, yaw}]
    this.goalPose = null; // {x, y, yaw}

    // Static Environment Obstacles (Pillars, Benches, Planters, Center Kiosk)
    this.staticObstacles = {
      boxes: [
        { x: 2.2, y: 2.2, w: 0.8, h: 0.8, name: 'Pillar NE' },
        { x: -2.2, y: 2.2, w: 0.8, h: 0.8, name: 'Pillar NW' },
        { x: -2.2, y: -2.2, w: 0.8, h: 0.8, name: 'Pillar SW' },
        { x: 2.2, y: -2.2, w: 0.8, h: 0.8, name: 'Pillar SE' },
        { x: 0.0, y: 2.6, w: 1.6, h: 0.6, name: 'Bench North' },
        { x: 0.0, y: -2.6, w: 1.6, h: 0.6, name: 'Bench South' }
      ],
      circles: [
        { x: -3.6, y: 0.0, r: 0.45, name: 'Planter West' },
        { x: 3.6, y: 0.0, r: 0.45, name: 'Planter East' },
        { x: 0.0, y: 0.0, r: 0.55, name: 'Center Kiosk' }
      ],
      walls: [
        [[-5.5, -5.5], [5.5, -5.5]],
        [[5.5, -5.5], [5.5, 5.5]],
        [[5.5, 5.5], [-5.5, 5.5]],
        [[-5.5, 5.5], [-5.5, -5.5]]
      ]
    };

    this.battery = {
      percentage: 88,
      voltage: 24.2,
      current: -1.8,
      temperature: 32.5,
      hasData: false
    };

    this.imu = {
      accelX: 0.0,
      accelY: 0.0,
      accelZ: 9.81,
      gyroZ: 0.0,
      hasData: false
    };

    this.proxemicsStats = {
      intimateBreaches: 0,
      personalBreaches: 0,
      minHumanDistance: 99.0,
      comfortScore: 100
    };

    // Virtual Teleop Joystick
    this.teleop = {
      active: false,
      linearX: 0,
      angularZ: 0,
      maxLinearSpeed: 1.2, // m/s
      maxAngularSpeed: 1.8, // rad/s
      intervalTimer: null
    };

    // Active Topic Inspector selection
    this.inspectingTopic = null;

    this.animationFrameId = null;
  }

  init() {
    this.canvas = document.getElementById('live-hardware-canvas');
    if (this.canvas) {
      this.ctx = this.canvas.getContext('2d');
      this.resizeCanvas();
      window.addEventListener('resize', () => this.resizeCanvas());
      this.initCanvasInteractions();
    }

    try { this.initUI(); } catch (e) { console.warn('initUI error:', e); }
    try { this.initROS2Subscriptions(); } catch (e) { console.warn('initROS2Subscriptions error:', e); }
    try { this.setupAutoTopicDiscovery(); } catch (e) { console.warn('setupAutoTopicDiscovery error:', e); }
    try { this.initTopicRemapperUI(); } catch (e) { console.warn('initTopicRemapperUI error:', e); }
    try { this.initTeleopWASD(); } catch (e) { console.warn('initTeleopWASD error:', e); }
    try { this.initTopicInspector(); } catch (e) { console.warn('initTopicInspector error:', e); }

    // Initialize SSH Remote Robot Manager
    try {
      this.sshManager = new SSHManager(this);
      this.sshManager.initUI();
    } catch (e) {
      console.warn('SSHManager init error:', e);
    }

    // Populate endpoint input with default or stored URL (no auto-connect)
    const storedUrl = localStorage.getItem('socialnav_live_ws_url') || 'ws://localhost:9091';
    const inputUrl = document.getElementById('live-ws-endpoint-input');
    if (inputUrl) inputUrl.value = storedUrl;

    // Start 60 FPS Visualizer Render Loop
    this.renderLoop();
  }

  toggleMockMode() {
    this.mockMode = !this.mockMode;
    const mockBtns = [
      document.getElementById('btn-navbar-mock-stream'),
      document.getElementById('btn-banner-mock-stream')
    ];
    const banner = document.getElementById('live-no-robot-banner');

    mockBtns.forEach(btn => {
      if (!btn) return;
      btn.classList.toggle('active', this.mockMode);
      if (this.mockMode) {
        btn.innerHTML = '⏹ Stop Mock Stream';
      } else {
        btn.innerHTML = '▶ Demo Mock Stream';
      }
    });

    if (this.mockMode) {
      // Disconnect hardware ROS2 bridge if active to avoid conflicting packet streams
      if (this.bridge.isConnected) {
        this.bridge.disconnect();
      }
      if (banner) banner.style.display = 'none';
      this.startMockStream();
    } else {
      if (banner && !this.bridge.isConnected) banner.style.display = 'flex';
      this.stopMockStream();
    }
  }

  rayCircleIntersect(ox, oy, cosA, sinA, cx, cy, radius) {
    const fx = ox - cx;
    const fy = oy - cy;
    const b = 2 * (fx * cosA + fy * sinA);
    const c = (fx * fx + fy * fy) - radius * radius;
    const discriminant = b * b - 4 * c;
    if (discriminant < 0) return null;
    const sqrtD = Math.sqrt(discriminant);
    const t1 = (-b - sqrtD) / 2.0;
    const t2 = (-b + sqrtD) / 2.0;
    if (t1 > 0.05) return t1;
    if (t2 > 0.05) return t2;
    return null;
  }

  rayRectIntersect(ox, oy, cosA, sinA, rx, ry, rw, rh) {
    const bx1 = rx - rw / 2;
    const bx2 = rx + rw / 2;
    const by1 = ry - rh / 2;
    const by2 = ry + rh / 2;

    let tmin = 0.0;
    let tmax = 10000.0;

    if (Math.abs(cosA) > 0.0001) {
      let t1 = (bx1 - ox) / cosA;
      let t2 = (bx2 - ox) / cosA;
      if (t1 > t2) { const tmp = t1; t1 = t2; t2 = tmp; }
      tmin = Math.max(tmin, t1);
      tmax = Math.min(tmax, t2);
      if (tmin > tmax) return null;
    } else if (ox < bx1 || ox > bx2) {
      return null;
    }

    if (Math.abs(sinA) > 0.0001) {
      let t1 = (by1 - oy) / sinA;
      let t2 = (by2 - oy) / sinA;
      if (t1 > t2) { const tmp = t1; t1 = t2; t2 = tmp; }
      tmin = Math.max(tmin, t1);
      tmax = Math.min(tmax, t2);
      if (tmin > tmax) return null;
    } else if (oy < by1 || oy > by2) {
      return null;
    }

    return tmin > 0.05 ? tmin : null;
  }

  raySegmentIntersect(ox, oy, cosA, sinA, x1, y1, x2, y2) {
    const sx = x2 - x1;
    const sy = y2 - y1;
    const denom = cosA * sy - sinA * sx;
    if (Math.abs(denom) < 1e-8) return null;
    const t = ((x1 - ox) * sy - (y1 - oy) * sx) / denom;
    const u = ((x1 - ox) * sinA - (y1 - oy) * cosA) / denom;
    if (t > 0.05 && u >= 0 && u <= 1) {
      return t;
    }
    return null;
  }

  startMockStream() {
    if (this.mockTimer) clearInterval(this.mockTimer);
    
    // Initial mock human agents
    this.trackedHumans = [
      { id: 1, x: 2.8, y: 1.0, vx: -0.35, vy: 0.15, yaw: 3.14 },
      { id: 2, x: -1.8, y: 1.6, vx: 0.30, vy: -0.20, yaw: -0.5 },
      { id: 3, x: 1.2, y: -1.8, vx: -0.10, vy: 0.40, yaw: 1.8 },
      { id: 4, x: -1.2, y: -3.2, vx: 0.35, vy: 0.10, yaw: 0.2 }
    ];

    let cameraFrameCounter = 0;

    this.mockTimer = setInterval(() => {
      this.mockAngle += 0.03;
      const r = 2.4;
      const x = r * Math.cos(this.mockAngle);
      const y = r * Math.sin(this.mockAngle * 0.8);
      const yaw = this.mockAngle + Math.PI / 2;

      this.robot.x = +x.toFixed(3);
      this.robot.y = +y.toFixed(3);
      this.robot.yaw = yaw;
      this.robot.linearV = 0.85;
      this.robot.angularW = 0.35;
      this.robot.hasData = true;

      // Update trail
      this.robotTrail.push({ x: this.robot.x, y: this.robot.y, time: performance.now() });
      if (this.robotTrail.length > this.maxTrailPoints) this.robotTrail.shift();

      // Update mock humans
      for (const h of this.trackedHumans) {
        h.x += h.vx * 0.05;
        h.y += h.vy * 0.05;
        if (Math.abs(h.x) > 4.5) h.vx *= -1;
        if (Math.abs(h.y) > 4.5) h.vy *= -1;
        h.yaw = Math.atan2(h.vy, h.vx);
      }

      // Generate accurate 360-ray LiDAR matching Simulation Playground architecture
      const hits = [];
      const numRays = 180;
      const rx = this.robot.x;
      const ry = this.robot.y;
      const ryaw = this.robot.yaw;
      const maxRangeM = 6.0;

      for (let i = 0; i < numRays; i++) {
        const localAngleRos = -Math.PI + (i / numRays) * (Math.PI * 2);
        const rayAngle = ryaw + localAngleRos;
        const cosA = Math.cos(rayAngle);
        const sinA = Math.sin(rayAngle);

        let closestDist = maxRangeM;
        let hitType = 'max';

        // 1. Raycast Walls
        for (const [[wx1, wy1], [wx2, wy2]] of this.staticObstacles.walls) {
          const d = this.raySegmentIntersect(rx, ry, cosA, sinA, wx1, wy1, wx2, wy2);
          if (d !== null && d < closestDist) {
            closestDist = d;
            hitType = 'wall';
          }
        }

        // 2. Raycast Rectangular Boxes (Pillars & Benches)
        for (const b of this.staticObstacles.boxes) {
          const d = this.rayRectIntersect(rx, ry, cosA, sinA, b.x, b.y, b.w, b.h);
          if (d !== null && d < closestDist) {
            closestDist = d;
            hitType = 'pillar';
          }
        }

        // 3. Raycast Circular Planters & Center Kiosk
        for (const c of this.staticObstacles.circles) {
          const d = this.rayCircleIntersect(rx, ry, cosA, sinA, c.x, c.y, c.r);
          if (d !== null && d < closestDist) {
            closestDist = d;
            hitType = 'obstacle';
          }
        }

        // 4. Raycast Dynamic Humans
        for (const h of this.trackedHumans) {
          const d = this.rayCircleIntersect(rx, ry, cosA, sinA, h.x, h.y, 0.35);
          if (d !== null && d < closestDist) {
            closestDist = d;
            hitType = 'human';
          }
        }

        hits.push({
          x: rx + closestDist * cosA,
          y: ry + closestDist * sinA,
          dist: closestDist,
          type: hitType
        });
      }
      this.laserScan.hits = hits;
      this.laserScan.points = hits.filter(h => h.type !== 'max');
      this.laserScan.hasData = true;

      // Generate Synthetic 3D Pointcloud (16-beam Velodyne VLP-16 style)
      const cloudPts = [];
      const numRings = 16;
      const numAzimuths = 72;
      const rSensorZ = 0.35; // Sensor height on Jackal AMR
      const nowMs = performance.now();

      for (let ring = 0; ring < numRings; ring++) {
        // Vertical angle: -15 deg to +15 deg
        const vertAngle = (-15.0 + (ring / (numRings - 1)) * 30.0) * (Math.PI / 180.0);
        const cosV = Math.cos(vertAngle);
        const sinV = Math.sin(vertAngle);

        for (let az = 0; az < numAzimuths; az++) {
          const azAngle = ryaw + -Math.PI + (az / numAzimuths) * (Math.PI * 2);
          const cosA = Math.cos(azAngle);
          const sinA = Math.sin(azAngle);

          let closest2D = maxRangeM;
          let hitHeight = 1.0;
          let hitIntensity = 40.0;

          // 1. Raycast Walls
          for (const [[wx1, wy1], [wx2, wy2]] of this.staticObstacles.walls) {
            const d = this.raySegmentIntersect(rx, ry, cosA, sinA, wx1, wy1, wx2, wy2);
            if (d !== null && d < closest2D) {
              closest2D = d;
              hitHeight = 2.4;
              hitIntensity = 75.0;
            }
          }

          // 2. Raycast Boxes (Pillars & Benches)
          for (const b of this.staticObstacles.boxes) {
            const d = this.rayRectIntersect(rx, ry, cosA, sinA, b.x, b.y, b.w, b.h);
            if (d !== null && d < closest2D) {
              closest2D = d;
              hitHeight = b.name.includes('Pillar') ? 2.0 : 0.45;
              hitIntensity = b.name.includes('Pillar') ? 85.0 : 60.0;
            }
          }

          // 3. Raycast Circles (Planters & Kiosk)
          for (const c of this.staticObstacles.circles) {
            const d = this.rayCircleIntersect(rx, ry, cosA, sinA, c.x, c.y, c.r);
            if (d !== null && d < closest2D) {
              closest2D = d;
              hitHeight = 0.8;
              hitIntensity = 65.0;
            }
          }

          // 4. Raycast Humans
          for (const h of this.trackedHumans) {
            const d = this.rayCircleIntersect(rx, ry, cosA, sinA, h.x, h.y, 0.35);
            if (d !== null && d < closest2D) {
              closest2D = d;
              hitHeight = 1.75;
              hitIntensity = 95.0;
            }
          }

          // 3D Ground Hit (Z = 0)
          if (sinV < -0.01) {
            const groundDist = rSensorZ / -sinV;
            const ground2D = groundDist * cosV;
            if (ground2D < closest2D && ground2D < maxRangeM) {
              const px = rx + ground2D * cosA;
              const py = ry + ground2D * sinA;
              cloudPts.push({ x: +px.toFixed(3), y: +py.toFixed(3), z: 0.0, intensity: 25.0 });
              continue;
            }
          }

          if (closest2D < maxRangeM - 0.1) {
            const hitZ = rSensorZ + closest2D * (sinV / (cosV || 1e-4));
            if (hitZ >= 0.0 && hitZ <= hitHeight) {
              const px = rx + closest2D * cosA;
              const py = ry + closest2D * sinA;
              cloudPts.push({ x: +px.toFixed(3), y: +py.toFixed(3), z: +hitZ.toFixed(3), intensity: hitIntensity });
            }
          }
        }
      }

      this.pointCloud.points = cloudPts;
      this.pointCloud.hasData = true;
      for (let i = 0; i < cloudPts.length; i++) {
        this.pointCloud.history.push({
          x: cloudPts[i].x,
          y: cloudPts[i].y,
          z: cloudPts[i].z,
          intensity: cloudPts[i].intensity,
          time: nowMs
        });
      }
      const maxHistory = 100000;
      if (this.pointCloud.history.length > maxHistory) {
        this.pointCloud.history.splice(0, this.pointCloud.history.length - maxHistory);
      }

      // Battery & IMU fluctuation
      this.battery.percentage = Math.max(10, +(88 - (this.mockAngle * 0.1)).toFixed(1));
      this.battery.voltage = +(24.2 - (this.mockAngle * 0.01)).toFixed(1);
      this.battery.hasData = true;

      this.imu.accelX = +(Math.cos(this.mockAngle) * 0.4).toFixed(2);
      this.imu.accelY = +(Math.sin(this.mockAngle) * 0.4).toFixed(2);
      this.imu.gyroZ = 0.35;
      this.imu.hasData = true;

      // Record simulated stats in bridge inspector
      this.bridge.recordTopicStat('/odom', { pose: { position: { x: this.robot.x, y: this.robot.y } } });
      this.bridge.recordTopicStat('/scan', { ranges_count: this.laserScan.hits.length });
      this.bridge.recordTopicStat('/points', { point_count: this.pointCloud.points.length });
      this.bridge.recordTopicStat('/battery_state', { percentage: this.battery.percentage });
      this.bridge.recordTopicStat('/imu/data', { accel: this.imu.accelX });
      this.bridge.recordTopicStat('/tracked_humans', { count: this.trackedHumans.length });
      this.bridge.recordTopicStat('/camera/image_raw/compressed', { format: 'jpeg', fps: 15 });

      this.updateKinematicsUI();
      this.updateBatteryUI();
      this.evaluateRealWorldProxemics();

      // Render Mock Camera Frame at ~12.5 FPS
      cameraFrameCounter++;
      if (cameraFrameCounter % 2 === 0) {
        this.renderMockCameraFrame();
      }
    }, 50); // 20 Hz
  }

  renderMockCameraFrame() {
    const imgEl = document.getElementById('live-camera-img');
    const noSignal = document.getElementById('camera-no-signal-box');
    if (!imgEl) return;

    if (!this.mockCamCanvas) {
      this.mockCamCanvas = document.createElement('canvas');
      this.mockCamCanvas.width = 384;
      this.mockCamCanvas.height = 216;
      this.mockCamCtx = this.mockCamCanvas.getContext('2d');
    }

    const ctx = this.mockCamCtx;
    const w = this.mockCamCanvas.width;
    const h = this.mockCamCanvas.height;

    // 1. Sky & Ground
    const horizonY = Math.floor(h * 0.48);
    ctx.fillStyle = '#0a1017';
    ctx.fillRect(0, 0, w, horizonY);
    ctx.fillStyle = '#121e1a';
    ctx.fillRect(0, horizonY, w, h - horizonY);

    // 2. Perspective Ground Grid
    ctx.strokeStyle = 'rgba(0, 229, 255, 0.18)';
    ctx.lineWidth = 1;
    for (let gx = -10; gx <= 10; gx += 2) {
      const cxVanish = w / 2;
      const xBottom = cxVanish + gx * 45 - this.robot.yaw * 30;
      ctx.beginPath();
      ctx.moveTo(cxVanish, horizonY);
      ctx.lineTo(xBottom, h);
      ctx.stroke();
    }

    for (const depthStep of [1.2, 2.0, 3.2, 5.0, 8.0]) {
      const yPlane = Math.floor(horizonY + (h - horizonY) / depthStep);
      ctx.beginPath();
      ctx.moveTo(0, yPlane);
      ctx.lineTo(w, yPlane);
      ctx.stroke();
    }

    // 3. Project 3D Static Pillars
    const rx = this.robot.x;
    const ry = this.robot.y;
    const ryaw = this.robot.yaw;
    const fov = 1.2;
    const fLen = w / (2.0 * Math.tan(fov / 2.0));

    for (const b of this.staticObstacles.boxes) {
      const dx = b.x - rx;
      const dy = b.y - ry;
      const xRel = dx * Math.cos(ryaw) + dy * Math.sin(ryaw);
      const yRel = -dx * Math.sin(ryaw) + dy * Math.cos(ryaw);

      if (xRel > 0.4) {
        const u = Math.floor(w / 2 - (yRel / xRel) * fLen);
        const wPx = Math.floor((b.w / xRel) * fLen);
        const hPx = Math.floor((1.8 / xRel) * fLen);
        const vBottom = Math.floor(horizonY + (0.9 / xRel) * fLen);
        const vTop = vBottom - hPx;

        if (u > -100 && u < w + 100) {
          ctx.fillStyle = 'rgba(40, 60, 55, 0.9)';
          ctx.strokeStyle = 'rgba(0, 229, 255, 0.7)';
          ctx.lineWidth = 1.5;
          ctx.fillRect(u - wPx / 2, Math.max(0, vTop), wPx, Math.min(h, vBottom) - Math.max(0, vTop));
          ctx.strokeRect(u - wPx / 2, Math.max(0, vTop), wPx, Math.min(h, vBottom) - Math.max(0, vTop));

          ctx.fillStyle = '#00e5ff';
          ctx.font = '9px JetBrains Mono, monospace';
          ctx.textAlign = 'center';
          ctx.fillText(b.name, u, Math.max(15, vTop - 4));
        }
      }
    }

    // 4. Project 3D Humans with AI Bounding Boxes
    for (const human of this.trackedHumans) {
      const dx = human.x - rx;
      const dy = human.y - ry;
      const xRel = dx * Math.cos(ryaw) + dy * Math.sin(ryaw);
      const yRel = -dx * Math.sin(ryaw) + dy * Math.cos(ryaw);
      const dist = Math.hypot(xRel, yRel);

      if (xRel > 0.4) {
        const u = Math.floor(w / 2 - (yRel / xRel) * fLen);
        const wPx = Math.floor((0.55 / xRel) * fLen);
        const hPx = Math.floor((1.75 / xRel) * fLen);
        const vBottom = Math.floor(horizonY + (0.9 / xRel) * fLen);
        const vTop = vBottom - hPx;

        if (u >= 0 && u <= w) {
          // Silhouette
          ctx.fillStyle = '#00f0ff';
          ctx.beginPath();
          ctx.arc(u, vTop + hPx / 5, wPx / 3, 0, Math.PI * 2);
          ctx.fill();
          ctx.fillRect(u - wPx / 3, vTop + hPx / 3, (wPx * 2) / 3, vBottom - (vTop + hPx / 3));

          // Green AI Bounding Box
          ctx.strokeStyle = '#00ff9d';
          ctx.lineWidth = 1.5;
          ctx.strokeRect(u - wPx / 2, Math.max(0, vTop), wPx, Math.min(h, vBottom) - Math.max(0, vTop));

          ctx.fillStyle = '#00ff9d';
          ctx.font = '9px JetBrains Mono, monospace';
          ctx.textAlign = 'left';
          ctx.fillText(`HUMAN #${human.id} | ${dist.toFixed(1)}m`, Math.max(5, u - wPx / 2), Math.max(12, vTop - 4));
        }
      }
    }

    // 5. Cyber HUD Crosshairs & Telemetry Bar
    const cx = w / 2;
    const cy = h / 2;
    ctx.strokeStyle = 'rgba(0, 229, 255, 0.7)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(cx - 15, cy); ctx.lineTo(cx - 4, cy);
    ctx.moveTo(cx + 4, cy); ctx.lineTo(cx + 15, cy);
    ctx.moveTo(cx, cy - 15); ctx.lineTo(cx, cy - 4);
    ctx.moveTo(cx, cy + 4); ctx.lineTo(cx, cy + 15);
    ctx.arc(cx, cy, 18, 0, Math.PI * 2);
    ctx.stroke();

    // Top HUD Bar
    ctx.fillStyle = 'rgba(4, 8, 12, 0.85)';
    ctx.fillRect(0, 0, w, 18);
    ctx.fillStyle = '#00e5ff';
    ctx.font = '9px JetBrains Mono, monospace';
    ctx.textAlign = 'left';
    ctx.fillText(`CAM: FPV-FRONT | ROS2 /camera/image_raw/compressed | 15 FPS`, 8, 12);

    // Bottom Telemetry Bar
    ctx.fillRect(0, h - 20, w, 20);
    ctx.fillStyle = '#00ff9d';
    ctx.fillText(`SPEED: ${this.robot.linearV.toFixed(2)} m/s | YAW: ${(this.robot.yaw * 180 / Math.PI).toFixed(0)}° | BATT: ${Math.round(this.battery.percentage)}%`, 8, h - 6);

    imgEl.src = this.mockCamCanvas.toDataURL('image/jpeg', 0.8);
    imgEl.style.display = 'block';
    if (noSignal) noSignal.style.display = 'none';
  }

  stopMockStream() {
    if (this.mockTimer) {
      clearInterval(this.mockTimer);
      this.mockTimer = null;
    }
    const imgEl = document.getElementById('live-camera-img');
    const noSignal = document.getElementById('camera-no-signal-box');
    if (imgEl) imgEl.style.display = 'none';
    if (noSignal) noSignal.style.display = 'flex';
  }

  resizeCanvas() {
    if (!this.canvas) return;
    const rect = this.canvas.parentElement ? this.canvas.parentElement.getBoundingClientRect() : null;
    const width = rect && rect.width > 0 ? rect.width : (this.canvas.clientWidth || 900);
    const height = rect && rect.height > 0 ? rect.height : (this.canvas.clientHeight || 650);

    const dpr = window.devicePixelRatio || 1;
    this.canvas.width = width * dpr;
    this.canvas.height = height * dpr;
    if (this.ctx) {
      this.ctx.setTransform(1, 0, 0, 1, 0, 0);
      this.ctx.scale(dpr, dpr);
    }
    this.canvasWidth = width;
    this.canvasHeight = height;
  }

  connectBridge(url) {
    this.bridge.connect(url);
    localStorage.setItem('socialnav_live_ws_url', url);
  }

  initUI() {
    // 1. Connection Controls (Toolbar & Navbar)
    const btnConnect = document.getElementById('btn-live-ws-connect');
    const btnNavConnect = document.getElementById('btn-navbar-live-connect');
    const navConnectText = document.getElementById('navbar-connect-text');
    const inputUrl = document.getElementById('live-ws-endpoint-input');
    
    const handleToggleConnect = () => {
      if (this.bridge.isConnected) {
        this.bridge.disconnect();
      } else {
        // Automatically stop in-browser mock stream if active to prevent state/topic conflicts
        if (this.mockMode) {
          this.toggleMockMode();
        }
        const url = inputUrl?.value.trim() || 'ws://localhost:9091';
        this.connectBridge(url);
      }
    };

    btnConnect?.addEventListener('click', handleToggleConnect);
    btnNavConnect?.addEventListener('click', handleToggleConnect);

    inputUrl?.addEventListener('keydown', (e) => {
      if (e.key === 'Enter') {
        e.preventDefault();
        handleToggleConnect();
      }
    });

    // Bind Demo Mock Stream trigger buttons
    const mockTriggers = [
      document.getElementById('btn-navbar-mock-stream'),
      document.getElementById('btn-banner-mock-stream')
    ];
    mockTriggers.forEach(btn => btn?.addEventListener('click', () => this.toggleMockMode()));

    this.bridge.onStatusChange = (status, text) => {
      const pill = document.getElementById('live-bridge-status-pill');
      const textSpan = document.getElementById('live-bridge-status-text');
      const banner = document.getElementById('live-no-robot-banner');

      if (pill && textSpan) {
        pill.className = `live-status-pill ${status.toLowerCase()}`;
        textSpan.textContent = `ROS2: ${status} (${this.bridge.wsUrl})`;
      }

      if (status === 'CONNECTED') {
        if (banner) banner.style.display = 'none';
        if (this.mockMode) {
          this.toggleMockMode();
        }
      }

      const updateBtn = (btn) => {
        if (!btn) return;
        if (status === 'CONNECTED') {
          btn.innerHTML = `
            <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5"><line x1="18" y1="6" x2="6" y2="18"></line><line x1="6" y1="6" x2="18" y2="18"></line></svg>
            <span>Disconnect</span>
          `;
          btn.className = 'live-btn-connect-primary connected';
        } else if (status === 'CONNECTING') {
          btn.innerHTML = `<span>Connecting...</span>`;
          btn.className = 'live-btn-connect-primary connecting';
        } else {
          btn.innerHTML = `
            <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5"><path d="M10 13a5 5 0 0 0 7.54.54l3-3a5 5 0 0 0-7.07-7.07l-1.72 1.71"></path><path d="M14 11a5 5 0 0 0-7.54-.54l-3 3a5 5 0 0 0 7.07 7.07l1.71-1.71"></path></svg>
            <span>Connect ROS2</span>
          `;
          btn.className = 'live-btn-connect-primary';
        }
      };

      updateBtn(btnConnect);
      updateBtn(btnNavConnect);
    };

    // 2. View Mode & 3D PointCloud Controls
    const btnViewMode = document.getElementById('btn-view-mode-toggle');
    const titleText = document.getElementById('live-canvas-title-text');
    btnViewMode?.addEventListener('click', () => {
      this.viewMode = this.viewMode === '2D' ? '3D' : '2D';
      if (this.viewMode === '3D') {
        btnViewMode.innerHTML = '🗺️ 2D Map View';
        btnViewMode.style.borderColor = 'var(--neon-cyan)';
        btnViewMode.style.color = 'var(--neon-cyan)';
        if (titleText?.querySelector('span')) {
          titleText.querySelector('span').textContent = 'LIVE 3D POINTCLOUD ORBIT VISUALIZER (RVIZ2 COMPATIBLE)';
        }
      } else {
        btnViewMode.innerHTML = '🌐 3D Orbit View';
        btnViewMode.style.borderColor = '#bb9af7';
        btnViewMode.style.color = '#c084fc';
        if (titleText?.querySelector('span')) {
          titleText.querySelector('span').textContent = 'LIVE HARDWARE 2D VISUALIZER (ODOM / POINTCLOUD / PROXEMICS)';
        }
      }
    });

    const selColormap = document.getElementById('select-pointcloud-colormap');
    selColormap?.addEventListener('change', (e) => {
      this.pointCloud.colormap = e.target.value;
    });

    const selDecay = document.getElementById('select-pointcloud-decay');
    selDecay?.addEventListener('change', (e) => {
      this.pointCloud.decaySeconds = parseFloat(e.target.value);
    });

    const selFrame = document.getElementById('select-pointcloud-frame');
    selFrame?.addEventListener('change', (e) => {
      this.pointCloud.frameMode = e.target.value;
    });

    // 3. Layer Toggle Buttons
    const toggles = [
      { id: 'toggle-layer-pointcloud', key: 'pointcloud' },
      { id: 'toggle-layer-lidar', key: 'lidar' },
      { id: 'toggle-layer-map', key: 'map' },
      { id: 'toggle-layer-path', key: 'navPath' },
      { id: 'toggle-layer-proxemics', key: 'proxemics' },
      { id: 'toggle-layer-trail', key: 'trail' }
    ];

    toggles.forEach(t => {
      const btn = document.getElementById(t.id);
      btn?.addEventListener('click', () => {
        this.layers[t.key] = !this.layers[t.key];
        btn.classList.toggle('active', this.layers[t.key]);
      });
    });

    // 4. Auto-Center & Clear Screen
    const btnAutoCenter = document.getElementById('btn-live-autocenter');
    btnAutoCenter?.addEventListener('click', () => {
      this.autoCenter = !this.autoCenter;
      btnAutoCenter.classList.toggle('active', this.autoCenter);
    });

    const btnClearScreen = document.getElementById('btn-live-cleartrail');
    btnClearScreen?.addEventListener('click', () => {
      this.clearVisualizerScreen();
    });

    // 4. 2D Nav Goal Mode
    const btnNavGoal = document.getElementById('btn-live-navgoal');
    btnNavGoal?.addEventListener('click', () => {
      this.navGoalMode = !this.navGoalMode;
      btnNavGoal.classList.toggle('active', this.navGoalMode);
      if (this.navGoalMode) {
        this.canvas.style.cursor = 'crosshair';
      } else {
        this.canvas.style.cursor = 'default';
        this.navGoalDraft = null;
      }
    });

    // 5. Zoom Buttons
    document.getElementById('btn-live-zoom-in')?.addEventListener('click', () => {
      this.scale = Math.min(120, this.scale * 1.25);
    });
    document.getElementById('btn-live-zoom-out')?.addEventListener('click', () => {
      this.scale = Math.max(10, this.scale * 0.8);
    });
    document.getElementById('btn-live-zoom-reset')?.addEventListener('click', () => {
      this.scale = 35;
      this.cameraX = this.robot.x;
      this.cameraY = this.robot.y;
    });

    // 6. Mode Switcher (to Simulation)
    const btnSwitchToSim = document.getElementById('btn-switch-to-sim');
    btnSwitchToSim?.addEventListener('click', (e) => {
      e.preventDefault();
      this.triggerModeTransition('SWITCHING TO SIMULATION STUDIO...', 'Loading Social Force physics and synthetic crowd benchmarks...', './index.html');
    });

    // 7. Emergency Stop Button & Spacebar
    const btnEStop = document.getElementById('btn-teleop-estop');
    btnEStop?.addEventListener('click', () => this.emergencyStop());
    window.addEventListener('keydown', (e) => {
      if (e.code === 'Space' && document.activeElement?.tagName !== 'INPUT') {
        e.preventDefault();
        this.emergencyStop();
      }
    });
  }

  triggerModeTransition(title, subtitle, targetUrl) {
    try {
      sessionStorage.setItem('socialnav_skip_boot', 'true');
    } catch (e) {}

    const overlay = document.getElementById('mode-transition-overlay');
    const titleEl = document.getElementById('transition-title-text');
    const subEl = document.getElementById('transition-sub-text');
    const fillEl = document.getElementById('transition-progress-fill');

    if (!overlay) {
      window.location.href = targetUrl;
      return;
    }

    if (titleEl) titleEl.textContent = title;
    if (subEl) subEl.textContent = subtitle;
    if (fillEl) fillEl.style.width = '0%';

    overlay.classList.add('active');

    setTimeout(() => { if (fillEl) fillEl.style.width = '50%'; }, 100);
    setTimeout(() => { if (fillEl) fillEl.style.width = '100%'; }, 350);
    setTimeout(() => { window.location.href = targetUrl; }, 600);
  }

  emergencyStop() {
    this.teleop.linearX = 0;
    this.teleop.angularZ = 0;
    this.bridge.publishCmdVel(0, 0);
    const thumb = document.getElementById('teleop-joystick-thumb');
    if (thumb) thumb.style.transform = 'translate(-50%, -50%)';

    const pill = document.getElementById('live-estop-alert');
    if (pill) {
      pill.style.display = 'block';
      setTimeout(() => { pill.style.display = 'none'; }, 2000);
    }
  }

  initROS2Subscriptions() {
    // Define reusable subscription handlers
    this.subHandlers = {
      // 1. Odometry & Robot Pose
      robotOdom: (msg) => {
        const pos = msg.pose?.pose?.position || msg.pose?.position;
        const ori = msg.pose?.pose?.orientation || msg.pose?.orientation;
        if (pos && typeof pos.x === 'number' && typeof pos.y === 'number') {
          this.robot.x = pos.x;
          this.robot.y = pos.y;
          this.robot.hasData = true;

          if (ori) {
            this.robot.yaw = 2 * Math.atan2(ori.z, ori.w);
          }

          const vx = msg.twist?.twist?.linear?.x || msg.twist?.linear?.x || 0;
          const vy = msg.twist?.twist?.linear?.y || msg.twist?.linear?.y || 0;
          this.robot.linearV = Math.sqrt(vx * vx + vy * vy);
          this.robot.angularW = msg.twist?.twist?.angular?.z || msg.twist?.angular?.z || 0;

          // Record trail
          const now = performance.now();
          if (this.robotTrail.length === 0 || Math.hypot(pos.x - this.robotTrail[this.robotTrail.length - 1].x, pos.y - this.robotTrail[this.robotTrail.length - 1].y) > 0.05) {
            this.robotTrail.push({ x: pos.x, y: pos.y, time: now });
            if (this.robotTrail.length > this.maxTrailPoints) this.robotTrail.shift();
          }

          this.updateKinematicsUI();
        }
      },

      // 2. 2D LiDAR LaserScan
      laserScan: (msg) => {
        if (Array.isArray(msg.ranges) && msg.ranges.length > 0) {
          this.laserScan.angleMin = msg.angle_min !== undefined ? msg.angle_min : -Math.PI;
          this.laserScan.angleMax = msg.angle_max !== undefined ? msg.angle_max : Math.PI;
          this.laserScan.rangeMax = msg.range_max || 12.0;
          const inc = msg.angle_increment || (this.laserScan.angleMax - this.laserScan.angleMin) / msg.ranges.length;

          const hits = [];
          const pts = [];
          const ranges = msg.ranges;
          const rx = this.robot.x;
          const ry = this.robot.y;
          const ryaw = this.robot.yaw;
          const maxRangeM = this.laserScan.rangeMax;

          for (let i = 0; i < ranges.length; i++) {
            const r = ranges[i];
            const localAngleRos = this.laserScan.angleMin + i * inc;
            const rayAngle = ryaw + localAngleRos;
            const cosA = Math.cos(rayAngle);
            const sinA = Math.sin(rayAngle);

            let hitDist = maxRangeM;
            let hitType = 'max';

            if (r > (msg.range_min || 0.05) && r < (maxRangeM - 0.2) && isFinite(r) && !isNaN(r)) {
              hitDist = r;
              hitType = 'obstacle';
              for (const h of this.trackedHumans) {
                const hx = rx + r * cosA;
                const hy = ry + r * sinA;
                if (Math.hypot(hx - h.x, hy - h.y) < 0.6) {
                  hitType = 'human';
                  break;
                }
              }
            }

            const hp = {
              x: rx + hitDist * cosA,
              y: ry + hitDist * sinA,
              dist: hitDist,
              type: hitType
            };
            hits.push(hp);
            if (hitType !== 'max') {
              pts.push({ x: hp.x, y: hp.y, range: hitDist });
            }
          }
          this.laserScan.hits = hits;
          this.laserScan.points = pts;
          this.laserScan.hasData = true;
        }
      },

      // 2.1 3D PointCloud2
      pointCloud: (msg) => {
        const rawPts = decodePointCloud2(msg);
        if (rawPts.length > 0) {
          const frameId = (msg.header?.frame_id || '').toLowerCase();
          const topic = (this.bridge.topics.pointCloud || '').toLowerCase();

          // Determine Frame Transformation Mode
          let isWorldFrame = false;
          let isOpticalFrame = false;

          if (this.pointCloud.frameMode === 'world') {
            isWorldFrame = true;
          } else if (this.pointCloud.frameMode === 'optical') {
            isOpticalFrame = true;
          } else if (this.pointCloud.frameMode === 'robot') {
            isWorldFrame = false;
          } else {
            // Auto Mode based on standard ROS2 conventions
            if (frameId.includes('map') || frameId.includes('odom') || frameId.includes('world') || topic.includes('registered') || topic.includes('fused')) {
              isWorldFrame = true;
            } else if (frameId.includes('optical') || topic.includes('depth/color/points')) {
              isOpticalFrame = true;
            } else {
              // Local robot sensor frame (e.g. livox_frame, velodyne, laser_link, base_link, zed_camera_link)
              isWorldFrame = false;
            }
          }

          const rx = this.robot.x || 0;
          const ry = this.robot.y || 0;
          const rz = this.pointCloud.sensorHeight || 0.35;
          const yawOffsetRad = (this.pointCloud.yawOffsetDeg || 0) * (Math.PI / 180);
          const ryaw = (this.robot.yaw || 0) + yawOffsetRad;
          const cosY = Math.cos(ryaw);
          const sinY = Math.sin(ryaw);
          const flipZ = this.pointCloud.flipZ ? -1 : 1;

          const transformedPts = [];
          for (let i = 0; i < rawPts.length; i++) {
            let lx = rawPts[i].x;
            let ly = rawPts[i].y;
            let lz = rawPts[i].z * flipZ;

            if (isOpticalFrame) {
              // Camera Optical Frame (Z Forward, X Right, Y Down) -> ROS Body Frame (X Forward, Y Left, Z Up)
              const ox = lx;
              const oy = ly;
              const oz = lz;
              lx = oz;
              ly = -ox;
              lz = -oy;
            }

            let wx, wy, wz;
            if (isWorldFrame) {
              // Already world coordinates
              wx = lx;
              wy = ly;
              wz = lz;
            } else {
              // Transform from robot local frame to world coordinates
              wx = rx + (lx * cosY - ly * sinY);
              wy = ry + (lx * sinY + ly * cosY);
              wz = rz + lz;
            }

            transformedPts.push({
              x: wx,
              y: wy,
              z: wz,
              intensity: rawPts[i].intensity
            });
          }

          this.pointCloud.points = transformedPts;
          this.pointCloud.hasData = true;

          if (this.pointCloud.decaySeconds > 0) {
            const now = performance.now();
            const hist = this.pointCloud.history;
            for (let i = 0; i < transformedPts.length; i++) {
              hist.push({
                x: transformedPts[i].x,
                y: transformedPts[i].y,
                z: transformedPts[i].z,
                intensity: transformedPts[i].intensity,
                time: now
              });
            }
            const maxHistory = 40000;
            if (hist.length > maxHistory) {
              this.pointCloud.history = hist.slice(-maxHistory);
            }
          }
        }
      },

      // 3. Tracked Humans & ZED Object Detection
      trackedHumans: (msg) => {
        let humans = [];
        if (Array.isArray(msg.poses)) {
          // geometry_msgs/PoseArray
          humans = msg.poses.map((p, idx) => {
            const pos = p.position || { x: 0, y: 0 };
            const ori = p.orientation || { z: 0, w: 1 };
            const yaw = 2 * Math.atan2(ori.z, ori.w);
            return { id: idx + 1, x: pos.x, y: pos.y, yaw: yaw, vx: 0, vy: 0 };
          });
        } else if (Array.isArray(msg.objects)) {
          // Stereolabs ZED X / ZED 2i obj_det/objects
          humans = msg.objects.map((obj, idx) => {
            const pos = Array.isArray(obj.position)
              ? { x: obj.position[0], y: obj.position[1] }
              : (obj.position || { x: 0, y: 0 });
            return { id: obj.id || idx + 1, x: pos.x || 0, y: pos.y || 0, yaw: 0, vx: 0, vy: 0 };
          });
        } else if (Array.isArray(msg.markers)) {
          // visualization_msgs/MarkerArray
          humans = msg.markers.filter(m => m.pose?.position).map((m, idx) => {
            const pos = m.pose.position;
            return { id: m.id || idx + 1, x: pos.x, y: pos.y, yaw: 0, vx: 0, vy: 0 };
          });
        } else if (Array.isArray(msg.people)) {
          // people_msgs/People
          humans = msg.people.map((p, idx) => {
            const pos = p.position || { x: 0, y: 0 };
            return { id: idx + 1, x: pos.x, y: pos.y, yaw: 0, vx: 0, vy: 0 };
          });
        }

        if (humans.length > 0 || this.trackedHumans.length > 0) {
          this.trackedHumans = humans;
          this.evaluateRealWorldProxemics();
        }
      },

      // 4. Global Plan
      globalPlan: (msg) => {
        if (Array.isArray(msg.poses)) {
          this.navPaths.global = msg.poses.map(p => ({
            x: p.pose?.position?.x || 0,
            y: p.pose?.position?.y || 0
          }));
        }
      },

      // 5. SLAM Map / OccupancyGrid
      map: (msg) => {
        if (msg.info && Array.isArray(msg.data)) {
          this.occupancyMap.width = msg.info.width;
          this.occupancyMap.height = msg.info.height;
          this.occupancyMap.resolution = msg.info.resolution;
          this.occupancyMap.originX = msg.info.origin?.position?.x || 0;
          this.occupancyMap.originY = msg.info.origin?.position?.y || 0;
          this.occupancyMap.data = msg.data;
          this.occupancyMap.hasData = true;
          this.renderOccupancyMapOffscreen();
        }
      },

      // 6. Camera Feed (Supports ZED X image_rect_color, compressed and raw base64)
      cameraCompressed: (msg) => {
        if (msg.data) {
          const format = msg.format || 'jpeg';
          const imgEl = document.getElementById('live-camera-img');
          const noSignal = document.getElementById('camera-no-signal-box');
          if (imgEl) {
            if (typeof msg.data === 'string') {
              imgEl.src = msg.data.startsWith('data:image') ? msg.data : `data:image/${format};base64,${msg.data}`;
            }
            imgEl.style.display = 'block';
          }
          if (noSignal) noSignal.style.display = 'none';
        }
      },

      // 7. Battery State
      battery: (msg) => {
        if (typeof msg.percentage === 'number') {
          const pct = msg.percentage <= 1.0 ? Math.round(msg.percentage * 100) : Math.round(msg.percentage);
          this.battery.percentage = pct;
          this.battery.voltage = +(msg.voltage || 24.0).toFixed(1);
          this.battery.temperature = +(msg.temperature || 32.0).toFixed(1);
          this.battery.hasData = true;
          this.updateBatteryUI();
        }
      },

      // 8. IMU Data
      imu: (msg) => {
        if (msg.linear_acceleration) {
          this.imu.accelX = +(msg.linear_acceleration.x || 0).toFixed(2);
          this.imu.accelY = +(msg.linear_acceleration.y || 0).toFixed(2);
          this.imu.accelZ = +(msg.linear_acceleration.z || 9.81).toFixed(2);
        }
        if (msg.angular_velocity) {
          this.imu.gyroZ = +(msg.angular_velocity.z || 0).toFixed(2);
        }
        this.imu.hasData = true;
        this.updateKinematicsUI();
      }
    };

    // Initial default subscriptions
    this.bridge.subscribeCustom(this.bridge.topics.robotOdom, 'nav_msgs/msg/Odometry', this.subHandlers.robotOdom);
    this.bridge.subscribeCustom(this.bridge.topics.laserScan, 'sensor_msgs/msg/LaserScan', this.subHandlers.laserScan);
    this.bridge.subscribeCustom('/points', 'sensor_msgs/msg/PointCloud2', this.subHandlers.pointCloud);
    this.bridge.subscribeCustom('/velodyne_points', 'sensor_msgs/msg/PointCloud2', this.subHandlers.pointCloud);
    this.bridge.subscribeCustom(this.bridge.topics.trackedHumans, 'geometry_msgs/msg/PoseArray', this.subHandlers.trackedHumans);
    this.bridge.subscribeCustom(this.bridge.topics.globalPlan, 'nav_msgs/msg/Path', this.subHandlers.globalPlan);
    this.bridge.subscribeCustom(this.bridge.topics.map, 'nav_msgs/msg/OccupancyGrid', this.subHandlers.map);
    this.bridge.subscribeCustom(this.bridge.topics.cameraCompressed, 'sensor_msgs/msg/CompressedImage', this.subHandlers.cameraCompressed);
    this.bridge.subscribeCustom(this.bridge.topics.battery, 'sensor_msgs/msg/BatteryState', this.subHandlers.battery);
    this.bridge.subscribeCustom(this.bridge.topics.imu, 'sensor_msgs/msg/Imu', this.subHandlers.imu);
  }

  setupAutoTopicDiscovery() {
    this.bridge.onTopicsDiscovered = (mapping, allTopics) => {
      this.handleTopicsDiscovered(mapping, allTopics);
    };
  }

  handleTopicsDiscovered(mapping, allTopics) {
    if (!mapping) return;

    // Dynamically re-subscribe to all matched topics
    for (const [roleKey, info] of Object.entries(mapping)) {
      const handler = this.subHandlers[roleKey];
      if (handler && info.topic) {
        this.bridge.remapTopic(roleKey, info.topic, handler);
      }
    }

    // Update UI counters
    const totalCount = Object.keys(allTopics || {}).length;
    const mappedCount = Object.values(mapping).filter(m => m.autoMatched || (m.topic && m.candidates?.length > 0)).length;

    const elTotal = document.getElementById('remapper-total-count');
    const elMapped = document.getElementById('remapper-mapped-count');
    const elRawCount = document.getElementById('remapper-raw-count');
    const elStatus = document.getElementById('remapper-status-text');

    if (elTotal) elTotal.textContent = totalCount;
    if (elMapped) elMapped.textContent = `${mappedCount} / ${Object.keys(mapping).length}`;
    if (elRawCount) elRawCount.textContent = totalCount;
    if (elStatus) elStatus.textContent = `Auto-Resolved (${totalCount} topics active)`;

    // Render topic remapper modal UI if open or updated
    this.renderTopicRemapperUI(mapping, allTopics);

    // Also update Inspector / Deck badges
    const cameraCardHeader = document.querySelector('#live-camera-card .live-card-header span');
    if (cameraCardHeader && mapping.cameraCompressed?.topic) {
      cameraCardHeader.textContent = `FPV CAMERA (${mapping.cameraCompressed.topic})`;
    }
  }

  initTopicRemapperUI() {
    const modal = document.getElementById('topic-remapper-modal');
    const btnOpen = document.getElementById('btn-open-topic-remapper');
    const btnClose = document.getElementById('btn-close-topic-remapper');
    const btnRescan = document.getElementById('btn-rescan-topics');

    const openModal = () => {
      if (!modal) return;
      modal.classList.add('open');
      if (this.bridge.resolvedMapping && Object.keys(this.bridge.resolvedMapping).length > 0) {
        this.renderTopicRemapperUI(this.bridge.resolvedMapping, this.bridge.discoveredTopics);
      } else {
        this.bridge.discoverTopicsAndTypes();
      }
    };

    const closeModal = () => {
      if (!modal) return;
      modal.classList.remove('open');
    };

    btnOpen?.addEventListener('click', openModal);
    btnClose?.addEventListener('click', closeModal);

    modal?.addEventListener('click', (e) => {
      if (e.target === modal) closeModal();
    });

    window.addEventListener('keydown', (e) => {
      if (e.key === 'Escape' && modal?.classList.contains('open')) {
        closeModal();
      }
    });

    btnRescan?.addEventListener('click', () => {
      const elStatus = document.getElementById('remapper-status-text');
      if (elStatus) elStatus.textContent = 'Scanning ROS2 topics...';

      this.bridge.discoverTopicsAndTypes();
      if (this.sshManager && this.sshManager.isConnected) {
        this.sshManager.fetchRobotTopics();
      }
    });
  }

  renderTopicRemapperUI(mapping, allTopics = {}) {
    const grid = document.getElementById('topic-remapper-grid');
    const tableBody = document.getElementById('remapper-raw-table-body');
    if (!grid) return;

    grid.innerHTML = '';

    const allTopicNames = Object.keys(allTopics);

    for (const [roleKey, info] of Object.entries(mapping)) {
      const card = document.createElement('div');
      const isMapped = !!info.topic && (info.autoMatched || (info.candidates && info.candidates.length > 0));
      card.className = `topic-role-card ${isMapped ? 'mapped' : 'unmapped'}`;

      const statusBadge = info.autoMatched
        ? '<span class="topic-match-status-badge auto">✨ Auto-Mapped</span>'
        : (info.topic ? '<span class="topic-match-status-badge manual">Custom</span>' : '<span class="topic-match-status-badge none">Not Found</span>');

      // Candidate topics options
      let optionsHtml = '';
      const candidateList = info.candidates && info.candidates.length > 0 ? info.candidates : (info.topic ? [info.topic] : []);
      
      // Include all discovered topics in dropdown for ultimate flexibility
      const extraTopics = allTopicNames.filter(t => !candidateList.includes(t));

      candidateList.forEach(t => {
        const isSelected = t === info.topic;
        optionsHtml += `<option value="${t}" ${isSelected ? 'selected' : ''}>🎯 ${t} (${allTopics[t] || info.type})</option>`;
      });

      if (extraTopics.length > 0) {
        optionsHtml += `<optgroup label="Other Detected Topics">`;
        extraTopics.forEach(t => {
          const isSelected = t === info.topic;
          optionsHtml += `<option value="${t}" ${isSelected ? 'selected' : ''}>${t} (${allTopics[t] || 'unknown'})</option>`;
        });
        optionsHtml += `</optgroup>`;
      }

      card.innerHTML = `
        <div class="topic-role-top">
          <span class="topic-role-title">${info.label}</span>
          ${statusBadge}
        </div>
        <select class="topic-role-select" data-role="${roleKey}">
          ${optionsHtml || `<option value="${info.topic || ''}">${info.topic || 'No topic detected'}</option>`}
        </select>
        <div class="topic-role-footer">
          <span>Type: <code>${info.type}</code></span>
          <span>Candidates: ${candidateList.length}</span>
        </div>
      `;

      // Bind change listener
      const selectEl = card.querySelector('.topic-role-select');
      selectEl?.addEventListener('change', (e) => {
        const newTopic = e.target.value;
        const handler = this.subHandlers[roleKey];
        this.bridge.remapTopic(roleKey, newTopic, handler);
        info.autoMatched = false;
        info.topic = newTopic;
        this.renderTopicRemapperUI(mapping, allTopics);
      });

      grid.appendChild(card);
    }

    // Render raw discovered topics table
    if (tableBody) {
      if (allTopicNames.length === 0) {
        tableBody.innerHTML = '<tr><td colspan="3" style="text-align: center; color: var(--text-muted); padding: 12px;">No active topics detected yet. Connect ROS2 or click Re-Scan.</td></tr>';
      } else {
        tableBody.innerHTML = allTopicNames.map(t => {
          const type = allTopics[t] || 'unknown';
          const stat = this.bridge.topicStats.get(t);
          const hzText = stat && stat.hz > 0 ? `<span style="color: var(--neon-green);">${stat.hz} Hz</span>` : '<span style="color: var(--text-muted);">Idle</span>';
          return `
            <tr>
              <td><strong style="color: var(--neon-cyan);">${t}</strong></td>
              <td><code>${type}</code></td>
              <td>${hzText}</td>
            </tr>
          `;
        }).join('');
      }
    }
  }

  evaluateRealWorldProxemics() {
    if (!this.robot.hasData || this.trackedHumans.length === 0) return;

    let minDist = 999.0;
    let intimateViolations = 0;
    let personalViolations = 0;

    for (const h of this.trackedHumans) {
      const d = Math.hypot(this.robot.x - h.x, this.robot.y - h.y);
      if (d < minDist) minDist = d;
      if (d < 0.45) intimateViolations++;
      else if (d < 1.20) personalViolations++;
    }

    this.proxemicsStats.minHumanDistance = +minDist.toFixed(2);
    this.proxemicsStats.intimateBreaches = intimateViolations;
    this.proxemicsStats.personalBreaches = personalViolations;

    // Compute comfort index (100% minus penalties)
    let penalty = intimateViolations * 25 + personalViolations * 8;
    this.proxemicsStats.comfortScore = Math.max(0, 100 - penalty);

    const distEl = document.getElementById('live-prox-mindist');
    const breachEl = document.getElementById('live-prox-breaches');
    const comfortEl = document.getElementById('live-prox-comfort');

    if (distEl) distEl.textContent = `${this.proxemicsStats.minHumanDistance}m`;
    if (breachEl) breachEl.textContent = `${intimateViolations} Intimate / ${personalViolations} Personal`;
    if (comfortEl) comfortEl.textContent = `${this.proxemicsStats.comfortScore}%`;
  }

  updateBatteryUI() {
    const pctEl = document.getElementById('live-battery-pct');
    const voltEl = document.getElementById('live-battery-volt');
    const barEl = document.getElementById('live-battery-bar');

    if (pctEl) pctEl.textContent = `${this.battery.percentage}%`;
    if (voltEl) voltEl.textContent = `${this.battery.voltage}V`;
    if (barEl) {
      barEl.style.width = `${this.battery.percentage}%`;
      barEl.style.background = this.battery.percentage < 25 ? '#ff4444' : this.battery.percentage < 50 ? '#ffb86c' : 'var(--neon-green)';
    }
  }

  updateKinematicsUI() {
    const vEl = document.getElementById('live-kin-v');
    const wEl = document.getElementById('live-kin-w');
    const yawEl = document.getElementById('live-kin-yaw');
    const axEl = document.getElementById('live-kin-ax');

    if (vEl) vEl.textContent = `${this.robot.linearV.toFixed(2)} m/s`;
    if (wEl) wEl.textContent = `${this.robot.angularW.toFixed(2)} r/s`;
    if (yawEl) yawEl.textContent = `${(this.robot.yaw * 180 / Math.PI).toFixed(1)}°`;
    if (axEl) axEl.textContent = `${this.imu.accelX} m/s²`;
  }

  renderOccupancyMapOffscreen() {
    const { width, height, data } = this.occupancyMap;
    if (!width || !height || !data) return;

    if (!this.occupancyMap.offscreenCanvas) {
      this.occupancyMap.offscreenCanvas = document.createElement('canvas');
    }
    const offCanvas = this.occupancyMap.offscreenCanvas;
    offCanvas.width = width;
    offCanvas.height = height;
    const offCtx = offCanvas.getContext('2d');
    const imgData = offCtx.createImageData(width, height);
    const d = imgData.data;

    // In ROS OccupancyGrid, row 0 is bottom (Y-min), row (height-1) is top (Y-max).
    // Canvas ImageData row 0 is top (Y-min). We flip Y so map aligns with ROS world coordinates.
    for (let gy = 0; gy < height; gy++) {
      const canvasY = height - 1 - gy;
      for (let gx = 0; gx < width; gx++) {
        const gridIdx = gy * width + gx;
        const imgIdx = (canvasY * width + gx) * 4;
        const val = data[gridIdx];

        if (val === -1) {
          // Unknown: transparent
          d[imgIdx] = 0; d[imgIdx+1] = 0; d[imgIdx+2] = 0; d[imgIdx+3] = 0;
        } else if (val === 0) {
          // Free space: subtle dark floor tint
          d[imgIdx] = 12; d[imgIdx+1] = 20; d[imgIdx+2] = 28; d[imgIdx+3] = 40;
        } else {
          // Occupied obstacle/wall: crisp cyan
          const alpha = Math.min(255, val * 2.0);
          d[imgIdx] = 0; d[imgIdx+1] = 229; d[imgIdx+2] = 255; d[imgIdx+3] = alpha;
        }
      }
    }
    offCtx.putImageData(imgData, 0, 0);
  }

  initCanvasInteractions() {
    this.canvas.addEventListener('contextmenu', (e) => e.preventDefault());
    this.canvas.addEventListener('auxclick', (e) => { if (e.button === 1) e.preventDefault(); });

    this.canvas.addEventListener('mousedown', (e) => {
      if (e.button === 1) e.preventDefault();

      const rect = this.canvas.getBoundingClientRect();
      const mouseX = e.clientX - rect.left;
      const mouseY = e.clientY - rect.top;

      if (this.viewMode === '3D') {
        // Right click (2) OR Shift + Click: 3D Pan
        // Left click (0) OR Middle wheel click (1): 3D Orbit Rotate
        if (e.button === 2 || (e.shiftKey && (e.button === 0 || e.button === 1))) {
          this.isPanning = true;
          this.canvas.style.cursor = 'move';
        } else {
          this.isOrbiting = true;
          this.canvas.style.cursor = 'grab';
        }
        this.dragStartX = mouseX;
        this.dragStartY = mouseY;
        return;
      }

      if (this.navGoalMode) {
        const rosPos = this.toROS(mouseX, mouseY);
        this.navGoalDraft = { startX: rosPos.x, startY: rosPos.y, currentX: rosPos.x, currentY: rosPos.y };
        return;
      }

      this.isDragging = true;
      this.dragStartX = mouseX;
      this.dragStartY = mouseY;
      this.camStartX = this.cameraX;
      this.camStartY = this.cameraY;
    });

    window.addEventListener('mousemove', (e) => {
      if (!this.canvas) return;
      const rect = this.canvas.getBoundingClientRect();
      const mouseX = e.clientX - rect.left;
      const mouseY = e.clientY - rect.top;

      if (this.viewMode === '3D') {
        const dx = mouseX - this.dragStartX;
        const dy = mouseY - this.dragStartY;
        this.dragStartX = mouseX;
        this.dragStartY = mouseY;

        if (this.isOrbiting) {
          this.orbitCamera.rotate(-dx * 0.009, -dy * 0.009);
        } else if (this.isPanning) {
          this.orbitCamera.pan(dx, dy);
        }
        return;
      }

      if (this.navGoalMode && this.navGoalDraft) {
        const rosPos = this.toROS(mouseX, mouseY);
        this.navGoalDraft.currentX = rosPos.x;
        this.navGoalDraft.currentY = rosPos.y;
        return;
      }

      if (this.isDragging) {
        const dx = (mouseX - this.dragStartX) / this.scale;
        const dy = (mouseY - this.dragStartY) / this.scale;
        this.cameraX = this.camStartX - dx;
        this.cameraY = this.camStartY + dy; // Invert Y
        this.autoCenter = false;
        document.getElementById('btn-live-autocenter')?.classList.remove('active');
      }

      // Update cursor coordinates badge
      const rosPos = this.toROS(mouseX, mouseY);
      const coordEl = document.getElementById('live-cursor-coord');
      if (coordEl) coordEl.textContent = `X: ${rosPos.x.toFixed(2)}m, Y: ${rosPos.y.toFixed(2)}m`;
    });

    window.addEventListener('mouseup', () => {
      if (this.viewMode === '3D') {
        this.isOrbiting = false;
        this.isPanning = false;
        this.canvas.style.cursor = 'default';
        return;
      }

      if (this.navGoalMode && this.navGoalDraft) {
        const dx = this.navGoalDraft.currentX - this.navGoalDraft.startX;
        const dy = this.navGoalDraft.currentY - this.navGoalDraft.startY;
        let yaw = 0;
        if (Math.hypot(dx, dy) > 0.1) {
          yaw = Math.atan2(dy, dx);
        }
        this.publishNavGoal(this.navGoalDraft.startX, this.navGoalDraft.startY, yaw);
        this.navGoalDraft = null;
        this.navGoalMode = false;
        document.getElementById('btn-live-navgoal')?.classList.remove('active');
        this.canvas.style.cursor = 'default';
        return;
      }
      this.isDragging = false;
    });

    this.canvas.addEventListener('wheel', (e) => {
      e.preventDefault();
      if (this.viewMode === '3D') {
        const zoomFactor = e.deltaY > 0 ? 1.12 : 0.88;
        this.orbitCamera.zoom(zoomFactor);
      } else {
        const zoomFactor = e.deltaY < 0 ? 1.15 : 0.85;
        this.scale = Math.max(8, Math.min(160, this.scale * zoomFactor));
      }
    }, { passive: false });
  }

  publishNavGoal(x, y, yaw = 0) {
    const qz = Math.sin(-yaw / 2);
    const qw = Math.cos(-yaw / 2);

    const goalMsg = {
      op: 'publish',
      topic: '/goal_pose',
      msg: {
        header: {
          stamp: { sec: Math.floor(Date.now() / 1000), nanosec: 0 },
          frame_id: 'map'
        },
        pose: {
          position: { x: +x.toFixed(3), y: +y.toFixed(3), z: 0.0 },
          orientation: { x: 0.0, y: 0.0, z: +qz.toFixed(4), w: +qw.toFixed(4) }
        }
      }
    };

    if (this.bridge.isConnected && this.bridge.socket) {
      this.bridge.socket.send(JSON.stringify(goalMsg));
      this.goalPose = { x, y, yaw };
    }
  }

  toScreen(rosX, rosY) {
    const cx = this.canvasWidth / 2;
    const cy = this.canvasHeight / 2;
    const sx = cx + (rosX - this.cameraX) * this.scale;
    const sy = cy - (rosY - this.cameraY) * this.scale; // Invert Y
    return { x: sx, y: sy };
  }

  toROS(screenX, screenY) {
    const cx = this.canvasWidth / 2;
    const cy = this.canvasHeight / 2;
    const rx = this.cameraX + (screenX - cx) / this.scale;
    const ry = this.cameraY - (screenY - cy) / this.scale; // Invert Y
    return { x: +rx.toFixed(3), y: +ry.toFixed(3) };
  }

  initTeleopWASD() {
    this.activeKeys = new Set();
    const linearSpeed = 0.65;  // m/s
    const angularSpeed = 1.10; // rad/s

    const keyElements = {
      w: document.getElementById('teleop-key-w'),
      a: document.getElementById('teleop-key-a'),
      s: document.getElementById('teleop-key-s'),
      d: document.getElementById('teleop-key-d')
    };

    const updateKeyVisuals = () => {
      keyElements.w?.classList.toggle('active', this.activeKeys.has('w') || this.activeKeys.has('arrowup'));
      keyElements.a?.classList.toggle('active', this.activeKeys.has('a') || this.activeKeys.has('arrowleft'));
      keyElements.s?.classList.toggle('active', this.activeKeys.has('s') || this.activeKeys.has('arrowdown'));
      keyElements.d?.classList.toggle('active', this.activeKeys.has('d') || this.activeKeys.has('arrowright'));
    };

    let isPublishing = false;

    const processTeleopTick = () => {
      let lin = 0;
      let ang = 0;

      if (this.activeKeys.has('w') || this.activeKeys.has('arrowup')) lin += linearSpeed;
      if (this.activeKeys.has('s') || this.activeKeys.has('arrowdown')) lin -= linearSpeed;
      if (this.activeKeys.has('a') || this.activeKeys.has('arrowleft')) ang += angularSpeed;
      if (this.activeKeys.has('d') || this.activeKeys.has('arrowright')) ang -= angularSpeed;

      if (lin !== 0 || ang !== 0) {
        isPublishing = true;
        this.teleop.linearX = lin;
        this.teleop.angularZ = ang;
        this.bridge.publishCmdVel(lin, ang);

        // If in Mock Stream Mode, update mock robot position directly
        if (this.mockMode && this.robot) {
          const dt = 0.05;
          this.robot.yaw += ang * dt;
          this.robot.x += lin * Math.cos(this.robot.yaw) * dt;
          this.robot.y += lin * Math.sin(this.robot.yaw) * dt;
          this.robot.linearV = lin;
          this.robot.angularW = ang;
          this.updateKinematicsUI();
        }
      } else if (isPublishing) {
        isPublishing = false;
        this.teleop.linearX = 0;
        this.teleop.angularZ = 0;
        this.bridge.publishCmdVel(0, 0);
        if (this.mockMode && this.robot) {
          this.robot.linearV = 0;
          this.robot.angularW = 0;
          this.updateKinematicsUI();
        }
      }
    };

    // 20 Hz Command Loop
    if (this.teleop.intervalTimer) clearInterval(this.teleop.intervalTimer);
    this.teleop.intervalTimer = setInterval(processTeleopTick, 50);

    // Keyboard Listeners
    window.addEventListener('keydown', (e) => {
      if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;

      const key = e.key.toLowerCase();
      if (['w', 'a', 's', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright'].includes(key)) {
        e.preventDefault();
        this.activeKeys.add(key);
        updateKeyVisuals();
      } else if (key === ' ' || e.code === 'Space') {
        e.preventDefault();
        this.triggerEmergencyStop();
      }
    });

    window.addEventListener('keyup', (e) => {
      const key = e.key.toLowerCase();
      if (['w', 'a', 's', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright'].includes(key)) {
        this.activeKeys.delete(key);
        updateKeyVisuals();
      }
    });

    // Mouse / Touch holding on button pad
    const bindButtonTouch = (btn, keyChar) => {
      if (!btn) return;
      const startPress = (e) => {
        e.preventDefault();
        this.activeKeys.add(keyChar);
        updateKeyVisuals();
      };
      const endPress = (e) => {
        e.preventDefault();
        this.activeKeys.delete(keyChar);
        updateKeyVisuals();
      };

      btn.addEventListener('mousedown', startPress);
      btn.addEventListener('mouseup', endPress);
      btn.addEventListener('mouseleave', endPress);
      btn.addEventListener('touchstart', startPress, { passive: false });
      btn.addEventListener('touchend', endPress, { passive: false });
    };

    bindButtonTouch(keyElements.w, 'w');
    bindButtonTouch(keyElements.a, 'a');
    bindButtonTouch(keyElements.s, 's');
    bindButtonTouch(keyElements.d, 'd');

    // E-Stop button
    const btnEstop = document.getElementById('btn-teleop-estop');
    btnEstop?.addEventListener('click', () => this.triggerEmergencyStop());
  }

  triggerEmergencyStop() {
    this.activeKeys.clear();
    const keyElements = [
      document.getElementById('teleop-key-w'),
      document.getElementById('teleop-key-a'),
      document.getElementById('teleop-key-s'),
      document.getElementById('teleop-key-d')
    ];
    keyElements.forEach(el => el?.classList.remove('active'));

    this.teleop.linearX = 0;
    this.teleop.angularZ = 0;
    this.bridge.publishCmdVel(0, 0);

    if (this.mockMode && this.robot) {
      this.robot.linearV = 0;
      this.robot.angularW = 0;
      this.updateKinematicsUI();
    }

    const btnEstop = document.getElementById('btn-teleop-estop');
    if (btnEstop) {
      btnEstop.style.transform = 'scale(0.96)';
      setTimeout(() => { btnEstop.style.transform = 'none'; }, 150);
    }
  }

  initTopicInspector() {
    setInterval(() => {
      this.refreshTopicTable();
    }, 1000);

    const closeBtn = document.getElementById('btn-close-payload-drawer');
    closeBtn?.addEventListener('click', () => {
      const drawer = document.getElementById('payload-drawer-modal');
      if (drawer) drawer.style.display = 'none';
    });

    // 1. Auto-Detect Topics Option Toggle Sync (Toolbar & Inspector)
    const chkTop = document.getElementById('chk-auto-detect-topics-top');
    const chkInspector = document.getElementById('chk-auto-detect-topics-inspector');

    const isAutoDetectEnabled = this.bridge.autoDetectTopics !== false;
    if (chkTop) chkTop.checked = isAutoDetectEnabled;
    if (chkInspector) chkInspector.checked = isAutoDetectEnabled;

    const handleAutoDetectToggle = (e) => {
      const enabled = e.target.checked;
      this.bridge.autoDetectTopics = enabled;
      try {
        localStorage.setItem('socialnav_auto_detect_topics', enabled);
      } catch (err) { console.warn(err); }

      if (chkTop) chkTop.checked = enabled;
      if (chkInspector) chkInspector.checked = enabled;

      if (enabled && this.bridge.isConnected) {
        this.bridge.discoverTopicsAndTypes();
      }
    };

    chkTop?.addEventListener('change', handleAutoDetectToggle);
    chkInspector?.addEventListener('change', handleAutoDetectToggle);

    // 2. Manual Custom Topic Addition
    const btnAddTopic = document.getElementById('btn-add-custom-topic');
    const inputTopic = document.getElementById('input-custom-topic-name');
    const selectRole = document.getElementById('select-custom-topic-role');

    const handleAddCustomTopic = () => {
      const rawTopic = inputTopic?.value.trim();
      if (!rawTopic) return;

      const topicName = rawTopic.startsWith('/') ? rawTopic : `/${rawTopic}`;
      const roleKey = selectRole?.value || 'custom';

      if (roleKey !== 'custom' && this.subHandlers[roleKey]) {
        const handler = this.subHandlers[roleKey];
        this.bridge.remapTopic(roleKey, topicName, handler);
      } else {
        this.bridge.subscribeCustom(topicName, 'auto', (msg) => {
          // Track general packet
        });
      }

      // Initialize placeholder stat
      if (!this.bridge.topicStats.has(topicName)) {
        this.bridge.topicStats.set(topicName, {
          count: 0,
          lastTime: performance.now(),
          hz: 0,
          hzAccum: [],
          lastMsg: null,
          bytes: 0,
          type: 'custom'
        });
      }

      if (inputTopic) inputTopic.value = '';
      this.refreshTopicTable();
    };

    btnAddTopic?.addEventListener('click', handleAddCustomTopic);
    inputTopic?.addEventListener('keydown', (e) => {
      if (e.key === 'Enter') {
        e.preventDefault();
        handleAddCustomTopic();
      }
    });
  }

  refreshTopicTable() {
    const tbody = document.getElementById('live-topics-tbody');
    if (!tbody) return;

    const stats = this.bridge.topicStats;
    if (stats.size === 0) {
      tbody.innerHTML = `<tr><td colspan="6" style="text-align: center; color: var(--text-muted); padding: 16px;">Waiting for live ROS2 topic packets or custom topics...</td></tr>`;
      return;
    }

    // Identify which visualizer role matches each topic
    const roleLabels = {
      robotOdom: '🚗 Odometry / Pose',
      laserScan: '📡 2D LiDAR Scan',
      pointCloud: '☁️ 3D PointCloud2',
      cameraCompressed: '📷 FPV Camera Feed',
      trackedHumans: '🚶 Tracked Humans / AI',
      map: '🗺️ SLAM Map',
      globalPlan: '🚩 Nav2 Global Plan',
      battery: '🔋 Battery State',
      imu: '🧭 IMU Sensor'
    };

    let rows = '';
    let visibleCount = 0;

    for (const [topic, st] of stats) {
      if (this.bridge.removedTopics && this.bridge.removedTopics.has(topic)) {
        continue;
      }

      visibleCount++;
      const kb = (st.bytes / 1024).toFixed(1);

      // Find role
      let roleText = '⚡ General Telemetry';
      for (const [rKey, rTopic] of Object.entries(this.bridge.topics)) {
        if (rTopic === topic && roleLabels[rKey]) {
          roleText = `<strong style="color: var(--neon-green);">${roleLabels[rKey]}</strong>`;
          break;
        }
      }

      const hzDisplay = st.hz > 0
        ? `<span class="topic-rate-val" style="color: var(--neon-green); font-weight: 700;">${st.hz} Hz</span>`
        : `<span style="color: var(--text-muted);">Idle</span>`;

      rows += `
        <tr>
          <td><strong style="color: var(--neon-cyan);">${topic}</strong></td>
          <td><span style="font-size: 11px;">${roleText}</span></td>
          <td>${hzDisplay}</td>
          <td>${st.count} msgs</td>
          <td>${kb} KB</td>
          <td>
            <button class="btn-action-inspect" data-topic="${topic}" title="Inspect last JSON payload">Inspect</button>
            <button class="btn-action-remove" data-topic="${topic}" title="Unsubscribe and purge from visualizer">✕ Remove</button>
          </td>
        </tr>
      `;
    }

    if (visibleCount === 0) {
      tbody.innerHTML = `<tr><td colspan="6" style="text-align: center; color: var(--text-muted); padding: 16px;">No active topics monitored. Add a topic above or toggle Auto-Detect.</td></tr>`;
      return;
    }

    tbody.innerHTML = rows;

    // Bind Inspect
    tbody.querySelectorAll('.btn-action-inspect').forEach(btn => {
      btn.addEventListener('click', (e) => {
        e.stopPropagation();
        const t = btn.getAttribute('data-topic');
        this.openPayloadDrawer(t);
      });
    });

    // Bind Remove / Unsubscribe
    tbody.querySelectorAll('.btn-action-remove').forEach(btn => {
      btn.addEventListener('click', (e) => {
        e.stopPropagation();
        const t = btn.getAttribute('data-topic');
        if (!t) return;

        // 1. Unsubscribe and permanently purge from bridge
        this.bridge.unsubscribeCustom(t);

        // 2. Immediately clear all associated visualizer data from canvas and UI
        this.clearVisualizerTopicData(t);

        // 3. Force instant visualizer redraw so items vanish immediately
        if (this.ctx) {
          this.draw();
        }

        // 4. Re-render table immediately
        this.refreshTopicTable();
      });
    });
  }

  clearVisualizerTopicData(topic) {
    if (!topic) return;
    const t = topic.toLowerCase();

    // 1. PointCloud (3D / 2D)
    if (this.bridge.topics.pointCloud === topic || t.includes('point') || t.includes('cloud') || t.includes('lidar') || t.includes('livox')) {
      this.pointCloud.points = [];
      this.pointCloud.history = [];
      this.pointCloud.hasData = false;
      if (this.bridge.topics.pointCloud === topic) {
        this.bridge.topics.pointCloud = '';
      }
    }

    // 2. 2D LaserScan
    if (this.bridge.topics.laserScan === topic || t.includes('scan') || t.includes('laser')) {
      this.laserScan.hits = [];
      this.laserScan.points = [];
      this.laserScan.hasData = false;
      if (this.bridge.topics.laserScan === topic) {
        this.bridge.topics.laserScan = '';
      }
    }

    // 3. Camera / Video Feed
    if (this.bridge.topics.cameraCompressed === topic || t.includes('image') || t.includes('camera') || t.includes('rgb') || t.includes('compressed')) {
      const imgEl = document.getElementById('live-camera-img');
      const noSignal = document.getElementById('camera-no-signal-box');
      if (imgEl) {
        imgEl.style.display = 'none';
        imgEl.src = '';
      }
      if (noSignal) noSignal.style.display = 'flex';
      if (this.bridge.topics.cameraCompressed === topic) {
        this.bridge.topics.cameraCompressed = '';
      }
    }

    // 4. Tracked Humans / Social Obstacles / AI Detection
    if (this.bridge.topics.trackedHumans === topic || t.includes('human') || t.includes('people') || t.includes('person') || t.includes('obj_det') || t.includes('detected')) {
      this.trackedHumans = [];
      if (this.bridge.topics.trackedHumans === topic) {
        this.bridge.topics.trackedHumans = '';
      }
    }

    // 5. OccupancyGrid Maps (SLAM Map, Global/Local Costmaps)
    if (this.bridge.topics.map === topic || this.bridge.topics.socialCostmap === topic || t.includes('map') || t.includes('costmap')) {
      this.occupancyMap.hasData = false;
      this.occupancyMap.data = null;
      this.occupancyMap.offscreenCanvas = null;
      this.occupancyMap.ctx = null;
      if (this.bridge.topics.map === topic) this.bridge.topics.map = '';
      if (this.bridge.topics.socialCostmap === topic) this.bridge.topics.socialCostmap = '';
    }

    // 6. Navigation Plans (Global / Local Paths)
    if (this.bridge.topics.globalPlan === topic || t.includes('global_plan') || t.includes('path')) {
      this.navPaths.global = [];
      if (this.bridge.topics.globalPlan === topic) this.bridge.topics.globalPlan = '';
    }
    if (this.bridge.topics.localPlan === topic || t.includes('local_plan')) {
      this.navPaths.local = [];
      if (this.bridge.topics.localPlan === topic) this.bridge.topics.localPlan = '';
    }

    // 7. Robot Odometry & Pose
    if (this.bridge.topics.robotPose === topic || this.bridge.topics.robotOdom === topic || t.includes('odom') || t.includes('pose')) {
      this.robot.hasData = false;
      this.robot.trail = [];
      this.robot.linearVel = 0;
      this.robot.angularVel = 0;
      if (this.bridge.topics.robotPose === topic) this.bridge.topics.robotPose = '';
      if (this.bridge.topics.robotOdom === topic) this.bridge.topics.robotOdom = '';
    }

    // 8. Goal Pose / Clicked Target
    if (this.bridge.topics.goalPose === topic || this.bridge.topics.moveBaseGoal === topic || this.bridge.topics.clickedPoint === topic || t.includes('goal')) {
      this.goalPose = null;
      this.navGoalDraft = null;
      if (this.bridge.topics.goalPose === topic) this.bridge.topics.goalPose = '';
      if (this.bridge.topics.moveBaseGoal === topic) this.bridge.topics.moveBaseGoal = '';
      if (this.bridge.topics.clickedPoint === topic) this.bridge.topics.clickedPoint = '';
    }

    // 9. Battery & Diagnostics
    if (this.bridge.topics.battery === topic || t.includes('battery')) {
      this.diagnostics.battery = null;
      this.diagnostics.batteryPercentage = 0;
      const batEl = document.getElementById('stat-battery');
      if (batEl) batEl.textContent = 'N/A';
      if (this.bridge.topics.battery === topic) this.bridge.topics.battery = '';
    }
    if (this.bridge.topics.imu === topic || t.includes('imu')) {
      this.diagnostics.imu = null;
      if (this.bridge.topics.imu === topic) this.bridge.topics.imu = '';
    }
    if (this.bridge.topics.diagnostics === topic || t.includes('diagnostic')) {
      this.diagnostics.systemHealth = 'UNKNOWN';
      if (this.bridge.topics.diagnostics === topic) this.bridge.topics.diagnostics = '';
    }
  }

  clearVisualizerScreen() {
    // 1. Purge PointCloud (3D/2D and decay accumulation)
    this.pointCloud.points = [];
    this.pointCloud.history = [];
    this.pointCloud.hasData = false;

    // 2. Purge 2D LaserScan
    this.laserScan.hits = [];
    this.laserScan.points = [];
    this.laserScan.hasData = false;

    // 3. Purge Robot Trails & Tracked Humans
    this.robotTrail = [];
    this.trackedHumans = [];

    // 4. Purge Navigation paths & goals
    this.navPaths.global = [];
    this.navPaths.local = [];
    this.goalPose = null;
    this.navGoalDraft = null;
    this.navGoalSent = null;

    // 5. Purge SLAM Occupancy Map / Costmaps
    this.occupancyMap.hasData = false;
    this.occupancyMap.data = null;
    this.occupancyMap.offscreenCanvas = null;
    this.occupancyMap.ctx = null;

    // 6. Immediately trigger re-render of clean canvas
    this.draw();
  }

  openPayloadDrawer(topic) {
    const st = this.bridge.topicStats.get(topic);
    if (!st || !st.lastMsg) return;

    const drawer = document.getElementById('payload-drawer-modal');
    const titleEl = document.getElementById('payload-drawer-topic-name');
    const contentEl = document.getElementById('payload-drawer-code');

    if (titleEl) titleEl.textContent = `Topic: ${topic} (${st.hz} Hz)`;
    if (contentEl) contentEl.textContent = JSON.stringify(st.lastMsg, null, 2);
    if (drawer) drawer.style.display = 'flex';
  }

  renderLoop() {
    if (this.autoCenter && this.robot.hasData) {
      this.cameraX += (this.robot.x - this.cameraX) * 0.1;
      this.cameraY += (this.robot.y - this.cameraY) * 0.1;
    }

    this.draw();
    this.animationFrameId = requestAnimationFrame(() => this.renderLoop());
  }

  draw() {
    if (!this.ctx) return;
    const ctx = this.ctx;
    const w = this.canvasWidth;
    const h = this.canvasHeight;

    ctx.clearRect(0, 0, w, h);

    if (this.viewMode === '3D') {
      this.draw3DOrbitScene(ctx, w, h);
      return;
    }

    // --- 2D Top-Down Navigation View ---

    // 1. Grid & Metric Axes
    if (this.layers.grid) {
      this.drawMetricGrid(ctx, w, h);
    }

    // 2. SLAM OccupancyGrid Map & Static Obstacles
    if (this.layers.map && this.occupancyMap.hasData && this.occupancyMap.offscreenCanvas) {
      this.drawOccupancyGrid(ctx);
    }
    // Only draw synthetic mock static obstacles when in mock mode
    if (this.mockMode && this.layers.map) {
      this.drawStaticObstacles(ctx);
    }

    // 3. Nav2 Paths (/plan)
    if (this.layers.navPath && this.navPaths.global.length > 1) {
      this.drawNavPath(ctx, this.navPaths.global, 'rgba(0, 229, 255, 0.85)', 3);
    }

    // 4. Past Robot Trajectory Trail
    if (this.layers.trail && this.robotTrail.length > 1) {
      this.drawRobotTrail(ctx);
    }

    // 5. Tracked Humans & Real-World Proxemics Zones
    if (this.trackedHumans.length > 0) {
      this.drawHumansAndProxemics(ctx);
    }

    // 6. 3D LiDAR PointCloud Projected in 2D
    if (this.layers.pointcloud && (this.pointCloud.points.length > 0 || this.pointCloud.history.length > 0)) {
      this.draw2DPointCloud(ctx);
    }

    // 7. Real LiDAR LaserScan Scan Lines (/scan)
    if (this.layers.lidar && this.laserScan.points.length > 0) {
      this.drawLaserScan(ctx);
    }

    // 8. AMR Robot Body & Turret
    if (this.robot.hasData) {
      this.drawAMRRobot(ctx);
    } else {
      this.drawNoRobotMessage(ctx, w, h);
    }

    // 9. 2D Nav Goal Target Flag / Draft Arrow
    if (this.goalPose) {
      this.drawGoalFlag(ctx, this.goalPose.x, this.goalPose.y);
    }
    if (this.navGoalDraft) {
      this.drawNavGoalDraft(ctx);
    }
    // 10. Overlay HUD Metadata
    this.drawHUDOverlay(ctx, w, h);
  }

  draw2DPointCloud(ctx) {
    ctx.save();
    const now = performance.now();
    const decayMs = this.pointCloud.decaySeconds * 1000;

    if (this.pointCloud.decaySeconds > 0) {
      this.pointCloud.history = this.pointCloud.history.filter(p => now - p.time <= decayMs);
    } else {
      this.pointCloud.history = [];
    }

    const pointsToRender = this.pointCloud.decaySeconds > 0 && this.pointCloud.history.length > 0
      ? this.pointCloud.history
      : this.pointCloud.points;

    const len = pointsToRender.length;
    const isHeightMap = this.pointCloud.colormap === 'height';
    const isIntensityMap = this.pointCloud.colormap === 'intensity';
    const isCyan = !isHeightMap && !isIntensityMap;
    const decayActive = this.pointCloud.decaySeconds > 0;

    if (isCyan) ctx.fillStyle = '#00e5ff';
    if (!decayActive) ctx.globalAlpha = 0.85;

    let lastCol = isCyan ? '#00e5ff' : '';
    let lastAlpha = !decayActive ? 0.85 : -1;

    for (let i = 0; i < len; i++) {
      const p = pointsToRender[i];
      const s = this.toScreen(p.x, p.y);

      if (isHeightMap) {
        const normZ = Math.max(0, Math.min(1, (p.z + 0.2) / 2.4));
        const col = turboColormap(normZ);
        if (col !== lastCol) {
          ctx.fillStyle = col;
          lastCol = col;
        }
      } else if (isIntensityMap) {
        const col = intensityColormap(p.intensity || 50, 0, 100);
        if (col !== lastCol) {
          ctx.fillStyle = col;
          lastCol = col;
        }
      }

      if (decayActive && p.time) {
        const age = now - p.time;
        const alpha = +(Math.max(0.15, 0.85 * (1 - age / decayMs))).toFixed(2);
        if (alpha !== lastAlpha) {
          ctx.globalAlpha = alpha;
          lastAlpha = alpha;
        }
      }

      ctx.fillRect(s.x - 1.5, s.y - 1.5, 3, 3);
    }
    ctx.restore();
  }

  draw3DOrbitScene(ctx, w, h) {
    if (this.autoCenter && this.robot.hasData) {
      this.orbitCamera.setTarget(this.robot.x, this.robot.y, 0.4);
    }

    const cam = this.orbitCamera;
    const now = performance.now();
    const decayMs = this.pointCloud.decaySeconds * 1000;

    // 1. Draw 3D Ground Grid (Z = 0)
    ctx.save();
    ctx.lineWidth = 1;
    for (let gx = -6; gx <= 6; gx += 1) {
      const p1 = cam.project(gx, -6, 0, w, h);
      const p2 = cam.project(gx, 6, 0, w, h);
      if (p1.visible && p2.visible) {
        ctx.strokeStyle = gx === 0 ? 'rgba(0, 229, 255, 0.4)' : 'rgba(255, 255, 255, 0.05)';
        ctx.beginPath();
        ctx.moveTo(p1.sx, p1.sy);
        ctx.lineTo(p2.sx, p2.sy);
        ctx.stroke();
      }
    }
    for (let gy = -6; gy <= 6; gy += 1) {
      const p1 = cam.project(-6, gy, 0, w, h);
      const p2 = cam.project(6, gy, 0, w, h);
      if (p1.visible && p2.visible) {
        ctx.strokeStyle = gy === 0 ? 'rgba(0, 229, 255, 0.4)' : 'rgba(255, 255, 255, 0.05)';
        ctx.beginPath();
        ctx.moveTo(p1.sx, p1.sy);
        ctx.lineTo(p2.sx, p2.sy);
        ctx.stroke();
      }
    }

    // 2. Draw 3D Origin Axes (RGB = XYZ)
    const o = cam.project(0, 0, 0, w, h);
    const axX = cam.project(1.5, 0, 0, w, h);
    const axY = cam.project(0, 1.5, 0, w, h);
    const axZ = cam.project(0, 0, 1.5, w, h);

    if (o.visible) {
      // X Axis (Red - Forward)
      if (axX.visible) {
        ctx.strokeStyle = '#ff4444';
        ctx.lineWidth = 2.5;
        ctx.beginPath(); ctx.moveTo(o.sx, o.sy); ctx.lineTo(axX.sx, axX.sy); ctx.stroke();
      }
      // Y Axis (Green - Left)
      if (axY.visible) {
        ctx.strokeStyle = '#00ff9d';
        ctx.lineWidth = 2.5;
        ctx.beginPath(); ctx.moveTo(o.sx, o.sy); ctx.lineTo(axY.sx, axY.sy); ctx.stroke();
      }
      // Z Axis (Blue - Up)
      if (axZ.visible) {
        ctx.strokeStyle = '#00e5ff';
        ctx.lineWidth = 2.5;
        ctx.beginPath(); ctx.moveTo(o.sx, o.sy); ctx.lineTo(axZ.sx, axZ.sy); ctx.stroke();
      }
    }

    // 3. Draw 3D Perimeter Arena Bounding Box Wireframe & Static Boxes (Only in Mock Mode)
    if (this.mockMode && this.layers.map) {
      ctx.strokeStyle = 'rgba(0, 229, 255, 0.5)';
      ctx.lineWidth = 1.5;
      for (const [[x1, y1], [x2, y2]] of this.staticObstacles.walls) {
        const b1 = cam.project(x1, y1, 0, w, h);
        const b2 = cam.project(x2, y2, 0, w, h);
        const t1 = cam.project(x1, y1, 1.5, w, h);
        const t2 = cam.project(x2, y2, 1.5, w, h);
        if (b1.visible && b2.visible) {
          ctx.beginPath(); ctx.moveTo(b1.sx, b1.sy); ctx.lineTo(b2.sx, b2.sy); ctx.stroke();
        }
        if (t1.visible && t2.visible) {
          ctx.beginPath(); ctx.moveTo(t1.sx, t1.sy); ctx.lineTo(t2.sx, t2.sy); ctx.stroke();
        }
        if (b1.visible && t1.visible) {
          ctx.beginPath(); ctx.moveTo(b1.sx, b1.sy); ctx.lineTo(t1.sx, t1.sy); ctx.stroke();
        }
      }

      // 4. Draw 3D Static Boxes (Pillars & Benches)
      for (const b of this.staticObstacles.boxes) {
        const hw = b.w / 2;
        const hh = b.h / 2;
        const hz = b.name.includes('Pillar') ? 2.0 : 0.45;
        const corners = [
          [b.x - hw, b.y - hh, 0], [b.x + hw, b.y - hh, 0],
          [b.x + hw, b.y + hh, 0], [b.x - hw, b.y + hh, 0],
          [b.x - hw, b.y - hh, hz], [b.x + hw, b.y - hh, hz],
          [b.x + hw, b.y + hh, hz], [b.x - hw, b.y + hh, hz]
        ].map(([cx, cy, cz]) => cam.project(cx, cy, cz, w, h));

        ctx.strokeStyle = 'rgba(0, 229, 255, 0.45)';
        ctx.lineWidth = 1;
        for (let i = 0; i < 4; i++) {
          const next = (i + 1) % 4;
          if (corners[i].visible && corners[next].visible) {
            ctx.beginPath(); ctx.moveTo(corners[i].sx, corners[i].sy); ctx.lineTo(corners[next].sx, corners[next].sy); ctx.stroke();
          }
          if (corners[i + 4].visible && corners[next + 4].visible) {
            ctx.beginPath(); ctx.moveTo(corners[i + 4].sx, corners[i + 4].sy); ctx.lineTo(corners[next + 4].sx, corners[next + 4].sy); ctx.stroke();
          }
          if (corners[i].visible && corners[i + 4].visible) {
            ctx.beginPath(); ctx.moveTo(corners[i].sx, corners[i].sy); ctx.lineTo(corners[i + 4].sx, corners[i + 4].sy); ctx.stroke();
          }
        }
      }
    }

    // 5. Draw 3D Tracked Humans & Proxemics Zones
    if (this.layers.proxemics) {
      for (const human of this.trackedHumans) {
        const hBase = cam.project(human.x, human.y, 0, w, h);
        const hTop = cam.project(human.x, human.y, 1.7, w, h);
        if (hBase.visible && hTop.visible) {
          ctx.strokeStyle = '#ff007f';
          ctx.lineWidth = 3.5;
          ctx.beginPath(); ctx.moveTo(hBase.sx, hBase.sy); ctx.lineTo(hTop.sx, hTop.sy); ctx.stroke();

          ctx.fillStyle = '#ff007f';
          ctx.beginPath(); ctx.arc(hTop.sx, hTop.sy, 5, 0, Math.PI * 2); ctx.fill();

          ctx.strokeStyle = 'rgba(255, 0, 127, 0.35)';
          ctx.lineWidth = 1.5;
          ctx.beginPath();
          for (let a = 0; a <= Math.PI * 2 + 0.1; a += 0.3) {
            const px = human.x + 0.8 * Math.cos(a);
            const py = human.y + 0.8 * Math.sin(a);
            const sp = cam.project(px, py, 0, w, h);
            if (sp.visible) {
              if (a === 0) ctx.moveTo(sp.sx, sp.sy);
              else ctx.lineTo(sp.sx, sp.sy);
            }
          }
          ctx.stroke();
        }
      }
    }

    // 6. Draw High-Fidelity 3D AMR Robot (Chassis, 4 Wheels, LiDAR Turret, Headlights, AR Hologram Tag)
    if (this.robot.hasData) {
      this.draw3DAMRRobot(ctx, cam, w, h);
    }

    // 7. Render 3D PointCloud Points with Depth Scaling & Height Colormap
    if (this.layers.pointcloud) {
      if (this.pointCloud.decaySeconds > 0) {
        this.pointCloud.history = this.pointCloud.history.filter(p => now - p.time <= decayMs);
      } else {
        this.pointCloud.history = [];
      }

      const pointsToRender = this.pointCloud.decaySeconds > 0 && this.pointCloud.history.length > 0
        ? this.pointCloud.history
        : this.pointCloud.points;

      const len = pointsToRender.length;
      const isHeightMap = this.pointCloud.colormap === 'height';
      const isIntensityMap = this.pointCloud.colormap === 'intensity';
      const isCyan = !isHeightMap && !isIntensityMap;
      const decayActive = this.pointCloud.decaySeconds > 0;
      const basePointSize = this.pointCloud.pointSize || 2.5;

      if (isCyan) ctx.fillStyle = '#00e5ff';
      if (!decayActive) ctx.globalAlpha = 0.9;

      const lodStride = len > 35000 ? 2 : 1;
      let lastCol = isCyan ? '#00e5ff' : '';
      let lastAlpha = !decayActive ? 0.9 : -1;

      for (let i = 0; i < len; i += lodStride) {
        const p = pointsToRender[i];
        const proj = cam.project(p.x, p.y, p.z, w, h);
        if (!proj.visible) continue;

        if (isHeightMap) {
          const normZ = Math.max(0, Math.min(1, (p.z + 0.2) / 2.4));
          const col = turboColormap(normZ);
          if (col !== lastCol) {
            ctx.fillStyle = col;
            lastCol = col;
          }
        } else if (isIntensityMap) {
          const col = intensityColormap(p.intensity || 50, 0, 100);
          if (col !== lastCol) {
            ctx.fillStyle = col;
            lastCol = col;
          }
        }

        if (decayActive && p.time) {
          const age = now - p.time;
          const alpha = +(Math.max(0.2, 0.9 * (1 - age / decayMs))).toFixed(2);
          if (alpha !== lastAlpha) {
            ctx.globalAlpha = alpha;
            lastAlpha = alpha;
          }
        }

        const ptSize = Math.max(1.5, Math.min(5.5, basePointSize * (9.0 / proj.depth)));
        ctx.fillRect(proj.sx - ptSize / 2, proj.sy - ptSize / 2, ptSize, ptSize);
      }
    }
    ctx.restore();

    // 8. 3D Orbit HUD Overlay
    this.draw3DHUDOverlay(ctx, w, h);
  }

  draw3DHUDOverlay(ctx, w, h) {
    ctx.save();
    ctx.fillStyle = 'rgba(15, 23, 42, 0.85)';
    ctx.strokeStyle = '#c084fc';
    ctx.lineWidth = 1;
    ctx.fillRect(16, 16, 260, 68);
    ctx.strokeRect(16, 16, 260, 68);

    ctx.fillStyle = '#c084fc';
    ctx.font = 'bold 11px JetBrains Mono, monospace';
    ctx.fillText('🌐 3D POINTCLOUD ORBIT MODE', 26, 36);

    const ptCount = this.pointCloud.decaySeconds > 0 && this.pointCloud.history.length > 0
      ? this.pointCloud.history.length
      : this.pointCloud.points.length;

    ctx.fillStyle = 'var(--neon-cyan)';
    ctx.font = '10px JetBrains Mono, monospace';
    ctx.fillText(`Points: ${ptCount.toLocaleString()} | Colormap: ${this.pointCloud.colormap.toUpperCase()}`, 26, 54);
    ctx.fillText(`Orbit Dist: ${this.orbitCamera.distance.toFixed(1)}m | Pitch: ${(this.orbitCamera.pitch * 180 / Math.PI).toFixed(0)}°`, 26, 70);

    ctx.fillStyle = 'rgba(255, 255, 255, 0.45)';
    ctx.font = '10px JetBrains Mono, monospace';
    ctx.textAlign = 'right';
    ctx.fillText('🖱️ Drag: 3D Orbit | Shift/Right Drag: Pan | Wheel: Zoom', w - 16, h - 14);
    ctx.restore();
  }

  draw3DAMRRobot(ctx, cam, w, h) {
    const rx = this.robot.x;
    const ry = this.robot.y;
    const ryaw = this.robot.yaw;
    const cosY = Math.cos(ryaw);
    const sinY = Math.sin(ryaw);

    ctx.save();

    // 1. Pulsing Ground Halo & Shadow Under Chassis (Z = 0)
    const haloRadius = 0.65;
    ctx.strokeStyle = 'rgba(0, 255, 157, 0.5)';
    ctx.lineWidth = 1.8;
    ctx.beginPath();
    for (let a = 0; a <= Math.PI * 2 + 0.1; a += 0.25) {
      const hx = rx + haloRadius * Math.cos(a);
      const hy = ry + haloRadius * Math.sin(a);
      const hp = cam.project(hx, hy, 0, w, h);
      if (hp.visible) {
        if (a === 0) ctx.moveTo(hp.sx, hp.sy);
        else ctx.lineTo(hp.sx, hp.sy);
      }
    }
    ctx.stroke();

    // 2. High-Beam LED Headlight Cones (projected onto ground Z = 0)
    const hlOrigin = cam.project(rx + 0.28 * cosY, ry + 0.28 * sinY, 0.05, w, h);
    const hlLeft = cam.project(rx + 1.8 * cosY - 0.65 * sinY, ry + 1.8 * sinY + 0.65 * cosY, 0, w, h);
    const hlRight = cam.project(rx + 1.8 * cosY + 0.65 * sinY, ry + 1.8 * sinY - 0.65 * cosY, 0, w, h);

    if (hlOrigin.visible && hlLeft.visible && hlRight.visible) {
      const grad = ctx.createRadialGradient(hlOrigin.sx, hlOrigin.sy, 5, (hlLeft.sx + hlRight.sx) / 2, (hlLeft.sy + hlRight.sy) / 2, 120);
      grad.addColorStop(0, 'rgba(0, 229, 255, 0.35)');
      grad.addColorStop(1, 'rgba(0, 229, 255, 0.0)');
      ctx.fillStyle = grad;
      ctx.beginPath();
      ctx.moveTo(hlOrigin.sx, hlOrigin.sy);
      ctx.lineTo(hlLeft.sx, hlLeft.sy);
      ctx.lineTo(hlRight.sx, hlRight.sy);
      ctx.closePath();
      ctx.fill();
    }

    // Helper: Local robot (lx, ly, lz) to Projected Screen Point
    const transformPt = (lx, ly, lz) => {
      const wx = rx + (lx * cosY - ly * sinY);
      const wy = ry + (lx * sinY + ly * cosY);
      return cam.project(wx, wy, lz, w, h);
    };

    // 3. Four 3D All-Terrain Tires
    const wheelOffsets = [
      { x: 0.17, y: -0.22 },
      { x: 0.17, y: 0.22 },
      { x: -0.17, y: -0.22 },
      { x: -0.17, y: 0.22 }
    ];
    const ww = 0.16, wh = 0.07, wz = 0.14;

    for (const wo of wheelOffsets) {
      const wx1 = wo.x - ww / 2, wx2 = wo.x + ww / 2;
      const wy1 = wo.y - wh / 2, wy2 = wo.y + wh / 2;
      const wz1 = 0.01, wz2 = 0.01 + wz;

      const wPts = [
        transformPt(wx1, wy1, wz1), transformPt(wx2, wy1, wz1),
        transformPt(wx2, wy2, wz1), transformPt(wx1, wy2, wz1),
        transformPt(wx1, wy1, wz2), transformPt(wx2, wy1, wz2),
        transformPt(wx2, wy2, wz2), transformPt(wx1, wy2, wz2)
      ];

      ctx.fillStyle = '#0f172a';
      ctx.strokeStyle = '#00e5ff';
      ctx.lineWidth = 1.2;

      // Top face
      if (wPts[4].visible && wPts[5].visible && wPts[6].visible && wPts[7].visible) {
        ctx.beginPath();
        ctx.moveTo(wPts[4].sx, wPts[4].sy);
        ctx.lineTo(wPts[5].sx, wPts[5].sy);
        ctx.lineTo(wPts[6].sx, wPts[6].sy);
        ctx.lineTo(wPts[7].sx, wPts[7].sy);
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
      }

      // Outer side face
      const sIdx = wo.y < 0 ? [0, 1, 5, 4] : [2, 3, 7, 6];
      if (wPts[sIdx[0]].visible && wPts[sIdx[1]].visible && wPts[sIdx[2]].visible && wPts[sIdx[3]].visible) {
        ctx.fillStyle = '#1e293b';
        ctx.beginPath();
        ctx.moveTo(wPts[sIdx[0]].sx, wPts[sIdx[0]].sy);
        ctx.lineTo(wPts[sIdx[1]].sx, wPts[sIdx[1]].sy);
        ctx.lineTo(wPts[sIdx[2]].sx, wPts[sIdx[2]].sy);
        ctx.lineTo(wPts[sIdx[3]].sx, wPts[sIdx[3]].sy);
        ctx.closePath();
        ctx.fill();
        ctx.stroke();

        const centerHub = transformPt(wo.x, wo.y + (wo.y < 0 ? -wh / 2 : wh / 2), wz / 2 + 0.01);
        if (centerHub.visible) {
          ctx.fillStyle = '#00e5ff';
          ctx.beginPath();
          ctx.arc(centerHub.sx, centerHub.sy, 2.5, 0, Math.PI * 2);
          ctx.fill();
        }
      }
    }

    // 4. Solid Metallic 3D Chassis (0.50m x 0.36m x 0.20m, Z in [0.07, 0.27])
    const cx1 = -0.25, cx2 = 0.25;
    const cy1 = -0.17, cy2 = 0.17;
    const cz1 = 0.07, cz2 = 0.27;

    const cPts = [
      transformPt(cx1, cy1, cz1), transformPt(cx2, cy1, cz1),
      transformPt(cx2, cy2, cz1), transformPt(cx1, cy2, cz1),
      transformPt(cx1, cy1, cz2), transformPt(cx2, cy1, cz2),
      transformPt(cx2, cy2, cz2), transformPt(cx1, cy2, cz2)
    ];

    // Side faces (Dark Metallic Slate)
    const faces = [
      { idx: [0, 1, 5, 4], fill: 'rgba(15, 23, 42, 0.95)' }, // Right
      { idx: [1, 2, 6, 5], fill: 'rgba(30, 41, 59, 0.98)' }, // Front
      { idx: [2, 3, 7, 6], fill: 'rgba(15, 23, 42, 0.95)' }, // Left
      { idx: [3, 0, 4, 7], fill: 'rgba(10, 16, 26, 0.95)' }, // Back
      { idx: [4, 5, 6, 7], fill: 'rgba(24, 34, 53, 0.96)' }  // Top Deck
    ];

    for (const f of faces) {
      const p0 = cPts[f.idx[0]], p1 = cPts[f.idx[1]], p2 = cPts[f.idx[2]], p3 = cPts[f.idx[3]];
      if (p0.visible && p1.visible && p2.visible && p3.visible) {
        ctx.fillStyle = f.fill;
        ctx.strokeStyle = 'var(--neon-green)';
        ctx.lineWidth = 1.8;
        ctx.beginPath();
        ctx.moveTo(p0.sx, p0.sy);
        ctx.lineTo(p1.sx, p1.sy);
        ctx.lineTo(p2.sx, p2.sy);
        ctx.lineTo(p3.sx, p3.sy);
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
      }
    }

    // Front Bumper Dual LED Headlights
    const ledL = transformPt(0.25, -0.11, 0.18);
    const ledR = transformPt(0.25, 0.11, 0.18);
    if (ledL.visible) {
      ctx.fillStyle = '#ffffff';
      ctx.shadowColor = '#00e5ff';
      ctx.shadowBlur = 8;
      ctx.beginPath(); ctx.arc(ledL.sx, ledL.sy, 3.5, 0, Math.PI * 2); ctx.fill();
    }
    if (ledR.visible) {
      ctx.fillStyle = '#ffffff';
      ctx.shadowColor = '#00e5ff';
      ctx.shadowBlur = 8;
      ctx.beginPath(); ctx.arc(ledR.sx, ledR.sy, 3.5, 0, Math.PI * 2); ctx.fill();
    }
    ctx.shadowBlur = 0;

    // Top Deck High-Contrast Heading Arrow
    const topArrowTail = transformPt(-0.15, 0, cz2 + 0.005);
    const topArrowNose = transformPt(0.18, 0, cz2 + 0.005);
    const topArrowL = transformPt(0.06, -0.09, cz2 + 0.005);
    const topArrowR = transformPt(0.06, 0.09, cz2 + 0.005);

    if (topArrowTail.visible && topArrowNose.visible && topArrowL.visible && topArrowR.visible) {
      ctx.fillStyle = '#00ff9d';
      ctx.strokeStyle = '#00ff9d';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(topArrowTail.sx, topArrowTail.sy);
      ctx.lineTo(topArrowNose.sx, topArrowNose.sy);
      ctx.moveTo(topArrowL.sx, topArrowL.sy);
      ctx.lineTo(topArrowNose.sx, topArrowNose.sy);
      ctx.lineTo(topArrowR.sx, topArrowR.sy);
      ctx.stroke();
    }

    // 5. Elevated 3D LiDAR Sensor Turret & Spinning Laser Indicator
    const mastBase = transformPt(0.0, 0.0, cz2);
    const mastTop = transformPt(0.0, 0.0, 0.36);
    if (mastBase.visible && mastTop.visible) {
      ctx.strokeStyle = '#64748b';
      ctx.lineWidth = 4;
      ctx.beginPath(); ctx.moveTo(mastBase.sx, mastBase.sy); ctx.lineTo(mastTop.sx, mastTop.sy); ctx.stroke();
    }

    // Solid LiDAR Puck Cylinder (radius = 0.09m, Z in [0.36, 0.44])
    const puckPts = [];
    const numPuckSteps = 12;
    for (let i = 0; i < numPuckSteps; i++) {
      const a = (i / numPuckSteps) * Math.PI * 2;
      puckPts.push(transformPt(0.09 * Math.cos(a), 0.09 * Math.sin(a), 0.44));
    }

    ctx.fillStyle = '#0f172a';
    ctx.strokeStyle = '#00e5ff';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    for (let i = 0; i < puckPts.length; i++) {
      if (puckPts[i].visible) {
        if (i === 0) ctx.moveTo(puckPts[i].sx, puckPts[i].sy);
        else ctx.lineTo(puckPts[i].sx, puckPts[i].sy);
      }
    }
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    // 3D Spinning Laser Sweep Indicator
    const spinAngle = (performance.now() * 0.008) % (Math.PI * 2);
    const puckCenter = transformPt(0, 0, 0.40);
    const laserTip = transformPt(1.2 * Math.cos(spinAngle), 1.2 * Math.sin(spinAngle), 0.40);
    if (puckCenter.visible && laserTip.visible) {
      ctx.strokeStyle = 'rgba(0, 229, 255, 0.6)';
      ctx.lineWidth = 1.5;
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      ctx.moveTo(puckCenter.sx, puckCenter.sy);
      ctx.lineTo(laserTip.sx, laserTip.sy);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // 6. Floating Holographic AR Tag Above Robot
    const holoTether = transformPt(0.0, 0.0, 0.70);
    if (mastTop.visible && holoTether.visible) {
      ctx.strokeStyle = 'rgba(192, 132, 252, 0.4)';
      ctx.lineWidth = 1;
      ctx.setLineDash([2, 2]);
      ctx.beginPath();
      ctx.moveTo(mastTop.sx, mastTop.sy);
      ctx.lineTo(holoTether.sx, holoTether.sy);
      ctx.stroke();
      ctx.setLineDash([]);

      const bw = 130, bh = 22;
      const bx = holoTether.sx - bw / 2;
      const by = holoTether.sy - bh;

      ctx.fillStyle = 'rgba(10, 16, 26, 0.9)';
      ctx.strokeStyle = '#c084fc';
      ctx.lineWidth = 1;
      ctx.fillRect(bx, by, bw, bh);
      ctx.strokeRect(bx, by, bw, bh);

      ctx.fillStyle = '#00ff9d';
      ctx.font = 'bold 9.5px JetBrains Mono, monospace';
      ctx.textAlign = 'center';
      ctx.fillText(`🤖 JACKAL AMR [${(this.robot.linearV).toFixed(2)}m/s]`, holoTether.sx, by + 14);
    }

    ctx.restore();
  }

  drawMetricGrid(ctx, w, h) {
    const stepMeters = 1.0;
    const stepPx = stepMeters * this.scale;

    const leftWorld = this.cameraX - (w / 2) / this.scale;
    const rightWorld = this.cameraX + (w / 2) / this.scale;
    const topWorld = this.cameraY + (h / 2) / this.scale;
    const bottomWorld = this.cameraY - (h / 2) / this.scale;

    const startX = Math.floor(leftWorld / stepMeters) * stepMeters;
    const endX = Math.ceil(rightWorld / stepMeters) * stepMeters;
    const startY = Math.floor(bottomWorld / stepMeters) * stepMeters;
    const endY = Math.ceil(topWorld / stepMeters) * stepMeters;

    ctx.lineWidth = 1;

    for (let x = startX; x <= endX; x += stepMeters) {
      const sx = this.toScreen(x, 0).x;
      const isMajor = Math.abs(x) < 0.001 || Math.abs(x % 5) < 0.001;
      ctx.strokeStyle = isMajor ? 'rgba(0, 229, 255, 0.18)' : 'rgba(255, 255, 255, 0.035)';
      ctx.beginPath();
      ctx.moveTo(sx, 0);
      ctx.lineTo(sx, h);
      ctx.stroke();

      if (isMajor && this.scale > 18) {
        ctx.fillStyle = 'rgba(0, 229, 255, 0.4)';
        ctx.font = '9px JetBrains Mono, monospace';
        ctx.fillText(`${x.toFixed(0)}m`, sx + 3, h - 6);
      }
    }

    for (let y = startY; y <= endY; y += stepMeters) {
      const sy = this.toScreen(0, y).y;
      const isMajor = Math.abs(y) < 0.001 || Math.abs(y % 5) < 0.001;
      ctx.strokeStyle = isMajor ? 'rgba(0, 229, 255, 0.18)' : 'rgba(255, 255, 255, 0.035)';
      ctx.beginPath();
      ctx.moveTo(0, sy);
      ctx.lineTo(w, sy);
      ctx.stroke();

      if (isMajor && this.scale > 18) {
        ctx.fillStyle = 'rgba(0, 229, 255, 0.4)';
        ctx.font = '9px JetBrains Mono, monospace';
        ctx.fillText(`${y.toFixed(0)}m`, 6, sy - 3);
      }
    }
  }

  drawOccupancyGrid(ctx) {
    const { originX, originY, resolution, width, height, offscreenCanvas } = this.occupancyMap;
    const topLeft = this.toScreen(originX, originY + height * resolution);
    const destW = width * resolution * this.scale;
    const destH = height * resolution * this.scale;

    ctx.save();
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(offscreenCanvas, topLeft.x, topLeft.y, destW, destH);
    ctx.restore();
  }

  drawStaticObstacles(ctx) {
    if (!this.staticObstacles) return;

    ctx.save();

    // 1. Draw Perimeter & Divider Walls
    ctx.strokeStyle = 'rgba(0, 229, 255, 0.75)';
    ctx.lineWidth = 2.5;
    for (const [[x1, y1], [x2, y2]] of this.staticObstacles.walls) {
      const s1 = this.toScreen(x1, y1);
      const s2 = this.toScreen(x2, y2);
      ctx.beginPath();
      ctx.moveTo(s1.x, s1.y);
      ctx.lineTo(s2.x, s2.y);
      ctx.stroke();
    }

    // 2. Draw Rectangular Boxes (Pillars & Benches)
    for (const b of this.staticObstacles.boxes) {
      const s = this.toScreen(b.x, b.y);
      const wPx = b.w * this.scale;
      const hPx = b.h * this.scale;

      ctx.fillStyle = 'rgba(30, 41, 59, 0.85)';
      ctx.strokeStyle = 'rgba(0, 229, 255, 0.8)';
      ctx.lineWidth = 1.5;
      ctx.fillRect(s.x - wPx / 2, s.y - hPx / 2, wPx, hPx);
      ctx.strokeRect(s.x - wPx / 2, s.y - hPx / 2, wPx, hPx);

      ctx.fillStyle = 'rgba(255, 255, 255, 0.7)';
      ctx.font = '8.5px JetBrains Mono, monospace';
      ctx.textAlign = 'center';
      ctx.fillText(b.name, s.x, s.y + 3);
    }

    // 3. Draw Circular Obstacles (Planters & Kiosks)
    for (const c of this.staticObstacles.circles) {
      const s = this.toScreen(c.x, c.y);
      const rPx = c.r * this.scale;

      ctx.fillStyle = 'rgba(15, 30, 35, 0.9)';
      ctx.strokeStyle = 'rgba(0, 255, 157, 0.8)';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.arc(s.x, s.y, rPx, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();

      ctx.fillStyle = 'rgba(0, 255, 157, 0.75)';
      ctx.font = '8px JetBrains Mono, monospace';
      ctx.textAlign = 'center';
      ctx.fillText(c.name, s.x, s.y + 3);
    }

    ctx.restore();
  }

  drawNavPath(ctx, pathPoints, color, width) {
    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    ctx.shadowColor = color;
    ctx.shadowBlur = 8;

    ctx.beginPath();
    for (let i = 0; i < pathPoints.length; i++) {
      const sp = this.toScreen(pathPoints[i].x, pathPoints[i].y);
      if (i === 0) ctx.moveTo(sp.x, sp.y);
      else ctx.lineTo(sp.x, sp.y);
    }
    ctx.stroke();
    ctx.restore();
  }

  drawRobotTrail(ctx) {
    ctx.save();
    ctx.lineWidth = 2.5;
    ctx.lineCap = 'round';
    ctx.strokeStyle = 'rgba(0, 255, 157, 0.5)';
    ctx.shadowColor = 'var(--neon-green)';
    ctx.shadowBlur = 6;

    ctx.beginPath();
    for (let i = 0; i < this.robotTrail.length; i++) {
      const sp = this.toScreen(this.robotTrail[i].x, this.robotTrail[i].y);
      if (i === 0) ctx.moveTo(sp.x, sp.y);
      else ctx.lineTo(sp.x, sp.y);
    }
    ctx.stroke();
    ctx.restore();
  }

  drawLaserScan(ctx) {
    const hits = this.laserScan.hits;
    if (!hits || hits.length === 0) return;

    ctx.save();
    const rScreen = this.toScreen(this.robot.x, this.robot.y);
    const ox = rScreen.x;
    const oy = rScreen.y;

    // 1. Laser Rays (Electric Azure & Cobalt Blue 360° Raycast Fan - exactly like simulator)
    ctx.lineWidth = 1;
    for (let hit of hits) {
      const sp = this.toScreen(hit.x, hit.y);
      ctx.beginPath();
      ctx.moveTo(ox, oy);
      ctx.lineTo(sp.x, sp.y);

      if (hit.type === 'human') {
        // Dynamic Human Encounter: Vibrant Cyan-Blue Ray
        ctx.strokeStyle = 'rgba(0, 210, 255, 0.20)';
      } else if (hit.type === 'obstacle' || hit.type === 'pillar' || hit.type === 'wall' || hit.type === 'boundary') {
        // Static Barrier Encounter: Electric Azure Blue Ray
        ctx.strokeStyle = 'rgba(56, 189, 248, 0.15)';
      } else {
        // Free Space Max-Range: Deep Translucent Cobalt Blue
        ctx.strokeStyle = 'rgba(14, 165, 233, 0.055)';
      }
      ctx.stroke();
    }

    // 2. Laser Point Cloud Hits (Illuminated Blue Return Dots - exactly like simulator)
    for (let hit of hits) {
      if (hit.type !== 'max') {
        const sp = this.toScreen(hit.x, hit.y);
        ctx.beginPath();
        ctx.arc(sp.x, sp.y, 2.5, 0, Math.PI * 2);
        if (hit.type === 'human') {
          // Human Return: Electric Cyan Blue with Soft Glow
          ctx.fillStyle = '#00E5FF';
          ctx.shadowColor = '#00E5FF';
        } else {
          // Obstacle/Wall Return: Vibrant Deep Azure Blue
          ctx.fillStyle = '#0099FF';
          ctx.shadowColor = '#0099FF';
        }
        ctx.shadowBlur = 8;
        ctx.fill();
      }
    }

    ctx.restore();
  }

  drawHumansAndProxemics(ctx) {
    ctx.save();
    for (const h of this.trackedHumans) {
      const sp = this.toScreen(h.x, h.y);

      // 1. Proxemics Ellipses
      if (this.layers.proxemics) {
        // Intimate Zone (0.45m)
        ctx.strokeStyle = 'rgba(255, 85, 85, 0.6)';
        ctx.fillStyle = 'rgba(255, 85, 85, 0.08)';
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.arc(sp.x, sp.y, 0.45 * this.scale, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();

        // Personal Zone (1.20m)
        ctx.strokeStyle = 'rgba(255, 184, 108, 0.4)';
        ctx.fillStyle = 'rgba(255, 184, 108, 0.04)';
        ctx.beginPath();
        ctx.arc(sp.x, sp.y, 1.20 * this.scale, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
      }

      // 2. Human Body Node
      ctx.fillStyle = '#ff79c6';
      ctx.shadowColor = '#ff79c6';
      ctx.shadowBlur = 10;
      ctx.beginPath();
      ctx.arc(sp.x, sp.y, 7, 0, Math.PI * 2);
      ctx.fill();

      // Label
      ctx.fillStyle = '#ffffff';
      ctx.font = '10px JetBrains Mono, monospace';
      ctx.fillText(`P${h.id}`, sp.x - 6, sp.y - 10);
    }
    ctx.restore();
  }

  drawAMRRobot(ctx) {
    ctx.save();
    const sp = this.toScreen(this.robot.x, this.robot.y);
    ctx.translate(sp.x, sp.y);
    ctx.rotate(-this.robot.yaw); // Invert yaw for screen coords

    const robotLen = 0.5 * this.scale;
    const robotWid = 0.38 * this.scale;

    // Chassis Glow & Shadow
    ctx.shadowColor = 'var(--neon-cyan)';
    ctx.shadowBlur = 16;

    // Main Chassis
    ctx.fillStyle = '#0f172a';
    ctx.strokeStyle = 'var(--neon-cyan)';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.roundRect(-robotLen / 2, -robotWid / 2, robotLen, robotWid, 4);
    ctx.fill();
    ctx.stroke();

    // Left & Right Wheels
    ctx.fillStyle = '#334155';
    ctx.fillRect(-robotLen * 0.3, -robotWid / 2 - 3, robotLen * 0.6, 3);
    ctx.fillRect(-robotLen * 0.3, robotWid / 2, robotLen * 0.6, 3);

    // Heading Forward Arrow
    ctx.fillStyle = 'var(--neon-green)';
    ctx.beginPath();
    ctx.moveTo(robotLen * 0.35, 0);
    ctx.lineTo(0, -robotWid * 0.25);
    ctx.lineTo(0, robotWid * 0.25);
    ctx.closePath();
    ctx.fill();

    // Central LiDAR Turret
    ctx.fillStyle = '#0284c7';
    ctx.beginPath();
    ctx.arc(0, 0, 4, 0, Math.PI * 2);
    ctx.fill();

    ctx.restore();
  }

  drawNoRobotMessage(ctx, w, h) {
    ctx.save();
    ctx.fillStyle = 'rgba(255, 255, 255, 0.4)';
    ctx.font = '13px JetBrains Mono, monospace';
    ctx.textAlign = 'center';
    ctx.fillText('📡 Awaiting live /odom or /robot_pose stream from ROS2 bridge...', w / 2, h / 2);
    ctx.restore();
  }

  drawGoalFlag(ctx, x, y) {
    const sp = this.toScreen(x, y);
    ctx.save();
    ctx.fillStyle = 'var(--neon-green)';
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 2;
    ctx.shadowColor = 'var(--neon-green)';
    ctx.shadowBlur = 12;

    ctx.beginPath();
    ctx.arc(sp.x, sp.y, 6, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    ctx.font = '10px JetBrains Mono, monospace';
    ctx.fillStyle = 'var(--neon-green)';
    ctx.fillText('GOAL', sp.x + 8, sp.y - 4);
    ctx.restore();
  }

  drawNavGoalDraft(ctx) {
    const s1 = this.toScreen(this.navGoalDraft.startX, this.navGoalDraft.startY);
    const s2 = this.toScreen(this.navGoalDraft.currentX, this.navGoalDraft.currentY);

    ctx.save();
    ctx.strokeStyle = 'var(--neon-green)';
    ctx.lineWidth = 2;
    ctx.setLineDash([4, 4]);

    ctx.beginPath();
    ctx.moveTo(s1.x, s1.y);
    ctx.lineTo(s2.x, s2.y);
    ctx.stroke();

    ctx.fillStyle = 'var(--neon-green)';
    ctx.beginPath();
    ctx.arc(s1.x, s1.y, 5, 0, Math.PI * 2);
    ctx.fill();
    ctx.restore();
  }

  drawHUDOverlay(ctx, w, h) {
    ctx.save();
    ctx.font = '11px JetBrains Mono, monospace';
    ctx.fillStyle = 'rgba(255, 255, 255, 0.6)';

    // Coordinate & Telemetry stats at top left
    ctx.fillText(`ROBOT POSE: X: ${this.robot.x.toFixed(2)}m | Y: ${this.robot.y.toFixed(2)}m | YAW: ${(this.robot.yaw * 180 / Math.PI).toFixed(1)}°`, 16, 24);
    ctx.fillText(`LIDAR CONTACTS: ${this.laserScan.points.length} pts | TRACKED HUMANS: ${this.trackedHumans.length}`, 16, 40);

    // Scale Ruler at bottom right
    const rulerMeters = 5.0;
    const rulerPx = rulerMeters * this.scale;
    const rx = w - rulerPx - 70;
    const ry = h - 20;

    ctx.strokeStyle = 'rgba(0, 229, 255, 0.7)';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(rx, ry);
    ctx.lineTo(rx + rulerPx, ry);
    ctx.moveTo(rx, ry - 4);
    ctx.lineTo(rx, ry + 4);
    ctx.moveTo(rx + rulerPx, ry - 4);
    ctx.lineTo(rx + rulerPx, ry + 4);
    ctx.stroke();

    ctx.fillStyle = 'var(--neon-cyan)';
    ctx.textAlign = 'center';
    ctx.fillText(`${rulerMeters}m`, rx + rulerPx / 2, ry - 6);
    ctx.restore();
  }
}

// Standalone Self-Contained Helpers for live.html
function initBackgroundCanvas() {
  const canvas = document.getElementById('bg-canvas');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');

  let width = (canvas.width = window.innerWidth);
  let height = (canvas.height = window.innerHeight);

  window.addEventListener('resize', () => {
    width = canvas.width = window.innerWidth;
    height = canvas.height = window.innerHeight;
  });

  const particles = Array.from({ length: 45 }, () => ({
    x: Math.random() * width,
    y: Math.random() * height,
    vx: (Math.random() - 0.5) * 0.35,
    vy: (Math.random() - 0.5) * 0.35,
    size: Math.random() * 1.8 + 0.6,
    alpha: Math.random() * 0.35 + 0.08
  }));

  function animate() {
    ctx.clearRect(0, 0, width, height);
    for (let p of particles) {
      p.x += p.vx;
      p.y += p.vy;
      if (p.x < 0) p.x = width;
      if (p.x > width) p.x = 0;
      if (p.y < 0) p.y = height;
      if (p.y > height) p.y = 0;
      ctx.beginPath();
      ctx.arc(p.x, p.y, p.size, 0, Math.PI * 2);
      ctx.fillStyle = `rgba(0, 229, 255, ${p.alpha})`;
      ctx.fill();
    }
    requestAnimationFrame(animate);
  }
  animate();
}

function initNavbarBrand() {
  const brandLogo = document.querySelector('.nav-brand') || document.getElementById('header-brand-logo');
  if (brandLogo) {
    brandLogo.addEventListener('click', (e) => {
      e.preventDefault();
      window.scrollTo({ top: 0, left: 0, behavior: 'smooth' });
    });
  }
}

function initThemeSwitcher() {
  const fabBtn = document.getElementById('theme-fab-btn');
  const popover = document.getElementById('theme-popover');
  const closeBtn = document.getElementById('btn-close-theme-popover');
  const themeBtns = document.querySelectorAll('.theme-option-btn');
  const activeBadge = document.getElementById('theme-active-badge');

  function applyTheme(themeId) {
    document.documentElement.setAttribute('data-theme', themeId);
    localStorage.setItem('socialnav_studio_theme', themeId);
    if (activeBadge) activeBadge.textContent = themeId;
  }

  const saved = localStorage.getItem('socialnav_studio_theme') || 'dracula_vampire';
  applyTheme(saved);

  fabBtn?.addEventListener('click', (e) => {
    e.stopPropagation();
    popover?.classList.toggle('open');
  });

  closeBtn?.addEventListener('click', (e) => {
    e.stopPropagation();
    popover?.classList.remove('open');
  });

  themeBtns.forEach(btn => {
    btn.addEventListener('click', () => {
      const themeId = btn.getAttribute('data-theme');
      applyTheme(themeId);
      popover?.classList.remove('open');
    });
  });

  document.addEventListener('click', (e) => {
    if (popover && !popover.contains(e.target) && e.target !== fabBtn && !fabBtn?.contains(e.target)) {
      popover.classList.remove('open');
    }
  });
}

// Instantiate and initialize on DOM ready
document.addEventListener('DOMContentLoaded', () => {
  initBackgroundCanvas();
  initNavbarBrand();
  initThemeSwitcher();

  const manager = new LiveStreamManager();
  manager.init();
  window.liveStreamManager = manager;
});


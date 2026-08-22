// Social Navigation 2D Physics & Proxemics Canvas Engine
// Supports SFM, SARL (Graph DRL), CADRL, Social MPC, Social-ORCA, and Non-Social A*
import { ros2BridgeInstance } from './ros2Bridge.js';
import { telemetryAnalytics } from './analyticsCharts.js';

export class SocialNavSimulator {
  constructor(canvasId) {
    this.canvas = document.getElementById(canvasId);
    if (!this.canvas) return;
    this.ctx = this.canvas.getContext('2d');
    
    // Config & Metric Scaling (1 meter = 40 pixels)
    this.scale = 40;
    this.algorithm = 'sfm'; // 'sfm' | 'drl' | 'cadrl' | 'social_mpc' | 'orca_social' | 'nonsocial'
    this.pedestrianCount = 7;
    this.robotMaxSpeed = 1.2; // m/s
    this.courtesyWeight = 0.8;
    this.proxemicRadius = 1.2; // meters
    
    // Visual Layer Toggles
    this.showHeatmap = true;
    this.showCostmap = false;
    this.costmapResolution = 0.2; // 20cm per cell (5 cells/m)
    this.costmapData = null;
    this.costmapInfo = null;
    this.onCostmapUpdate = null;
    this.lastCostmapPublishTime = 0;
    this.showVectors = true;
    this.showLidar = true;
    this.showLidarRays = true;
    this.showLidarPoints = true;
    this.lidarRays = 360; // Configurable: 18 to 720 rays
    this.lidarMaxRangeM = 6.0; // Configurable: 1.0m to 15.0m
    this.lidarFovDeg = 360; // Configurable: 120, 180, 270, 360 deg
    this.showTrajectory = true;

    // Simulation Execution Control
    this.isPaused = false;
    this.activeTool = 'drag'; // 'drag' | 'spawn_ped' | 'add_pillar' | 'set_goal' | 'set_robot'

    // Robot State
    this.robot = {
      x: 80,
      y: 260,
      vx: 0,
      vy: 0,
      radius: 13,
      heading: 0,
      targetHeading: 0,
      history: []
    };

    // Goal Navigation State ('single' | 'multi')
    this.goalMode = 'single';
    this.goal = {
      x: 720,
      y: 260,
      radius: 14,
      isDragging: false
    };
    this.waypoints = []; // Sequential waypoints: [ { id, x, y, radius: 14 } ]
    this.activeWaypointIndex = 0;
    this.waypointLoop = true;
    this.draggedWaypoint = null;
    this.goalReachedDist = 28; // ~0.7m arrival tolerance

    // Entities & Environmental Sensors
    this.obstacles = [];
    this.pedestrians = [];
    this.laserHits = [];
    this.laserScanData = null;

    // Metrics & Benchmark Evaluation State
    this.violationsCount = 0;
    this.totalSteps = 0;
    this.minDistanceToHuman = 999;
    this.complianceScore = 98.4;
    this.comfortIndex = 95.0;

    // Dragging & Interaction Interaction
    this.isDraggingGoal = false;
    this.isDraggingRobot = false;
    this.draggedObstacle = null;
    this.draggedPedestrian = null;
    this.goalPulse = 0;
    this.mpcHorizonWaypoints = [];

    // Rotating Interaction State
    this.isRotatingRobot = false;
    this.rotatingPedestrian = null;
    this.rotatingObstacle = null;
    this.lastRotateAngle = 0;

    // Custom Object Configuration ('circle' | 'rect' | 'poly' | 'draw_poly')
    this.customObjectConfig = {
      type: 'circle',
      radius: 22,
      width: 60,
      height: 30
    };

    // Custom Polygon Freehand / Click-to-Draw State
    this.isDrawingCustomPolygon = false;
    this.customPolygonDraftPoints = [];
    this.draftCursorPos = null;
    this.onDraftPointsUpdated = null;

    // Benchmark Scenarios: 'synthetic' | 'eth_univ' | 'ucy_zara' | 'bottleneck' | 'doorway' | 'custom'
    this.currentScenario = 'synthetic';
    this.customDatasetFrames = null;
    this.datasetFrameIndex = 0;

    this.initCanvasSize();
    this.initWorld();
    this.bindEvents();
    this.startLoop();
  }

  initCanvasSize() {
    const parent = this.canvas.parentElement;
    if (!parent) return;
    const rect = parent.getBoundingClientRect();
    this.canvas.width = Math.max(600, rect.width || 800);
    this.canvas.height = Math.max(400, rect.height || 500);
    
    if (this.goal.x > this.canvas.width - 60) {
      this.goal.x = this.canvas.width - 80;
    }
  }

  initWorld() {
    this.pedestrians = [];
    this.violationsCount = 0;
    this.totalSteps = 0;
    this.robot.x = 80;
    this.robot.y = this.canvas.height / 2;
    this.robot.vx = 0;
    this.robot.vy = 0;
    this.robot.history = [];
    this.goal.x = this.canvas.width - 80;
    this.goal.y = this.canvas.height / 2;

    const w = this.canvas.width;
    const h = this.canvas.height;

    if (this.currentScenario === 'synthetic') {
      // Standard Hallway with Pillars
      this.obstacles = [
        { id: 'obs-1', type: 'circle', x: w * 0.32, y: h * 0.30, radius: 24, label: 'Column A' },
        { id: 'obs-2', type: 'circle', x: w * 0.68, y: h * 0.70, radius: 24, label: 'Column B' },
        { id: 'obs-3', type: 'rect', x: w * 0.48 - 14, y: h * 0.38, width: 28, height: 110, label: 'Divider' }
      ];
      for (let i = 0; i < this.pedestrianCount; i++) {
        this.spawnPedestrian(i);
      }
    } else if (this.currentScenario === 'eth_univ') {
      // ETH Zurich University Benchmark (Diagonal Crossing Streams)
      this.obstacles = [
        { id: 'eth-1', type: 'circle', x: w * 0.25, y: h * 0.22, radius: 26, label: 'Univ Arch' },
        { id: 'eth-2', type: 'circle', x: w * 0.75, y: h * 0.78, radius: 26, label: 'Kiosk' }
      ];
      this.pedestrians = [
        { id: 'ETH_01', x: w * 0.2, y: h * 0.8, vx: 1.2, vy: -0.9, speed: 0.9, radius: 9, heading: -0.65, targetX: w * 0.85, targetY: h * 0.2, color: '#00E5FF', history: [], customWaypoints: [{x: w * 0.2, y: h * 0.8}, {x: w * 0.35, y: h * 0.65}, {x: w * 0.55, y: h * 0.45}, {x: w * 0.70, y: h * 0.30}, {x: w * 0.85, y: h * 0.2}] },
        { id: 'ETH_02', x: w * 0.25, y: h * 0.85, vx: 1.1, vy: -0.85, speed: 0.85, radius: 9, heading: -0.65, targetX: w * 0.9, targetY: h * 0.25, color: '#00E5FF', history: [], customWaypoints: [{x: w * 0.25, y: h * 0.85}, {x: w * 0.40, y: h * 0.70}, {x: w * 0.60, y: h * 0.50}, {x: w * 0.75, y: h * 0.35}, {x: w * 0.9, y: h * 0.25}] },
        { id: 'ETH_03', x: w * 0.85, y: h * 0.15, vx: -1.2, vy: 0.9, speed: 0.9, radius: 9, heading: 2.5, targetX: w * 0.15, targetY: h * 0.85, color: '#FFB800', history: [], customWaypoints: [{x: w * 0.85, y: h * 0.15}, {x: w * 0.70, y: h * 0.30}, {x: w * 0.50, y: h * 0.50}, {x: w * 0.30, y: h * 0.70}, {x: w * 0.15, y: h * 0.85}] },
        { id: 'ETH_04', x: w * 0.5, y: h * 0.1, vx: 0.1, vy: 1.1, speed: 0.8, radius: 9, heading: 1.57, targetX: w * 0.5, targetY: h * 0.9, color: '#00FF9D', history: [], customWaypoints: [{x: w * 0.5, y: h * 0.1}, {x: w * 0.5, y: h * 0.35}, {x: w * 0.5, y: h * 0.65}, {x: w * 0.5, y: h * 0.9}] },
        { id: 'ETH_05', x: w * 0.52, y: h * 0.9, vx: -0.1, vy: -1.1, speed: 0.8, radius: 9, heading: -1.57, targetX: w * 0.5, targetY: h * 0.1, color: '#00FF9D', history: [], customWaypoints: [{x: w * 0.52, y: h * 0.9}, {x: w * 0.52, y: h * 0.65}, {x: w * 0.52, y: h * 0.35}, {x: w * 0.52, y: h * 0.1}] },
        { id: 'ETH_06', x: w * 0.7, y: h * 0.4, vx: -1.0, vy: -0.2, speed: 0.75, radius: 9, heading: 3.0, targetX: w * 0.1, targetY: h * 0.45, color: '#B388FF', history: [], customWaypoints: [{x: w * 0.7, y: h * 0.4}, {x: w * 0.5, y: h * 0.42}, {x: w * 0.3, y: h * 0.44}, {x: w * 0.1, y: h * 0.45}] }
      ];
    } else if (this.currentScenario === 'eth_hotel') {
      // ETH Hotel Tram Stop Benchmark (High Density Crossing & Waiting)
      this.obstacles = [
        { id: 'hotel-tram', type: 'rect', x: w * 0.15, y: 15, width: w * 0.7, height: 20, label: 'Tram Station' },
        { id: 'hotel-kiosk', type: 'circle', x: w * 0.5, y: h * 0.75, radius: 24, label: 'Kiosk' }
      ];
      this.pedestrians = [
        { id: 'HT_01', x: w * 0.18, y: h * 0.45, vx: 1.25, vy: 0.05, speed: 0.9, radius: 9, heading: 0.04, targetX: w * 0.88, targetY: h * 0.45, color: '#00E5FF', history: [], customWaypoints: [{x: w * 0.18, y: h * 0.45}, {x: w * 0.35, y: h * 0.45}, {x: w * 0.55, y: h * 0.45}, {x: w * 0.72, y: h * 0.45}, {x: w * 0.88, y: h * 0.45}] },
        { id: 'HT_02', x: w * 0.24, y: h * 0.50, vx: 1.20, vy: -0.02, speed: 0.88, radius: 9, heading: -0.02, targetX: w * 0.85, targetY: h * 0.48, color: '#00E5FF', history: [], customWaypoints: [{x: w * 0.24, y: h * 0.50}, {x: w * 0.40, y: h * 0.49}, {x: w * 0.60, y: h * 0.49}, {x: w * 0.75, y: h * 0.48}, {x: w * 0.85, y: h * 0.48}] },
        { id: 'HT_03', x: w * 0.82, y: h * 0.55, vx: -1.25, vy: -0.05, speed: 0.9, radius: 9, heading: 3.10, targetX: w * 0.12, targetY: h * 0.55, color: '#FFB800', history: [], customWaypoints: [{x: w * 0.82, y: h * 0.55}, {x: w * 0.65, y: h * 0.55}, {x: w * 0.45, y: h * 0.55}, {x: w * 0.28, y: h * 0.55}, {x: w * 0.12, y: h * 0.55}] },
        { id: 'HT_04', x: w * 0.48, y: h * 0.82, vx: 0.05, vy: -1.15, speed: 0.85, radius: 9, heading: -1.53, targetX: w * 0.52, targetY: h * 0.15, color: '#00FF9D', history: [], customWaypoints: [{x: w * 0.48, y: h * 0.82}, {x: w * 0.49, y: h * 0.60}, {x: w * 0.50, y: h * 0.38}, {x: w * 0.52, y: h * 0.15}] },
        { id: 'HT_05', x: w * 0.65, y: h * 0.25, vx: -0.8, vy: 0.8, speed: 0.8, radius: 9, heading: 2.35, targetX: w * 0.25, targetY: h * 0.75, color: '#ff007f', history: [], customWaypoints: [{x: w * 0.65, y: h * 0.25}, {x: w * 0.55, y: h * 0.38}, {x: w * 0.42, y: h * 0.52}, {x: w * 0.25, y: h * 0.75}] }
      ];
    } else if (this.currentScenario === 'sdd_coupa') {
      // Stanford Drone Dataset - Coupa Cafe Campus Plaza (Multi-Directional Diagonal Flow)
      this.obstacles = [
        { id: 'sdd-fountain', type: 'circle', x: w * 0.5, y: h * 0.5, radius: 28, label: 'Plaza Center' },
        { id: 'sdd-bench', type: 'rect', x: w * 0.25, y: h * 0.25, width: 35, height: 16, label: 'Outdoor Table' }
      ];
      this.pedestrians = [
        { id: 'SDD_01', x: w * 0.15, y: h * 0.15, vx: 1.1, vy: 0.8, speed: 0.88, radius: 9, heading: 0.63, targetX: w * 0.85, targetY: h * 0.85, color: '#00E5FF', history: [], customWaypoints: [{x: w * 0.15, y: h * 0.15}, {x: w * 0.35, y: h * 0.35}, {x: w * 0.55, y: h * 0.55}, {x: w * 0.70, y: h * 0.70}, {x: w * 0.85, y: h * 0.85}] },
        { id: 'SDD_02', x: w * 0.85, y: h * 0.85, vx: -1.1, vy: -0.8, speed: 0.88, radius: 9, heading: -2.51, targetX: w * 0.15, targetY: h * 0.15, color: '#FFB800', history: [], customWaypoints: [{x: w * 0.85, y: h * 0.85}, {x: w * 0.70, y: h * 0.70}, {x: w * 0.55, y: h * 0.55}, {x: w * 0.35, y: h * 0.35}, {x: w * 0.15, y: h * 0.15}] },
        { id: 'SDD_03', x: w * 0.15, y: h * 0.85, vx: 1.1, vy: -0.8, speed: 0.88, radius: 9, heading: -0.63, targetX: w * 0.85, targetY: h * 0.15, color: '#00FF9D', history: [], customWaypoints: [{x: w * 0.15, y: h * 0.85}, {x: w * 0.35, y: h * 0.65}, {x: w * 0.55, y: h * 0.45}, {x: w * 0.70, y: h * 0.30}, {x: w * 0.85, y: h * 0.15}] },
        { id: 'SDD_04', x: w * 0.85, y: h * 0.15, vx: -1.1, vy: 0.8, speed: 0.88, radius: 9, heading: 2.51, targetX: w * 0.15, targetY: h * 0.85, color: '#ff007f', history: [], customWaypoints: [{x: w * 0.85, y: h * 0.15}, {x: w * 0.70, y: h * 0.30}, {x: w * 0.55, y: h * 0.45}, {x: w * 0.35, y: h * 0.65}, {x: w * 0.15, y: h * 0.85}] }
      ];
    } else if (this.currentScenario === 'ucy_zara') {
      // UCY Zara-01 Shopping Street Benchmark (Bidirectional Dense Flow)
      this.obstacles = [
        { id: 'zara-1', type: 'rect', x: 0, y: 10, width: w, height: 18, label: 'Storefront N' },
        { id: 'zara-2', type: 'rect', x: 0, y: h - 28, width: w, height: 18, label: 'Storefront S' },
        { id: 'zara-3', type: 'circle', x: w * 0.5, y: h * 0.5, radius: 20, label: 'Display Island' }
      ];
      this.pedestrians = [
        { id: 'ZARA_E1', x: w * 0.15, y: h * 0.35, vx: 1.3, vy: 0.05, speed: 0.95, radius: 9, heading: 0.04, targetX: w * 0.92, targetY: h * 0.38, color: '#00E5FF', history: [], customWaypoints: [{x: w * 0.15, y: h * 0.35}, {x: w * 0.35, y: h * 0.36}, {x: w * 0.55, y: h * 0.36}, {x: w * 0.75, y: h * 0.37}, {x: w * 0.92, y: h * 0.38}] },
        { id: 'ZARA_E2', x: w * 0.22, y: h * 0.32, vx: 1.3, vy: 0.03, speed: 0.95, radius: 9, heading: 0.02, targetX: w * 0.95, targetY: h * 0.35, color: '#00E5FF', history: [], customWaypoints: [{x: w * 0.22, y: h * 0.32}, {x: w * 0.40, y: h * 0.33}, {x: w * 0.60, y: h * 0.33}, {x: w * 0.80, y: h * 0.34}, {x: w * 0.95, y: h * 0.35}] },
        { id: 'ZARA_E3', x: w * 0.10, y: h * 0.40, vx: 1.2, vy: -0.02, speed: 0.9, radius: 9, heading: -0.01, targetX: w * 0.90, targetY: h * 0.42, color: '#00E5FF', history: [], customWaypoints: [{x: w * 0.10, y: h * 0.40}, {x: w * 0.30, y: h * 0.40}, {x: w * 0.50, y: h * 0.41}, {x: w * 0.70, y: h * 0.41}, {x: w * 0.90, y: h * 0.42}] },
        { id: 'ZARA_W1', x: w * 0.85, y: h * 0.65, vx: -1.3, vy: -0.05, speed: 0.95, radius: 9, heading: 3.10, targetX: w * 0.08, targetY: h * 0.62, color: '#FFB800', history: [], customWaypoints: [{x: w * 0.85, y: h * 0.65}, {x: w * 0.65, y: h * 0.64}, {x: w * 0.45, y: h * 0.64}, {x: w * 0.25, y: h * 0.63}, {x: w * 0.08, y: h * 0.62}] },
        { id: 'ZARA_W2', x: w * 0.80, y: h * 0.68, vx: -1.3, vy: -0.02, speed: 0.95, radius: 9, heading: 3.12, targetX: w * 0.05, targetY: h * 0.65, color: '#FFB800', history: [], customWaypoints: [{x: w * 0.80, y: h * 0.68}, {x: w * 0.60, y: h * 0.67}, {x: w * 0.40, y: h * 0.66}, {x: w * 0.20, y: h * 0.66}, {x: w * 0.05, y: h * 0.65}] },
        { id: 'ZARA_W3', x: w * 0.90, y: h * 0.60, vx: -1.2, vy: 0.04, speed: 0.9, radius: 9, heading: 3.08, targetX: w * 0.10, targetY: h * 0.58, color: '#FFB800', history: [], customWaypoints: [{x: w * 0.90, y: h * 0.60}, {x: w * 0.70, y: h * 0.59}, {x: w * 0.50, y: h * 0.59}, {x: w * 0.30, y: h * 0.58}, {x: w * 0.10, y: h * 0.58}] }
      ];
    } else if (this.currentScenario === 'jrdb_quad') {
      // Stanford JRDB (JackRabbot) Outdoor Quad Benchmark
      this.obstacles = [
        { id: 'jrdb-fountain', type: 'circle', x: w * 0.5, y: h * 0.5, radius: 26, label: 'Quad Center' },
        { id: 'jrdb-tree-1', type: 'circle', x: w * 0.22, y: h * 0.25, radius: 18, label: 'Quad Tree A' },
        { id: 'jrdb-tree-2', type: 'circle', x: w * 0.78, y: h * 0.75, radius: 18, label: 'Quad Tree B' }
      ];
      this.pedestrians = [
        // Conversational cluster chatting near center
        { id: 'JR_GRP1', x: w * 0.46, y: h * 0.38, vx: 0.05, vy: -0.02, speed: 0.1, radius: 9, heading: 0.5, targetX: w * 0.48, targetY: h * 0.36, color: '#FFB800', history: [], customWaypoints: [{x: w * 0.46, y: h * 0.38}, {x: w * 0.47, y: h * 0.37}, {x: w * 0.48, y: h * 0.36}] },
        { id: 'JR_GRP2', x: w * 0.54, y: h * 0.36, vx: -0.04, vy: 0.03, speed: 0.1, radius: 9, heading: 2.8, targetX: w * 0.52, targetY: h * 0.38, color: '#FFB800', history: [], customWaypoints: [{x: w * 0.54, y: h * 0.36}, {x: w * 0.53, y: h * 0.37}, {x: w * 0.52, y: h * 0.38}] },
        { id: 'JR_GRP3', x: w * 0.50, y: h * 0.30, vx: 0.02, vy: 0.04, speed: 0.1, radius: 9, heading: -1.4, targetX: w * 0.50, targetY: h * 0.32, color: '#FFB800', history: [], customWaypoints: [{x: w * 0.50, y: h * 0.30}, {x: w * 0.50, y: h * 0.31}, {x: w * 0.50, y: h * 0.32}] },
        // Diagonal student streams
        { id: 'JR_STR1', x: w * 0.15, y: h * 0.80, vx: 1.2, vy: -0.7, speed: 0.95, radius: 9, heading: -0.55, targetX: w * 0.85, targetY: h * 0.25, color: '#00E5FF', history: [], customWaypoints: [{x: w * 0.15, y: h * 0.80}, {x: w * 0.35, y: h * 0.65}, {x: w * 0.55, y: h * 0.50}, {x: w * 0.70, y: h * 0.35}, {x: w * 0.85, y: h * 0.25}] },
        { id: 'JR_STR2', x: w * 0.20, y: h * 0.85, vx: 1.15, vy: -0.68, speed: 0.92, radius: 9, heading: -0.55, targetX: w * 0.90, targetY: h * 0.30, color: '#00E5FF', history: [], customWaypoints: [{x: w * 0.20, y: h * 0.85}, {x: w * 0.40, y: h * 0.70}, {x: w * 0.60, y: h * 0.55}, {x: w * 0.75, y: h * 0.40}, {x: w * 0.90, y: h * 0.30}] },
        { id: 'JR_STR3', x: w * 0.82, y: h * 0.20, vx: -1.15, vy: 0.72, speed: 0.92, radius: 9, heading: 2.58, targetX: w * 0.12, targetY: h * 0.82, color: '#00FF9D', history: [], customWaypoints: [{x: w * 0.82, y: h * 0.20}, {x: w * 0.65, y: h * 0.35}, {x: w * 0.45, y: h * 0.50}, {x: w * 0.28, y: h * 0.68}, {x: w * 0.12, y: h * 0.82}] },
        // Fast Skateboarder / Runner
        { id: 'JR_SKATE', x: w * 0.10, y: h * 0.60, vx: 2.2, vy: -0.3, speed: 1.6, radius: 10, heading: -0.15, targetX: w * 0.95, targetY: h * 0.45, color: '#FF007F', history: [], customWaypoints: [{x: w * 0.10, y: h * 0.60}, {x: w * 0.30, y: h * 0.56}, {x: w * 0.50, y: h * 0.52}, {x: w * 0.75, y: h * 0.48}, {x: w * 0.95, y: h * 0.45}] }
      ];
    } else if (this.currentScenario === 'jrdb_atrium') {
      // Stanford JRDB Indoor Gates CS Atrium Benchmark
      this.obstacles = [
        { id: 'atrium-col-1', type: 'circle', x: w * 0.35, y: h * 0.35, radius: 22, label: 'Atrium Column 1' },
        { id: 'atrium-col-2', type: 'circle', x: w * 0.65, y: h * 0.65, radius: 22, label: 'Atrium Column 2' },
        { id: 'atrium-bench', type: 'rect', x: w * 0.45 - 20, y: h * 0.5 - 10, width: 40, height: 20, label: 'Lounge Sofa' }
      ];
      this.pedestrians = [
        { id: 'JR_IN1', x: w * 0.15, y: h * 0.20, vx: 1.1, vy: 0.6, speed: 0.85, radius: 9, heading: 0.5, targetX: w * 0.85, targetY: h * 0.75, color: '#00E5FF', history: [], customWaypoints: [{x: w * 0.15, y: h * 0.20}, {x: w * 0.38, y: h * 0.38}, {x: w * 0.62, y: h * 0.58}, {x: w * 0.85, y: h * 0.75}] },
        { id: 'JR_IN2', x: w * 0.85, y: h * 0.30, vx: -1.05, vy: 0.4, speed: 0.82, radius: 9, heading: 2.8, targetX: w * 0.15, targetY: h * 0.70, color: '#00FF9D', history: [], customWaypoints: [{x: w * 0.85, y: h * 0.30}, {x: w * 0.60, y: h * 0.45}, {x: w * 0.35, y: h * 0.58}, {x: w * 0.15, y: h * 0.70}] },
        { id: 'JR_IN3', x: w * 0.50, y: h * 0.85, vx: -0.1, vy: -1.1, speed: 0.85, radius: 9, heading: -1.57, targetX: w * 0.50, targetY: h * 0.15, color: '#FFB800', history: [], customWaypoints: [{x: w * 0.50, y: h * 0.85}, {x: w * 0.50, y: h * 0.60}, {x: w * 0.50, y: h * 0.38}, {x: w * 0.50, y: h * 0.15}] },
        { id: 'JR_IN4', x: w * 0.25, y: h * 0.70, vx: 0.9, vy: -0.7, speed: 0.85, radius: 9, heading: -0.65, targetX: w * 0.80, targetY: h * 0.20, color: '#B388FF', history: [], customWaypoints: [{x: w * 0.25, y: h * 0.70}, {x: w * 0.45, y: h * 0.52}, {x: w * 0.65, y: h * 0.35}, {x: w * 0.80, y: h * 0.20}] }
      ];
    } else if (this.currentScenario === 'thor_mocap') {
      // University of Lincoln THÖR Shared Space Laboratory
      this.obstacles = [
        { id: 'thor-bench-1', type: 'rect', x: 0, y: 12, width: w, height: 20, label: 'Lab Bench North' },
        { id: 'thor-bench-2', type: 'rect', x: 0, y: h - 32, width: w, height: 20, label: 'Lab Bench South' },
        { id: 'thor-station', type: 'circle', x: w * 0.5, y: h * 0.5, radius: 24, label: 'MoCap Rig' }
      ];
      this.pedestrians = [
        { id: 'THOR_01', x: w * 0.20, y: h * 0.38, vx: 1.15, vy: 0.02, speed: 0.85, radius: 9, heading: 0.02, targetX: w * 0.88, targetY: h * 0.40, color: '#00E5FF', history: [], customWaypoints: [{x: w * 0.20, y: h * 0.38}, {x: w * 0.42, y: h * 0.39}, {x: w * 0.65, y: h * 0.39}, {x: w * 0.88, y: h * 0.40}] },
        { id: 'THOR_02', x: w * 0.80, y: h * 0.62, vx: -1.15, vy: -0.02, speed: 0.85, radius: 9, heading: 3.12, targetX: w * 0.12, targetY: h * 0.60, color: '#FFB800', history: [], customWaypoints: [{x: w * 0.80, y: h * 0.62}, {x: w * 0.58, y: h * 0.61}, {x: w * 0.35, y: h * 0.61}, {x: w * 0.12, y: h * 0.60}] },
        { id: 'THOR_03', x: w * 0.15, y: h * 0.65, vx: 1.05, vy: -0.05, speed: 0.80, radius: 9, heading: -0.05, targetX: w * 0.85, targetY: h * 0.58, color: '#00FF9D', history: [], customWaypoints: [{x: w * 0.15, y: h * 0.65}, {x: w * 0.38, y: h * 0.63}, {x: w * 0.62, y: h * 0.60}, {x: w * 0.85, y: h * 0.58}] }
      ];
    } else if (this.currentScenario === 'atc_mall') {
      // ATR Kyoto ATC Shopping Center 3D LiDAR Benchmark
      this.obstacles = [
        { id: 'atc-store-n', type: 'rect', x: 0, y: 10, width: w, height: 22, label: 'Store Arcade North' },
        { id: 'atc-store-s', type: 'rect', x: 0, y: h - 32, width: w, height: 22, label: 'Store Arcade South' },
        { id: 'atc-display', type: 'circle', x: w * 0.5, y: h * 0.35, radius: 20, label: 'Promotion Display' }
      ];
      this.pedestrians = [
        // Group walking side-by-side
        { id: 'ATC_G1', x: w * 0.15, y: h * 0.62, vx: 1.0, vy: 0.0, speed: 0.8, radius: 9, heading: 0, targetX: w * 0.90, targetY: h * 0.62, color: '#00E5FF', history: [], customWaypoints: [{x: w * 0.15, y: h * 0.62}, {x: w * 0.40, y: h * 0.62}, {x: w * 0.65, y: h * 0.62}, {x: w * 0.90, y: h * 0.62}] },
        { id: 'ATC_G2', x: w * 0.15, y: h * 0.72, vx: 1.0, vy: 0.0, speed: 0.8, radius: 9, heading: 0, targetX: w * 0.90, targetY: h * 0.72, color: '#00E5FF', history: [], customWaypoints: [{x: w * 0.15, y: h * 0.72}, {x: w * 0.40, y: h * 0.72}, {x: w * 0.65, y: h * 0.72}, {x: w * 0.90, y: h * 0.72}] },
        // Window shopper lingering
        { id: 'ATC_WIN', x: w * 0.45, y: h * 0.24, vx: 0.2, vy: 0.05, speed: 0.2, radius: 9, heading: 0.2, targetX: w * 0.60, targetY: h * 0.24, color: '#FFB800', history: [], customWaypoints: [{x: w * 0.45, y: h * 0.24}, {x: w * 0.50, y: h * 0.23}, {x: w * 0.55, y: h * 0.25}, {x: w * 0.60, y: h * 0.24}] },
        // Fast opposite commuters
        { id: 'ATC_C1', x: w * 0.85, y: h * 0.45, vx: -1.3, vy: -0.02, speed: 1.05, radius: 9, heading: 3.12, targetX: w * 0.08, targetY: h * 0.45, color: '#00FF9D', history: [], customWaypoints: [{x: w * 0.85, y: h * 0.45}, {x: w * 0.60, y: h * 0.45}, {x: w * 0.35, y: h * 0.45}, {x: w * 0.08, y: h * 0.45}] },
        { id: 'ATC_C2', x: w * 0.90, y: h * 0.52, vx: -1.25, vy: 0.03, speed: 1.0, radius: 9, heading: 3.10, targetX: w * 0.12, targetY: h * 0.52, color: '#FF007F', history: [], customWaypoints: [{x: w * 0.90, y: h * 0.52}, {x: w * 0.65, y: h * 0.52}, {x: w * 0.38, y: h * 0.52}, {x: w * 0.12, y: h * 0.52}] }
      ];
    } else if (this.currentScenario === 'scand_plaza') {
      // UT Austin SCAND Robot Plaza Benchmark
      this.obstacles = [
        { id: 'scand-benches', type: 'rect', x: w * 0.25, y: h * 0.25, width: 40, height: 18, label: 'Campus Bench' },
        { id: 'scand-statue', type: 'circle', x: w * 0.5, y: h * 0.5, radius: 24, label: 'Plaza Landmark' }
      ];
      this.pedestrians = [
        { id: 'SCAND_01', x: w * 0.18, y: h * 0.20, vx: 1.05, vy: 0.65, speed: 0.88, radius: 9, heading: 0.55, targetX: w * 0.85, targetY: h * 0.75, color: '#00E5FF', history: [], customWaypoints: [{x: w * 0.18, y: h * 0.20}, {x: w * 0.40, y: h * 0.38}, {x: w * 0.62, y: h * 0.58}, {x: w * 0.85, y: h * 0.75}] },
        { id: 'SCAND_02', x: w * 0.22, y: h * 0.28, vx: 1.02, vy: 0.62, speed: 0.85, radius: 9, heading: 0.55, targetX: w * 0.88, targetY: h * 0.80, color: '#00E5FF', history: [], customWaypoints: [{x: w * 0.22, y: h * 0.28}, {x: w * 0.44, y: h * 0.45}, {x: w * 0.66, y: h * 0.62}, {x: w * 0.88, y: h * 0.80}] },
        { id: 'SCAND_03', x: w * 0.82, y: h * 0.75, vx: -1.05, vy: -0.65, speed: 0.88, radius: 9, heading: -2.55, targetX: w * 0.15, targetY: h * 0.20, color: '#FFB800', history: [], customWaypoints: [{x: w * 0.82, y: h * 0.75}, {x: w * 0.60, y: h * 0.56}, {x: w * 0.38, y: h * 0.38}, {x: w * 0.15, y: h * 0.20}] },
        { id: 'SCAND_04', x: w * 0.50, y: h * 0.82, vx: 0.05, vy: -1.1, speed: 0.85, radius: 9, heading: -1.53, targetX: w * 0.50, targetY: h * 0.15, color: '#00FF9D', history: [], customWaypoints: [{x: w * 0.50, y: h * 0.82}, {x: w * 0.50, y: h * 0.60}, {x: w * 0.50, y: h * 0.38}, {x: w * 0.50, y: h * 0.15}] }
      ];
    } else if (this.currentScenario === 'ind_urban') {
      // RWTH Aachen inD Urban Shared Space Intersection
      this.obstacles = [
        { id: 'ind-curb-nw', type: 'rect', x: 0, y: 0, width: w * 0.35, height: h * 0.32, label: 'Sidewalk NW' },
        { id: 'ind-curb-ne', type: 'rect', x: w * 0.65, y: 0, width: w * 0.35, height: h * 0.32, label: 'Sidewalk NE' },
        { id: 'ind-curb-sw', type: 'rect', x: 0, y: h * 0.68, width: w * 0.35, height: h * 0.32, label: 'Sidewalk SW' },
        { id: 'ind-curb-se', type: 'rect', x: w * 0.65, y: h * 0.68, width: w * 0.35, height: h * 0.32, label: 'Sidewalk SE' },
        { id: 'ind-island', type: 'circle', x: w * 0.5, y: h * 0.5, radius: 18, label: 'Roundabout Center' }
      ];
      this.pedestrians = [
        { id: 'IND_PED1', x: w * 0.42, y: h * 0.10, vx: 0.0, vy: 1.1, speed: 0.85, radius: 9, heading: 1.57, targetX: w * 0.42, targetY: h * 0.90, color: '#00E5FF', history: [], customWaypoints: [{x: w * 0.42, y: h * 0.10}, {x: w * 0.42, y: h * 0.35}, {x: w * 0.42, y: h * 0.65}, {x: w * 0.42, y: h * 0.90}] },
        { id: 'IND_PED2', x: w * 0.58, y: h * 0.90, vx: 0.0, vy: -1.1, speed: 0.85, radius: 9, heading: -1.57, targetX: w * 0.58, targetY: h * 0.10, color: '#FFB800', history: [], customWaypoints: [{x: w * 0.58, y: h * 0.90}, {x: w * 0.58, y: h * 0.65}, {x: w * 0.58, y: h * 0.35}, {x: w * 0.58, y: h * 0.10}] },
        { id: 'IND_BIKE', x: w * 0.10, y: h * 0.45, vx: 1.8, vy: 0.0, speed: 1.4, radius: 10, heading: 0, targetX: w * 0.90, targetY: h * 0.45, color: '#00FF9D', history: [], customWaypoints: [{x: w * 0.10, y: h * 0.45}, {x: w * 0.35, y: h * 0.45}, {x: w * 0.65, y: h * 0.45}, {x: w * 0.90, y: h * 0.45}] },
        { id: 'IND_CROSS', x: w * 0.90, y: h * 0.55, vx: -1.2, vy: 0.0, speed: 0.95, radius: 9, heading: Math.PI, targetX: w * 0.10, targetY: h * 0.55, color: '#FF007F', history: [], customWaypoints: [{x: w * 0.90, y: h * 0.55}, {x: w * 0.65, y: h * 0.55}, {x: w * 0.35, y: h * 0.55}, {x: w * 0.10, y: h * 0.55}] }
      ];
    } else if (this.currentScenario === 'bottleneck') {
      // Narrow Doorway / Hospital Corridor Choke-point
      this.obstacles = [
        { id: 'wall-top', type: 'rect', x: w * 0.5 - 12, y: 0, width: 24, height: h * 0.38, label: 'Wall Top' },
        { id: 'wall-bot', type: 'rect', x: w * 0.5 - 12, y: h * 0.62, width: 24, height: h * 0.38, label: 'Wall Bottom' }
      ];
      this.pedestrians = [
        { id: 'BN_01', x: w * 0.75, y: h * 0.50, vx: -1.1, vy: 0.0, speed: 0.85, radius: 9, heading: Math.PI, targetX: w * 0.15, targetY: h * 0.50, color: '#FF0055', history: [], customWaypoints: [{x: w * 0.75, y: h * 0.50}, {x: w * 0.55, y: h * 0.50}, {x: w * 0.45, y: h * 0.50}, {x: w * 0.15, y: h * 0.50}] },
        { id: 'BN_02', x: w * 0.85, y: h * 0.48, vx: -1.0, vy: 0.02, speed: 0.8, radius: 9, heading: Math.PI, targetX: w * 0.10, targetY: h * 0.52, color: '#FF0055', history: [], customWaypoints: [{x: w * 0.85, y: h * 0.48}, {x: w * 0.60, y: h * 0.49}, {x: w * 0.40, y: h * 0.51}, {x: w * 0.10, y: h * 0.52}] },
        { id: 'BN_03', x: w * 0.20, y: h * 0.52, vx: 1.0, vy: -0.01, speed: 0.8, radius: 9, heading: 0, targetX: w * 0.88, targetY: h * 0.50, color: '#00E5FF', history: [], customWaypoints: [{x: w * 0.20, y: h * 0.52}, {x: w * 0.40, y: h * 0.51}, {x: w * 0.60, y: h * 0.50}, {x: w * 0.88, y: h * 0.50}] }
      ];
    } else if (this.currentScenario === 'doorway') {
      // 90° Corner Corridor with Entering Humans (Yielding & Blind Corner Challenge)
      this.obstacles = [
        { id: 'corner-wall-1', type: 'rect', x: w * 0.4 - 12, y: 0, width: 24, height: h * 0.55, label: 'Corner Wall' },
        { id: 'corner-wall-2', type: 'rect', x: w * 0.4 - 12, y: h * 0.75, width: 24, height: h * 0.25, label: 'Door Wall' },
        { id: 'corner-wall-3', type: 'rect', x: w * 0.4 + 12, y: h * 0.4 - 12, width: w * 0.35, height: 24, label: 'L-Corridor Wall' }
      ];
      this.robot.x = 70;
      this.robot.y = h * 0.25;
      this.goal.x = w * 0.85;
      this.goal.y = h * 0.8;
      this.pedestrians = [
        { id: 'DW_01', x: w * 0.75, y: h * 0.65, vx: -1.1, vy: -0.2, speed: 0.85, radius: 9, heading: Math.PI, targetX: w * 0.15, targetY: h * 0.25, color: '#FF0055', history: [], customWaypoints: [{x: w * 0.75, y: h * 0.65}, {x: w * 0.48, y: h * 0.65}, {x: w * 0.48, y: h * 0.25}, {x: w * 0.15, y: h * 0.25}] },
        { id: 'DW_02', x: w * 0.82, y: h * 0.70, vx: -0.9, vy: 0.1, speed: 0.8, radius: 9, heading: Math.PI, targetX: w * 0.10, targetY: h * 0.30, color: '#00FF9D', history: [], customWaypoints: [{x: w * 0.82, y: h * 0.70}, {x: w * 0.48, y: h * 0.70}, {x: w * 0.48, y: h * 0.30}, {x: w * 0.10, y: h * 0.30}] },
        { id: 'DW_03', x: w * 0.45, y: h * 0.65, vx: 0.2, vy: 0.9, speed: 0.8, radius: 9, heading: Math.PI / 2, targetX: w * 0.45, targetY: h * 0.9, color: '#00E5FF', history: [], customWaypoints: [{x: w * 0.45, y: h * 0.65}, {x: w * 0.45, y: h * 0.78}, {x: w * 0.45, y: h * 0.90}] }
      ];
    }
  }

  loadScenario(scenarioName) {
    this.currentScenario = scenarioName;
    this.initWorld();
  }

  // Load custom trajectory dataset in standard ETH / UCY format (supports both 8-column raw obsmat and 4-column frames)
  loadCustomDataset(textContent) {
    try {
      const lines = textContent.trim().split('\n');
      const parsedTracks = {};
      let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;

      lines.forEach(line => {
        const trimmed = line.trim();
        if (!trimmed || trimmed.startsWith('#') || trimmed.startsWith('//')) return;
        const parts = trimmed.split(/[\s,;\t]+/);
        if (parts.length >= 4) {
          const frame = parseFloat(parts[0]);
          const pedId = parts[1];
          // If 8 columns (raw obsmat.txt format: frame, id, x, z, y, vx, vz, vy), y is at index 4
          const is8Col = parts.length >= 8;
          const x = parseFloat(parts[2]);
          const y = parseFloat(is8Col ? parts[4] : parts[3]);

          if (!isNaN(frame) && !isNaN(x) && !isNaN(y)) {
            if (!parsedTracks[pedId]) parsedTracks[pedId] = [];
            parsedTracks[pedId].push({ frame, x, y });
            if (x < minX) minX = x;
            if (x > maxX) maxX = x;
            if (y < minY) minY = y;
            if (y > maxY) maxY = y;
          }
        }
      });

      const pedIds = Object.keys(parsedTracks);
      if (pedIds.length === 0) {
        throw new Error("No valid trajectory points found in dataset file.");
      }

      this.currentScenario = 'custom';
      this.obstacles = [
        { id: 'custom-1', type: 'circle', x: this.canvas.width * 0.3, y: this.canvas.height * 0.3, radius: 22, label: 'Benchmark Landmark A' },
        { id: 'custom-2', type: 'circle', x: this.canvas.width * 0.7, y: this.canvas.height * 0.7, radius: 22, label: 'Benchmark Landmark B' }
      ];

      const cw = this.canvas.width;
      const ch = this.canvas.height;
      const spanX = Math.max(1, maxX - minX);
      const spanY = Math.max(1, maxY - minY);
      const padX = 60, padY = 50;
      const fitScale = Math.min((cw - 2 * padX) / spanX, (ch - 2 * padY) / spanY);
      const midX = (minX + maxX) / 2;
      const midY = (minY + maxY) / 2;

      const mapCoord = (x, y) => ({
        x: Math.max(25, Math.min(cw - 25, cw / 2 + (x - midX) * fitScale)),
        y: Math.max(25, Math.min(ch - 25, ch / 2 - (y - midY) * fitScale))
      });

      this.pedestrians = pedIds.slice(0, 24).map((id, idx) => {
        const pts = parsedTracks[id].sort((a, b) => a.frame - b.frame);
        const p0 = mapCoord(pts[0].x, pts[0].y);
        const pLast = mapCoord(pts[pts.length - 1].x, pts[pts.length - 1].y);
        const angle = Math.atan2(pLast.y - p0.y, pLast.x - p0.x);
        const colors = ['#00E5FF', '#00FF9D', '#FFB800', '#FF007F', '#B388FF'];

        // Determine frame interval (ETH 25fps vs sampled 2.5fps)
        const frameDiff = pts.length > 1 ? Math.max(1, pts[1].frame - pts[0].frame) : 10;
        const fps = frameDiff >= 5 ? 25.0 : 2.5; // If step is 10, recorded at 25 fps; if step is 1, recorded at 2.5 fps
        const baseFrame = pts[0].frame;

        return {
          id: `BENCH_${id}`,
          x: p0.x,
          y: p0.y,
          vx: Math.cos(angle) * 0.9 * (this.scale / 30),
          vy: Math.sin(angle) * 0.9 * (this.scale / 30),
          speed: 0.9 + (idx % 3) * 0.15,
          radius: 9,
          heading: angle,
          targetX: pLast.x,
          targetY: pLast.y,
          color: colors[idx % colors.length],
          history: [],
          customWaypoints: pts.map(p => mapCoord(p.x, p.y)),
          rawMetricWaypoints: pts.map(p => ({
            frame: p.frame,
            time: +((p.frame - baseFrame) / fps).toFixed(3),
            x: p.x,
            y: p.y
          }))
        };
      });

      return { success: true, pedestrianCount: this.pedestrians.length, totalUniquePeds: pedIds.length };
    } catch (e) {
      return { success: false, error: e.message };
    }
  }

  spawnPedestrian(index = 0, explicitX = null, explicitY = null) {
    const padding = 50;
    let x, y, safe = false, attempts = 0;

    if (explicitX !== null && explicitY !== null) {
      x = explicitX;
      y = explicitY;
      safe = true;
    } else {
      while (!safe && attempts < 20) {
        attempts++;
        x = padding + Math.random() * (this.canvas.width - 2 * padding);
        y = padding + Math.random() * (this.canvas.height - 2 * padding);

        const distRobot = Math.hypot(x - this.robot.x, y - this.robot.y);
        const distGoal = Math.hypot(x - this.goal.x, y - this.goal.y);

        let inObstacle = false;
        for (let obs of this.obstacles) {
          if (obs.type === 'circle' && Math.hypot(x - obs.x, y - obs.y) < obs.radius + 20) inObstacle = true;
          if (obs.type === 'rect' && x > obs.x - 20 && x < obs.x + obs.width + 20 && y > obs.y - 20 && y < obs.y + obs.height + 20) inObstacle = true;
        }

        if (distRobot > 70 && distGoal > 50 && !inObstacle) {
          safe = true;
        }
      }
    }

    if (!safe) return;

    const angle = Math.random() * Math.PI * 2;
    const speed = 0.5 + Math.random() * 0.7; // 0.5 - 1.2 m/s

    this.pedestrians.push({
      id: Math.random().toString(36).substring(7),
      x: x,
      y: y,
      vx: Math.cos(angle) * speed * (this.scale / 30),
      vy: Math.sin(angle) * speed * (this.scale / 30),
      speed: speed,
      radius: 9,
      heading: angle,
      targetX: Math.random() * this.canvas.width,
      targetY: Math.random() * this.canvas.height,
      color: '#00E5FF',
      history: []
    });
  }

  setCustomObjectConfig(config) {
    this.customObjectConfig = { ...this.customObjectConfig, ...config };
  }

  addCustomObject(x = null, y = null, config = null) {
    const cfg = config || this.customObjectConfig;
    const posX = x !== null ? x : (this.canvas.width * 0.5 + (Math.random() - 0.5) * 200);
    const posY = y !== null ? y : (this.canvas.height * 0.5 + (Math.random() - 0.5) * 150);

    let newObs;
    if (cfg.type === 'poly' || cfg.type === 'polygon' || cfg.type === 'random_poly') {
      newObs = this.createRandomGeometry(posX, posY);
    } else if (cfg.type === 'rect') {
      const w = Math.max(10, cfg.width || 60);
      const h = Math.max(10, cfg.height || 30);
      newObs = {
        id: 'obs-' + Math.random().toString(36).substring(7),
        type: 'rect',
        x: posX - w / 2,
        y: posY - h / 2,
        width: w,
        height: h,
        label: `Box ${w}x${h}`
      };
    } else {
      const r = Math.max(5, cfg.radius || 22);
      newObs = {
        id: 'obs-' + Math.random().toString(36).substring(7),
        type: 'circle',
        x: posX,
        y: posY,
        radius: r,
        label: `Pillar R${r}`
      };
    }

    this.obstacles.push(newObs);
    return newObs;
  }

  finishCustomPolygonDrawing() {
    if (!this.customPolygonDraftPoints || this.customPolygonDraftPoints.length < 3) {
      return null;
    }

    const pts = this.customPolygonDraftPoints;
    const len = pts.length;
    let sumX = 0, sumY = 0;
    for (let p of pts) {
      sumX += p.x;
      sumY += p.y;
    }
    const centerX = +(sumX / len).toFixed(2);
    const centerY = +(sumY / len).toFixed(2);

    const relPoints = pts.map(p => ({
      x: +(p.x - centerX).toFixed(2),
      y: +(p.y - centerY).toFixed(2)
    }));

    let maxR = 25;
    for (let p of relPoints) {
      const d = Math.hypot(p.x, p.y);
      if (d > maxR) maxR = d;
    }

    const newObs = {
      id: 'poly-' + Math.random().toString(36).substring(7),
      type: 'polygon',
      x: centerX,
      y: centerY,
      radius: +Math.max(15, maxR).toFixed(2),
      scale: 1.0,
      points: relPoints,
      label: `CUSTOM POLY (${len}V)`
    };

    this.obstacles.push(newObs);
    this.customPolygonDraftPoints = [];
    this.isDrawingCustomPolygon = false;
    this.draftCursorPos = null;

    if (this.onDraftPointsUpdated) {
      this.onDraftPointsUpdated(0);
    }
    if (this.onObjectPlaced) {
      this.onObjectPlaced(newObs);
    }
    return newObs;
  }

  clearCustomPolygonDraft() {
    this.customPolygonDraftPoints = [];
    this.isDrawingCustomPolygon = false;
    this.draftCursorPos = null;
    if (this.onDraftPointsUpdated) {
      this.onDraftPointsUpdated(0);
    }
  }

  drawCustomPolygonDraft() {
    if (!this.customPolygonDraftPoints || this.customPolygonDraftPoints.length === 0) return;

    const pts = this.customPolygonDraftPoints;
    const len = pts.length;
    const curPos = this.draftCursorPos;
    const isLight = ['light', 'solar_light', 'sakura_light', 'mint_light', 'coffee_latte'].includes(
      document.documentElement.getAttribute('data-theme')
    );

    this.ctx.save();

    // 1. Connecting polygon outline
    this.ctx.beginPath();
    this.ctx.moveTo(pts[0].x, pts[0].y);
    for (let i = 1; i < len; i++) {
      this.ctx.lineTo(pts[i].x, pts[i].y);
    }

    // Dynamic guide line to cursor
    let isNearFirst = false;
    if (curPos) {
      if (len >= 3 && Math.hypot(curPos.x - pts[0].x, curPos.y - pts[0].y) < 24) {
        isNearFirst = true;
        this.ctx.lineTo(pts[0].x, pts[0].y);
      } else {
        this.ctx.lineTo(curPos.x, curPos.y);
      }
    }

    if (len >= 3) {
      this.ctx.fillStyle = isLight ? 'rgba(2, 132, 199, 0.12)' : 'rgba(0, 229, 255, 0.15)';
      this.ctx.fill();
    }

    this.ctx.strokeStyle = isLight ? '#0284c7' : '#00E5FF';
    this.ctx.lineWidth = 2.2;
    this.ctx.shadowColor = isLight ? 'rgba(2, 132, 199, 0.4)' : '#00E5FF';
    this.ctx.shadowBlur = 10;
    this.ctx.setLineDash([4, 4]);
    this.ctx.stroke();
    this.ctx.setLineDash([]);

    // 2. Vertex Nodes
    for (let i = 0; i < len; i++) {
      const p = pts[i];
      const isStart = i === 0;

      // Glow Node
      this.ctx.beginPath();
      this.ctx.arc(p.x, p.y, isStart ? 6.5 : 4.5, 0, Math.PI * 2);
      this.ctx.fillStyle = isStart ? '#00FF9D' : '#00E5FF';
      this.ctx.shadowColor = isStart ? '#00FF9D' : '#00E5FF';
      this.ctx.shadowBlur = 12;
      this.ctx.fill();

      // White Center
      this.ctx.beginPath();
      this.ctx.arc(p.x, p.y, 2, 0, Math.PI * 2);
      this.ctx.fillStyle = '#FFFFFF';
      this.ctx.fill();

      // Index Badge
      this.ctx.font = '700 9px "JetBrains Mono", monospace';
      this.ctx.textAlign = 'center';
      this.ctx.textBaseline = 'bottom';
      this.ctx.fillStyle = isStart ? '#00FF9D' : 'rgba(255, 255, 255, 0.85)';
      this.ctx.shadowBlur = 0;
      this.ctx.fillText(`V${i + 1}`, p.x, p.y - 8);
    }

    // 3. Snap to Start Indicator
    if (isNearFirst) {
      const t = Date.now() * 0.005;
      const pulseR = 14 + Math.sin(t) * 4;
      this.ctx.beginPath();
      this.ctx.arc(pts[0].x, pts[0].y, pulseR, 0, Math.PI * 2);
      this.ctx.strokeStyle = '#00FF9D';
      this.ctx.lineWidth = 2;
      this.ctx.shadowColor = '#00FF9D';
      this.ctx.shadowBlur = 14;
      this.ctx.stroke();

      this.ctx.font = '700 9px "JetBrains Mono", monospace';
      this.ctx.textAlign = 'center';
      this.ctx.textBaseline = 'top';
      this.ctx.fillStyle = '#00FF9D';
      this.ctx.fillText('CLOSE POLYGON', pts[0].x, pts[0].y + 14);
    }

    this.ctx.restore();
  }

  addStaticPillar(x = null, y = null) {
    const posX = x !== null ? x : (this.canvas.width * 0.5 + (Math.random() - 0.5) * 200);
    const posY = y !== null ? y : (this.canvas.height * 0.5 + (Math.random() - 0.5) * 150);
    const newObs = {
      id: 'obs-' + Math.random().toString(36).substring(7),
      type: 'circle',
      x: posX,
      y: posY,
      radius: 22,
      label: `Pillar ${this.obstacles.length + 1}`
    };
    this.obstacles.push(newObs);
  }

  addRandomPolygonObstacle(x = null, y = null) {
    const posX = x !== null ? x : (this.canvas.width * 0.5 + (Math.random() - 0.5) * (this.canvas.width * 0.55));
    const posY = y !== null ? y : (this.canvas.height * 0.5 + (Math.random() - 0.5) * (this.canvas.height * 0.45));
    const newObs = this.createRandomGeometry(posX, posY);
    this.obstacles.push(newObs);
    return newObs;
  }

  createRandomGeometry(posX, posY) {
    const types = ['polygon_5', 'polygon_6', 'polygon_7', 'l_shape', 'u_pocket', 'wedge', 'star'];
    const chosenType = types[Math.floor(Math.random() * types.length)];
    let points = [];
    let label = 'POLYGON';

    // Random Size Scale Factor: from compact (0.55x) to grand obstacle (2.2x)
    const scaleFactor = 0.55 + Math.random() * 1.65;

    if (chosenType === 'l_shape') {
      label = 'L-WALL';
      const w = (50 + Math.random() * 25) * scaleFactor;
      const h = (50 + Math.random() * 25) * scaleFactor;
      const t = Math.max(10, (14 + Math.random() * 6) * scaleFactor);
      points = [
        { x: -w / 2, y: -h / 2 },
        { x: -w / 2 + t, y: -h / 2 },
        { x: -w / 2 + t, y: h / 2 - t },
        { x: w / 2, y: h / 2 - t },
        { x: w / 2, y: h / 2 },
        { x: -w / 2, y: h / 2 }
      ];
    } else if (chosenType === 'u_pocket') {
      label = 'U-POCKET';
      const w = (50 + Math.random() * 20) * scaleFactor;
      const h = (42 + Math.random() * 16) * scaleFactor;
      const t = Math.max(10, (13 + Math.random() * 5) * scaleFactor);
      points = [
        { x: -w / 2, y: -h / 2 },
        { x: -w / 2 + t, y: -h / 2 },
        { x: -w / 2 + t, y: h / 2 - t },
        { x: w / 2 - t, y: h / 2 - t },
        { x: w / 2 - t, y: -h / 2 },
        { x: w / 2, y: -h / 2 },
        { x: w / 2, y: h / 2 },
        { x: -w / 2, y: h / 2 }
      ];
    } else if (chosenType === 'wedge') {
      label = 'WEDGE';
      const r = (32 + Math.random() * 18) * scaleFactor;
      const ang = Math.random() * Math.PI * 2;
      points = [
        { x: Math.cos(ang) * r, y: Math.sin(ang) * r },
        { x: Math.cos(ang + 2.0) * (r * 0.85), y: Math.sin(ang + 2.0) * (r * 0.85) },
        { x: Math.cos(ang + 3.14) * (r * 0.4), y: Math.sin(ang + 3.14) * (r * 0.4) },
        { x: Math.cos(ang + 4.6) * (r * 0.9), y: Math.sin(ang + 4.6) * (r * 0.9) }
      ];
    } else if (chosenType === 'star') {
      label = 'STAR-POLY';
      const numPts = 5 + Math.floor(Math.random() * 3);
      const rot = Math.random() * Math.PI * 2;
      for (let i = 0; i < numPts * 2; i++) {
        const a = (i * Math.PI / numPts) + rot;
        const r = ((i % 2 === 0) ? (30 + Math.random() * 12) : (14 + Math.random() * 8)) * scaleFactor;
        points.push({ x: Math.cos(a) * r, y: Math.sin(a) * r });
      }
    } else {
      const numVerts = 5 + Math.floor(Math.random() * 4);
      label = `POLY-${numVerts}`;
      const angles = [];
      for (let i = 0; i < numVerts; i++) {
        angles.push(Math.random() * Math.PI * 2);
      }
      angles.sort((a, b) => a - b);
      const rot = Math.random() * Math.PI * 2;
      points = angles.map(a => {
        const rad = (18 + Math.random() * 26) * scaleFactor;
        return {
          x: Math.cos(a + rot) * rad,
          y: Math.sin(a + rot) * rad
        };
      });
    }

    // Apply random initial orientation rotation
    const initRot = Math.random() * Math.PI * 2;
    const cosR = Math.cos(initRot);
    const sinR = Math.sin(initRot);
    for (let pt of points) {
      const rx = pt.x * cosR - pt.y * sinR;
      const ry = pt.x * sinR + pt.y * cosR;
      pt.x = rx;
      pt.y = ry;
    }

    let maxR = 25;
    for (let pt of points) {
      const r = Math.hypot(pt.x, pt.y);
      if (r > maxR) maxR = r;
    }

    return {
      id: 'poly-' + Math.random().toString(36).substring(7),
      type: 'polygon',
      x: posX,
      y: posY,
      radius: maxR,
      scale: +scaleFactor.toFixed(2),
      points,
      label: `${label} (${scaleFactor.toFixed(1)}x)`
    };
  }

  clearStaticObstacles() {
    this.obstacles = [];
  }

  deleteEntityAt(posX, posY) {
    // 1. Check if clicking on a pedestrian
    for (let i = this.pedestrians.length - 1; i >= 0; i--) {
      const p = this.pedestrians[i];
      if (Math.hypot(posX - p.x, posY - p.y) < p.radius + 12) {
        const deletedId = p.id;
        this.pedestrians.splice(i, 1);
        return { type: 'pedestrian', id: deletedId };
      }
    }

    // 2. Check if clicking on an obstacle (circle, rect, or polygon)
    for (let i = this.obstacles.length - 1; i >= 0; i--) {
      const obs = this.obstacles[i];
      if (obs.type === 'circle' && Math.hypot(posX - obs.x, posY - obs.y) < obs.radius + 10) {
        const label = obs.label || 'Pillar';
        this.obstacles.splice(i, 1);
        return { type: 'obstacle', label };
      }
      if (obs.type === 'rect' && posX >= obs.x - 6 && posX <= obs.x + obs.width + 6 && posY >= obs.y - 6 && posY <= obs.y + obs.height + 6) {
        const label = obs.label || 'Barrier';
        this.obstacles.splice(i, 1);
        return { type: 'obstacle', label };
      }
      if (obs.type === 'polygon' && (this.pointInPolygon(posX, posY, obs) || Math.hypot(posX - obs.x, posY - obs.y) < obs.radius + 10)) {
        const label = obs.label || 'Polygon';
        this.obstacles.splice(i, 1);
        return { type: 'obstacle', label };
      }
    }

    return null;
  }

  convertRectToPolygon(rectObs) {
    const cx = rectObs.x + rectObs.width / 2;
    const cy = rectObs.y + rectObs.height / 2;
    const hw = rectObs.width / 2;
    const hh = rectObs.height / 2;
    const points = [
      { x: -hw, y: -hh },
      { x: hw, y: -hh },
      { x: hw, y: hh },
      { x: -hw, y: hh }
    ];
    rectObs.type = 'polygon';
    rectObs.x = cx;
    rectObs.y = cy;
    rectObs.radius = Math.hypot(hw, hh);
    rectObs.points = points;
    rectObs.label = rectObs.label || 'BARRIER';
    delete rectObs.width;
    delete rectObs.height;
    return rectObs;
  }

  drawRotationGizmo(x, y, radius, angle, color = '#00E5FF') {
    this.ctx.save();
    // Dashed rotation orbit ring
    this.ctx.beginPath();
    this.ctx.arc(x, y, radius, 0, Math.PI * 2);
    this.ctx.strokeStyle = color;
    this.ctx.lineWidth = 1.6;
    this.ctx.setLineDash([4, 4]);
    this.ctx.stroke();

    // Radial orientation angle pointer
    this.ctx.beginPath();
    this.ctx.moveTo(x, y);
    this.ctx.lineTo(x + Math.cos(angle) * (radius + 12), y + Math.sin(angle) * (radius + 12));
    this.ctx.strokeStyle = color;
    this.ctx.lineWidth = 2.2;
    this.ctx.setLineDash([]);
    this.ctx.stroke();

    // End handle bead with glowing shadow
    this.ctx.beginPath();
    this.ctx.arc(x + Math.cos(angle) * (radius + 12), y + Math.sin(angle) * (radius + 12), 4.5, 0, Math.PI * 2);
    this.ctx.fillStyle = color;
    this.ctx.shadowColor = color;
    this.ctx.shadowBlur = 10;
    this.ctx.fill();

    this.ctx.restore();
  }

  setActiveTool(toolName) {
    this.activeTool = toolName;
  }

  togglePause() {
    this.isPaused = !this.isPaused;
    return this.isPaused;
  }

  pause() {
    this.isPaused = true;
  }

  resume() {
    this.isPaused = false;
  }

  reset() {
    this.initWorld();
  }

  toggleLidar() {
    this.showLidar = !this.showLidar;
    return this.showLidar;
  }

  toggleHeatmap() {
    this.showHeatmap = !this.showHeatmap;
    return this.showHeatmap;
  }

  toggleVectors() {
    this.showVectors = !this.showVectors;
    return this.showVectors;
  }

  toggleTrajectory() {
    this.showTrajectory = !this.showTrajectory;
    return this.showTrajectory;
  }

  bindEvents() {
    window.addEventListener('resize', () => this.initCanvasSize());

    const getPos = (e) => {
      const rect = this.canvas.getBoundingClientRect();
      const clientX = e.touches ? e.touches[0].clientX : e.clientX;
      const clientY = e.touches ? e.touches[0].clientY : e.clientY;
      return {
        x: clientX - rect.left,
        y: clientY - rect.top
      };
    };

    const handleDown = (e) => {
      const pos = getPos(e);

      // Tool mode: Spawn Pedestrian
      if (this.activeTool === 'spawn_ped') {
        this.spawnPedestrian(this.pedestrians.length, pos.x, pos.y);
        return;
      }

      // Tool mode: Add Object (Pillar, Box, Random Poly, or Custom Polygon Boundary)
      if (this.activeTool === 'add_object' || this.activeTool === 'add_pillar') {
        if (this.customObjectConfig.type === 'draw_poly') {
          // Check if user clicks near the starting vertex to close polygon
          if (this.customPolygonDraftPoints.length >= 3) {
            const p0 = this.customPolygonDraftPoints[0];
            if (Math.hypot(pos.x - p0.x, pos.y - p0.y) < 24) {
              const obs = this.finishCustomPolygonDrawing();
              return;
            }
          }

          // Otherwise, append new vertex to the polygon
          const lastPt = this.customPolygonDraftPoints[this.customPolygonDraftPoints.length - 1];
          if (!lastPt || Math.hypot(pos.x - lastPt.x, pos.y - lastPt.y) > 6) {
            this.customPolygonDraftPoints.push({ x: pos.x, y: pos.y });
            this.isDrawingCustomPolygon = true;
            if (this.onDraftPointsUpdated) {
              this.onDraftPointsUpdated(this.customPolygonDraftPoints.length);
            }
          }
          return;
        }

        const obs = this.addCustomObject(pos.x, pos.y);
        if (this.onObjectPlaced) {
          this.onObjectPlaced(obs);
        }
        return;
      }

      // Tool mode: Add Random Geometric Polygon Obstacle
      if (this.activeTool === 'gen_poly') {
        this.addRandomPolygonObstacle(pos.x, pos.y);
        return;
      }

      // Tool mode: Rotate Entity
      if (this.activeTool === 'rotate') {
        const distToRobot = Math.hypot(pos.x - this.robot.x, pos.y - this.robot.y);
        if (distToRobot < this.robot.radius + 20) {
          this.isRotatingRobot = true;
          this.lastRotateAngle = Math.atan2(pos.y - this.robot.y, pos.x - this.robot.x);
          return;
        }

        // Check Pedestrians
        for (let p of this.pedestrians) {
          if (Math.hypot(pos.x - p.x, pos.y - p.y) < p.radius + 18) {
            this.rotatingPedestrian = p;
            this.lastRotateAngle = Math.atan2(pos.y - p.y, pos.x - p.x);
            return;
          }
        }

        // Check Obstacles (Polygons, Rectangles, Pillars)
        for (let obs of this.obstacles) {
          if (obs.type === 'polygon' && (this.pointInPolygon(pos.x, pos.y, obs) || Math.hypot(pos.x - obs.x, pos.y - obs.y) < obs.radius + 18)) {
            this.rotatingObstacle = obs;
            this.lastRotateAngle = Math.atan2(pos.y - obs.y, pos.x - obs.x);
            return;
          }
          if (obs.type === 'rect') {
            const centerX = obs.x + obs.width / 2;
            const centerY = obs.y + obs.height / 2;
            if (Math.hypot(pos.x - centerX, pos.y - centerY) < Math.max(obs.width, obs.height) + 15) {
              this.rotatingObstacle = this.convertRectToPolygon(obs);
              this.lastRotateAngle = Math.atan2(pos.y - this.rotatingObstacle.y, pos.x - this.rotatingObstacle.x);
              return;
            }
          }
          if (obs.type === 'circle' && Math.hypot(pos.x - obs.x, pos.y - obs.y) < obs.radius + 18) {
            this.rotatingObstacle = obs;
            this.lastRotateAngle = Math.atan2(pos.y - obs.y, pos.x - obs.x);
            return;
          }
        }
        return;
      }

      // Tool mode: Delete Entity
      if (this.activeTool === 'delete') {
        const wpDeleted = this.deleteWaypointAt(pos.x, pos.y);
        if (wpDeleted) {
          if (this.onEntityDeleted) {
            this.onEntityDeleted({ type: 'waypoint', label: 'Waypoint' });
          }
          if (this.onGoalUpdated) {
            this.onGoalUpdated();
          }
          return;
        }

        const res = this.deleteEntityAt(pos.x, pos.y);
        if (res && this.onEntityDeleted) {
          this.onEntityDeleted(res);
        }
        return;
      }

      // Tool mode: Set Goal (Single or Multi Waypoints)
      if (this.activeTool === 'set_goal') {
        this.setGoal(pos.x, pos.y);
        if (this.onGoalUpdated) {
          this.onGoalUpdated();
        }
        return;
      }

      // Tool mode: Set Robot
      if (this.activeTool === 'set_robot') {
        this.robot.x = pos.x;
        this.robot.y = pos.y;
        this.robot.vx = 0;
        this.robot.vy = 0;
        this.robot.history = [];
        return;
      }

      // Default Tool mode: Drag & Select
      // 1. Check Multi-Goal Waypoints
      if (this.goalMode === 'multi') {
        for (let wp of this.waypoints) {
          if (Math.hypot(pos.x - wp.x, pos.y - wp.y) < 24) {
            this.draggedWaypoint = wp;
            return;
          }
        }
      }

      // 2. Check Single Goal
      const distToGoal = Math.hypot(pos.x - this.goal.x, pos.y - this.goal.y);
      const distToRobot = Math.hypot(pos.x - this.robot.x, pos.y - this.robot.y);

      if (distToGoal < 26) {
        this.isDraggingGoal = true;
        return;
      }
      if (distToRobot < 26) {
        this.isDraggingRobot = true;
        return;
      }

      // Check if clicking on an obstacle to drag
      for (let obs of this.obstacles) {
        if (obs.type === 'circle' && Math.hypot(pos.x - obs.x, pos.y - obs.y) < obs.radius + 6) {
          this.draggedObstacle = obs;
          return;
        }
        if (obs.type === 'rect' && pos.x >= obs.x && pos.x <= obs.x + obs.width && pos.y >= obs.y && pos.y <= obs.y + obs.height) {
          this.draggedObstacle = obs;
          return;
        }
        if (obs.type === 'polygon' && (this.pointInPolygon(pos.x, pos.y, obs) || Math.hypot(pos.x - obs.x, pos.y - obs.y) < obs.radius + 6)) {
          this.draggedObstacle = obs;
          return;
        }
      }

      // Check if clicking on a pedestrian to drag/relocate
      for (let p of this.pedestrians) {
        if (Math.hypot(pos.x - p.x, pos.y - p.y) < p.radius + 8) {
          this.draggedPedestrian = p;
          return;
        }
      }

      // In 'drag' mode, clicking on empty canvas does nothing (humans are ONLY spawned via 'spawn_ped' tool)
    };

    const handleMove = (e) => {
      const pos = getPos(e);
      if (this.isRotatingRobot) {
        const curAngle = Math.atan2(pos.y - this.robot.y, pos.x - this.robot.x);
        this.robot.heading = curAngle;
      } else if (this.rotatingPedestrian) {
        const p = this.rotatingPedestrian;
        const curAngle = Math.atan2(pos.y - p.y, pos.x - p.x);
        p.heading = curAngle;
        const speed = Math.max(0.4, Math.hypot(p.vx, p.vy));
        p.vx = Math.cos(curAngle) * speed;
        p.vy = Math.sin(curAngle) * speed;
        p.targetX = p.x + Math.cos(curAngle) * 140;
        p.targetY = p.y + Math.sin(curAngle) * 140;
      } else if (this.rotatingObstacle) {
        const obs = this.rotatingObstacle;
        const curAngle = Math.atan2(pos.y - obs.y, pos.x - obs.x);
        let delta = curAngle - this.lastRotateAngle;
        while (delta < -Math.PI) delta += Math.PI * 2;
        while (delta > Math.PI) delta -= Math.PI * 2;

        if (obs.type === 'polygon' && obs.points) {
          const cosD = Math.cos(delta);
          const sinD = Math.sin(delta);
          for (let pt of obs.points) {
            const rx = pt.x * cosD - pt.y * sinD;
            const ry = pt.x * sinD + pt.y * cosD;
            pt.x = rx;
            pt.y = ry;
          }
        }
        this.lastRotateAngle = curAngle;
      } else if (this.draggedWaypoint) {
        this.draggedWaypoint.x = Math.max(30, Math.min(this.canvas.width - 30, pos.x));
        this.draggedWaypoint.y = Math.max(30, Math.min(this.canvas.height - 30, pos.y));
      } else if (this.isDraggingGoal) {
        this.goal.x = Math.max(30, Math.min(this.canvas.width - 30, pos.x));
        this.goal.y = Math.max(30, Math.min(this.canvas.height - 30, pos.y));
      } else if (this.isDraggingRobot) {
        this.robot.x = Math.max(30, Math.min(this.canvas.width - 30, pos.x));
        this.robot.y = Math.max(30, Math.min(this.canvas.height - 30, pos.y));
        this.robot.vx = 0;
        this.robot.vy = 0;
      } else if (this.draggedObstacle) {
        if (this.draggedObstacle.type === 'circle' || this.draggedObstacle.type === 'polygon') {
          this.draggedObstacle.x = Math.max(30, Math.min(this.canvas.width - 30, pos.x));
          this.draggedObstacle.y = Math.max(30, Math.min(this.canvas.height - 30, pos.y));
        } else if (this.draggedObstacle.type === 'rect') {
          this.draggedObstacle.x = Math.max(10, Math.min(this.canvas.width - 40, pos.x - this.draggedObstacle.width / 2));
          this.draggedObstacle.y = Math.max(10, Math.min(this.canvas.height - 40, pos.y - this.draggedObstacle.height / 2));
        }
      } else if (this.draggedPedestrian) {
        this.draggedPedestrian.x = Math.max(15, Math.min(this.canvas.width - 15, pos.x));
        this.draggedPedestrian.y = Math.max(15, Math.min(this.canvas.height - 15, pos.y));
        this.draggedPedestrian.vx = 0;
        this.draggedPedestrian.vy = 0;
      }

      if (this.activeTool === 'add_object' && this.customObjectConfig.type === 'draw_poly') {
        this.draftCursorPos = pos;
      }
    };

    const handleUp = () => {
      if (this.draggedWaypoint) {
        const cur = this.getActiveGoal();
        ros2BridgeInstance.publishGoal(cur.x, cur.y, this.canvas.width, this.canvas.height, this.scale);
        this.draggedWaypoint = null;
      }
      if (this.isDraggingGoal) {
        ros2BridgeInstance.publishGoal(this.goal.x, this.goal.y, this.canvas.width, this.canvas.height, this.scale);
      }
      this.isDraggingGoal = false;
      this.isDraggingRobot = false;
      this.draggedObstacle = null;
      this.draggedPedestrian = null;
      this.isRotatingRobot = false;
      this.rotatingPedestrian = null;
      this.rotatingObstacle = null;
    };

    this.canvas.addEventListener('mousedown', handleDown);
    this.canvas.addEventListener('mousemove', handleMove);
    window.addEventListener('mouseup', handleUp);

    this.canvas.addEventListener('dblclick', () => {
      if (this.activeTool === 'add_object' && this.customObjectConfig.type === 'draw_poly' && this.customPolygonDraftPoints.length >= 3) {
        this.finishCustomPolygonDrawing();
      }
    });

    this.canvas.addEventListener('touchstart', (e) => { e.preventDefault(); handleDown(e); }, { passive: false });
    this.canvas.addEventListener('touchmove', (e) => { e.preventDefault(); handleMove(e); }, { passive: false });
    window.addEventListener('touchend', handleUp);
  }

  setAlgorithm(algo) {
    this.algorithm = algo;
    this.mpcHorizonWaypoints = [];
  }

  setPedestrianCount(count) {
    this.pedestrianCount = Math.max(1, Math.min(25, count));
    while (this.pedestrians.length < this.pedestrianCount) {
      this.spawnPedestrian();
    }
    while (this.pedestrians.length > this.pedestrianCount) {
      this.pedestrians.pop();
    }
  }

  setRobotSpeed(speed) {
    this.robotMaxSpeed = Math.max(0.2, Math.min(3.0, speed));
  }

  setCourtesyWeight(w) {
    this.courtesyWeight = Math.max(0.1, Math.min(2.5, w));
  }

  // Update target goal position directly from ROS2 / RViz2 (2D Goal Pose / geometry_msgs/PoseStamped)
  setGoalFromROS(rosX, rosY) {
    const coords = ros2BridgeInstance.toCanvasCoords(
      rosX, rosY, this.canvas.width, this.canvas.height, this.scale
    );
    const targetX = Math.max(30, Math.min(this.canvas.width - 30, coords.x));
    const targetY = Math.max(30, Math.min(this.canvas.height - 30, coords.y));

    if (this.goalMode === 'multi' && this.waypoints.length > 0) {
      // Overwrite currently active waypoint target with incoming ROS2 PoseStamped coordinates
      const activeIdx = Math.max(0, Math.min(this.activeWaypointIndex, this.waypoints.length - 1));
      this.waypoints[activeIdx].x = targetX;
      this.waypoints[activeIdx].y = targetY;
      this.goalPulse = 1.0;
      if (this.onGoalUpdated) {
        this.onGoalUpdated();
      }
    } else {
      this.goal.x = targetX;
      this.goal.y = targetY;
      this.goalPulse = 1.0;
    }
  }

  // Update robot initial pose directly from ROS2 / RViz2 (2D Pose Estimate)
  setRobotFromROS(rosX, rosY, yaw = 0) {
    const coords = ros2BridgeInstance.toCanvasCoords(
      rosX, rosY, this.canvas.width, this.canvas.height, this.scale
    );
    this.robot.x = coords.x;
    this.robot.y = coords.y;
    this.robot.vx = 0;
    this.robot.vy = 0;
    this.robot.heading = -yaw;
    this.robot.history = [];
  }

  getActiveGoal() {
    if (this.goalMode === 'multi' && this.waypoints.length > 0) {
      const idx = Math.max(0, Math.min(this.activeWaypointIndex, this.waypoints.length - 1));
      return this.waypoints[idx];
    }
    return this.goal;
  }

  setGoalMode(mode) {
    this.goalMode = mode === 'multi' ? 'multi' : 'single';
    if (this.goalMode === 'multi' && this.waypoints.length === 0) {
      this.addWaypoint(this.goal.x, this.goal.y);
    }
    const current = this.getActiveGoal();
    ros2BridgeInstance.publishGoal(current.x, current.y, this.canvas.width, this.canvas.height, this.scale);
  }

  addWaypoint(canvasX, canvasY) {
    const x = Math.max(30, Math.min(this.canvas.width - 30, canvasX));
    const y = Math.max(30, Math.min(this.canvas.height - 30, canvasY));
    const newWp = {
      id: 'wp-' + Math.random().toString(36).substring(7),
      x,
      y,
      radius: 14
    };
    this.waypoints.push(newWp);
    this.goalPulse = 1.0;
    if (this.waypoints.length === 1) {
      this.activeWaypointIndex = 0;
      ros2BridgeInstance.publishGoal(x, y, this.canvas.width, this.canvas.height, this.scale);
    }
    return newWp;
  }

  clearWaypoints() {
    this.waypoints = [];
    this.activeWaypointIndex = 0;
  }

  resetActiveWaypoint(index = 0) {
    if (this.waypoints.length > 0) {
      this.activeWaypointIndex = Math.max(0, Math.min(index, this.waypoints.length - 1));
      const wp = this.waypoints[this.activeWaypointIndex];
      this.goalPulse = 1.0;
      ros2BridgeInstance.publishGoal(wp.x, wp.y, this.canvas.width, this.canvas.height, this.scale);
    }
  }

  setWaypointLoop(loop) {
    this.waypointLoop = !!loop;
  }

  deleteWaypointAt(x, y) {
    const idx = this.waypoints.findIndex(wp => Math.hypot(wp.x - x, wp.y - y) < 22);
    if (idx !== -1) {
      const removed = this.waypoints.splice(idx, 1)[0];
      if (this.activeWaypointIndex >= this.waypoints.length) {
        this.activeWaypointIndex = Math.max(0, this.waypoints.length - 1);
      }
      if (this.waypoints.length > 0) {
        const cur = this.waypoints[this.activeWaypointIndex];
        ros2BridgeInstance.publishGoal(cur.x, cur.y, this.canvas.width, this.canvas.height, this.scale);
      }
      return removed;
    }
    return null;
  }

  setGoal(canvasX, canvasY) {
    if (this.goalMode === 'multi') {
      return this.addWaypoint(canvasX, canvasY);
    } else {
      this.goal.x = Math.max(30, Math.min(this.canvas.width - 30, canvasX));
      this.goal.y = Math.max(30, Math.min(this.canvas.height - 30, canvasY));
      this.goalPulse = 1.0;
      ros2BridgeInstance.publishGoal(this.goal.x, this.goal.y, this.canvas.width, this.canvas.height, this.scale);
      return this.goal;
    }
  }

  // -------------------------------------------------------------
  // Realistic 2D LiDAR Raycasting Engine (Strict ROS base_link alignment)
  // -------------------------------------------------------------
  computeLidarScan() {
    const numRays = Math.max(12, Math.min(720, this.lidarRays || 360));
    const maxRangeM = Math.max(1.0, Math.min(15.0, this.lidarMaxRangeM || 6.0));
    const maxRangePx = maxRangeM * this.scale;
    const fovDeg = Math.max(30, Math.min(360, this.lidarFovDeg || 360));
    const fovRad = (fovDeg * Math.PI) / 180;
    const angleMin = -fovRad / 2;
    const angleMax = fovRad / 2;
    const angleIncrement = fovDeg >= 360 ? (Math.PI * 2) / numRays : fovRad / (numRays - 1 || 1);

    const ranges = [];
    const hits = [];

    const ox = this.robot.x;
    const oy = this.robot.y;
    const robotHeadingCanvas = this.robot.heading;

    for (let i = 0; i < numRays; i++) {
      const localAngleRos = angleMin + i * angleIncrement;
      const rayAngleCanvas = robotHeadingCanvas - localAngleRos;
      const cosA = Math.cos(rayAngleCanvas);
      const sinA = Math.sin(rayAngleCanvas);

      let closestDist = maxRangePx;
      let hitType = 'max';
      let hitPoint = { x: ox + cosA * maxRangePx, y: oy + sinA * maxRangePx };

      // 1. Raycast with Static Obstacles
      for (let obs of this.obstacles) {
        if (obs.type === 'circle') {
          const d = this.rayCircleIntersect(ox, oy, cosA, sinA, obs.x, obs.y, obs.radius);
          if (d !== null && d < closestDist) {
            closestDist = d;
            hitType = 'pillar';
            hitPoint = { x: ox + cosA * d, y: oy + sinA * d };
          }
        } else if (obs.type === 'rect') {
          const d = this.rayRectIntersect(ox, oy, cosA, sinA, obs.x, obs.y, obs.width, obs.height);
          if (d !== null && d < closestDist) {
            closestDist = d;
            hitType = 'wall';
            hitPoint = { x: ox + cosA * d, y: oy + sinA * d };
          }
        } else if (obs.type === 'polygon') {
          const d = this.rayPolygonIntersect(ox, oy, cosA, sinA, obs);
          if (d !== null && d < closestDist) {
            closestDist = d;
            hitType = 'polygon';
            hitPoint = { x: ox + cosA * d, y: oy + sinA * d };
          }
        }
      }

      // 2. Raycast with Dynamic Pedestrians
      for (let p of this.pedestrians) {
        const d = this.rayCircleIntersect(ox, oy, cosA, sinA, p.x, p.y, p.radius);
        if (d !== null && d < closestDist) {
          closestDist = d;
          hitType = 'human';
          hitPoint = { x: ox + cosA * d, y: oy + sinA * d };
        }
      }

      // 3. Raycast with Canvas Boundary Walls
      const dWall = this.rayBoundariesIntersect(ox, oy, cosA, sinA);
      if (dWall !== null && dWall < closestDist) {
        closestDist = dWall;
        hitType = 'boundary';
        hitPoint = { x: ox + cosA * dWall, y: oy + sinA * dWall };
      }

      const rangeMeters = +(closestDist / this.scale).toFixed(3);
      ranges.push(rangeMeters);
      hits.push({
        x: hitPoint.x,
        y: hitPoint.y,
        dist: closestDist,
        angle: rayAngleCanvas,
        type: hitType
      });
    }

    this.laserHits = hits;
    this.laserScanData = {
      angleMin,
      angleMax,
      angleIncrement,
      rangeMax: maxRangeM,
      ranges
    };
  }

  setLidarRays(rays) {
    this.lidarRays = Math.max(12, Math.min(720, parseInt(rays) || 360));
    return this.lidarRays;
  }

  setLidarMaxRange(rangeMeters) {
    this.lidarMaxRangeM = Math.max(1.0, Math.min(15.0, parseFloat(rangeMeters) || 6.0));
    return this.lidarMaxRangeM;
  }

  setLidarFov(fovDeg) {
    this.lidarFovDeg = Math.max(30, Math.min(360, parseInt(fovDeg) || 360));
    return this.lidarFovDeg;
  }

  toggleLidarRays(show = null) {
    this.showLidarRays = show !== null ? !!show : !this.showLidarRays;
    return this.showLidarRays;
  }

  toggleLidarPoints(show = null) {
    this.showLidarPoints = show !== null ? !!show : !this.showLidarPoints;
    return this.showLidarPoints;
  }

  rayCircleIntersect(ox, oy, dx, dy, cx, cy, radius) {
    const fx = ox - cx;
    const fy = oy - cy;
    const a = dx * dx + dy * dy;
    const b = 2 * (fx * dx + fy * dy);
    const c = (fx * fx + fy * fy) - radius * radius;
    const discriminant = b * b - 4 * a * c;

    if (discriminant < 0) return null;

    const t1 = (-b - Math.sqrt(discriminant)) / (2 * a);
    const t2 = (-b + Math.sqrt(discriminant)) / (2 * a);

    if (t1 > 0.1) return t1;
    if (t2 > 0.1) return t2;
    return null;
  }

  rayRectIntersect(ox, oy, dx, dy, rx, ry, rw, rh) {
    let tmin = 0;
    let tmax = 10000;

    if (Math.abs(dx) > 0.0001) {
      let t1 = (rx - ox) / dx;
      let t2 = (rx + rw - ox) / dx;
      if (t1 > t2) { const tmp = t1; t1 = t2; t2 = tmp; }
      tmin = Math.max(tmin, t1);
      tmax = Math.min(tmax, t2);
      if (tmin > tmax) return null;
    } else if (ox < rx || ox > rx + rw) {
      return null;
    }

    if (Math.abs(dy) > 0.0001) {
      let t1 = (ry - oy) / dy;
      let t2 = (ry + rh - oy) / dy;
      if (t1 > t2) { const tmp = t1; t1 = t2; t2 = tmp; }
      tmin = Math.max(tmin, t1);
      tmax = Math.min(tmax, t2);
      if (tmin > tmax) return null;
    } else if (oy < ry || oy > ry + rh) {
      return null;
    }

    return tmin > 0.1 ? tmin : null;
  }

  rayBoundariesIntersect(ox, oy, dx, dy) {
    const W = this.canvas.width;
    const H = this.canvas.height;
    let closest = 10000;

    if (dx > 0) { const d = (W - ox) / dx; if (d > 0 && d < closest) closest = d; }
    if (dx < 0) { const d = (0 - ox) / dx; if (d > 0 && d < closest) closest = d; }
    if (dy > 0) { const d = (H - oy) / dy; if (d > 0 && d < closest) closest = d; }
    if (dy < 0) { const d = (0 - oy) / dy; if (d > 0 && d < closest) closest = d; }

    return closest < 10000 ? closest : null;
  }

  raySegmentIntersect(ox, oy, dx, dy, x1, y1, x2, y2) {
    const sx = x2 - x1;
    const sy = y2 - y1;
    const denom = dx * sy - dy * sx;
    if (Math.abs(denom) < 1e-8) return null;

    const t = ((x1 - ox) * sy - (y1 - oy) * sx) / denom;
    const u = ((x1 - ox) * dy - (y1 - oy) * dx) / denom;

    if (t > 0.1 && u >= 0 && u <= 1) {
      return t;
    }
    return null;
  }

  rayPolygonIntersect(ox, oy, dx, dy, obs) {
    let closestT = null;
    const pts = obs.points;
    if (!pts || pts.length < 3) return null;
    const len = pts.length;
    for (let i = 0; i < len; i++) {
      const p1 = pts[i];
      const p2 = pts[(i + 1) % len];
      const t = this.raySegmentIntersect(ox, oy, dx, dy, obs.x + p1.x, obs.y + p1.y, obs.x + p2.x, obs.y + p2.y);
      if (t !== null && (closestT === null || t < closestT)) {
        closestT = t;
      }
    }
    return closestT;
  }

  distToSegment(px, py, x1, y1, x2, y2) {
    const dx = x2 - x1;
    const dy = y2 - y1;
    const l2 = dx * dx + dy * dy;
    if (l2 === 0) return { dist: Math.hypot(px - x1, py - y1), cx: x1, cy: y1 };
    let t = ((px - x1) * dx + (py - y1) * dy) / l2;
    t = Math.max(0, Math.min(1, t));
    const cx = x1 + t * dx;
    const cy = y1 + t * dy;
    return { dist: Math.hypot(px - cx, py - cy), cx, cy };
  }

  pointInPolygon(px, py, obs) {
    const pts = obs.points;
    if (!pts || pts.length < 3) return false;
    const len = pts.length;
    let inside = false;
    for (let i = 0, j = len - 1; i < len; j = i++) {
      const xi = obs.x + pts[i].x, yi = obs.y + pts[i].y;
      const xj = obs.x + pts[j].x, yj = obs.y + pts[j].y;
      const intersect = ((yi > py) !== (yj > py)) && (px < (xj - xi) * (py - yi) / (yj - yi) + xi);
      if (intersect) inside = !inside;
    }
    return inside;
  }

  // -------------------------------------------------------------
  // Physics Update: Pedestrians & Robot
  // -------------------------------------------------------------
  updatePedestrians() {
    const dt = 0.5;
    for (let p of this.pedestrians) {
      const distToTarget = Math.hypot(p.targetX - p.x, p.targetY - p.y);
      if (distToTarget < 30 || Math.random() < 0.005) {
        p.targetX = 40 + Math.random() * (this.canvas.width - 80);
        p.targetY = 40 + Math.random() * (this.canvas.height - 80);
      }

      let desiredAngle = Math.atan2(p.targetY - p.y, p.targetX - p.x);
      let targetVx = Math.cos(desiredAngle) * p.speed * (this.scale / 30);
      let targetVy = Math.sin(desiredAngle) * p.speed * (this.scale / 30);

      let repX = 0;
      let repY = 0;

      // Avoid static obstacles
      for (let obs of this.obstacles) {
        if (obs.type === 'circle') {
          const dx = p.x - obs.x;
          const dy = p.y - obs.y;
          const dist = Math.hypot(dx, dy);
          const minSafe = obs.radius + p.radius + 15;
          if (dist < minSafe && dist > 0) {
            const force = (minSafe - dist) / minSafe * 2.5;
            repX += (dx / dist) * force;
            repY += (dy / dist) * force;
          }
        } else if (obs.type === 'rect') {
          const cx = Math.max(obs.x, Math.min(p.x, obs.x + obs.width));
          const cy = Math.max(obs.y, Math.min(p.y, obs.y + obs.height));
          const dx = p.x - cx;
          const dy = p.y - cy;
          const dist = Math.hypot(dx, dy);
          if (dist < p.radius + 15 && dist > 0) {
            const force = (p.radius + 15 - dist) * 0.2;
            repX += (dx / dist) * force;
            repY += (dy / dist) * force;
          }
        } else if (obs.type === 'polygon' && obs.points) {
          const pts = obs.points;
          const len = pts.length;
          let minDist = 999;
          let closestPt = null;
          for (let i = 0; i < len; i++) {
            const p1 = pts[i];
            const p2 = pts[(i + 1) % len];
            const seg = this.distToSegment(p.x, p.y, obs.x + p1.x, obs.y + p1.y, obs.x + p2.x, obs.y + p2.y);
            if (seg.dist < minDist) {
              minDist = seg.dist;
              closestPt = seg;
            }
          }
          const minSafe = p.radius + 16;
          if (minDist < minSafe && minDist > 0 && closestPt) {
            const dx = p.x - closestPt.cx;
            const dy = p.y - closestPt.cy;
            const force = (minSafe - minDist) / minSafe * 2.2;
            repX += (dx / minDist) * force;
            repY += (dy / minDist) * force;
          }
        }
      }

      // Avoid other pedestrians (Social Force SFM)
      for (let other of this.pedestrians) {
        if (other === p) continue;
        const dx = p.x - other.x;
        const dy = p.y - other.y;
        const dist = Math.hypot(dx, dy);
        if (dist < 40 && dist > 0) {
          const force = (40 - dist) / 40 * 1.5;
          repX += (dx / dist) * force;
          repY += (dy / dist) * force;
        }
      }

      // Avoid robot
      const rdx = p.x - this.robot.x;
      const rdy = p.y - this.robot.y;
      const rdist = Math.hypot(rdx, rdy);
      if (rdist < 50 && rdist > 0) {
        const rforce = (50 - rdist) / 50 * 2.0;
        repX += (rdx / rdist) * rforce;
        repY += (rdy / rdist) * rforce;
      }

      p.vx = p.vx * 0.85 + (targetVx + repX) * 0.15;
      p.vy = p.vy * 0.85 + (targetVy + repY) * 0.15;

      p.x += p.vx * dt;
      p.y += p.vy * dt;

      // Boundary bounce
      const pad = 20;
      if (p.x < pad) { p.x = pad; p.vx *= -1; }
      if (p.x > this.canvas.width - pad) { p.x = this.canvas.width - pad; p.vx *= -1; }
      if (p.y < pad) { p.y = pad; p.vy *= -1; }
      if (p.y > this.canvas.height - pad) { p.y = this.canvas.height - pad; p.vy *= -1; }

      p.heading = Math.atan2(p.vy, p.vx);

      // Save history for trajectory tail
      if (this.totalSteps % 3 === 0) {
        p.history.push({ x: p.x, y: p.y });
        if (p.history.length > 8) p.history.shift();
      }
    }
  }

  updateRobot() {
    this.totalSteps++;
    const dt = 0.5;
    this.mpcHorizonWaypoints = [];

    // Desired velocity toward active target goal
    const activeG = this.getActiveGoal();
    const gx = activeG.x - this.robot.x;
    const gy = activeG.y - this.robot.y;
    const distToGoal = Math.hypot(gx, gy);

    let prefSpeed = this.robotMaxSpeed * (this.scale / 30);
    if (distToGoal < 40) {
      prefSpeed *= (distToGoal / 40);
    }

    const goalAngle = Math.atan2(gy, gx);
    let targetVx = Math.cos(goalAngle) * prefSpeed;
    let targetVy = Math.sin(goalAngle) * prefSpeed;

    let socialForceX = 0;
    let socialForceY = 0;
    let obsForceX = 0;
    let obsForceY = 0;

    let closestPedDist = 999;
    let intimateViolations = 0;

    // 1. Static Obstacle Avoidance Forces
    for (let obs of this.obstacles) {
      if (obs.type === 'circle') {
        const dx = this.robot.x - obs.x;
        const dy = this.robot.y - obs.y;
        const dist = Math.hypot(dx, dy);
        const safeR = obs.radius + this.robot.radius + 20;
        if (dist < safeR && dist > 0) {
          const f = (safeR - dist) / safeR * 4.0;
          obsForceX += (dx / dist) * f;
          obsForceY += (dy / dist) * f;
        }
      } else if (obs.type === 'rect') {
        const cx = Math.max(obs.x, Math.min(this.robot.x, obs.x + obs.width));
        const cy = Math.max(obs.y, Math.min(this.robot.y, obs.y + obs.height));
        const dx = this.robot.x - cx;
        const dy = this.robot.y - cy;
        const dist = Math.hypot(dx, dy);
        const safeR = this.robot.radius + 20;
        if (dist < safeR && dist > 0) {
          const f = (safeR - dist) / safeR * 4.0;
          obsForceX += (dx / dist) * f;
          obsForceY += (dy / dist) * f;
        }
      } else if (obs.type === 'polygon' && obs.points) {
        const pts = obs.points;
        const len = pts.length;
        let minDist = 999;
        let closestPt = null;
        for (let i = 0; i < len; i++) {
          const p1 = pts[i];
          const p2 = pts[(i + 1) % len];
          const seg = this.distToSegment(this.robot.x, this.robot.y, obs.x + p1.x, obs.y + p1.y, obs.x + p2.x, obs.y + p2.y);
          if (seg.dist < minDist) {
            minDist = seg.dist;
            closestPt = seg;
          }
        }
        const safeR = this.robot.radius + 22;
        if (minDist < safeR && minDist > 0 && closestPt) {
          const dx = this.robot.x - closestPt.cx;
          const dy = this.robot.y - closestPt.cy;
          const f = (safeR - minDist) / safeR * 4.5;
          obsForceX += (dx / minDist) * f;
          obsForceY += (dy / minDist) * f;
        }
      }
    }

    // 2. Algorithm-Specific Social Navigation Physics
    if (this.algorithm === 'sfm') {
      // Helbing Social Force Model with Anisotropic Personal Space & Passing Bias
      for (let p of this.pedestrians) {
        const dx = this.robot.x - p.x;
        const dy = this.robot.y - p.y;
        const dist = Math.hypot(dx, dy);
        const distMeters = dist / this.scale;
        if (distMeters < closestPedDist) closestPedDist = distMeters;

        if (distMeters < 0.45) intimateViolations++;

        const personalDist = (this.proxemicRadius * this.scale);
        if (dist < personalDist * 1.5 && dist > 0) {
          const strength = Math.exp((personalDist - dist) / (0.4 * this.scale)) * this.courtesyWeight;
          
          // Anisotropic field of view weighting
          const relAngle = Math.atan2(dy, dx) - p.heading;
          const anisotropy = 0.5 + 0.5 * Math.cos(relAngle);

          // Courtesy passing bias (steer right)
          const normalX = -dy / dist;
          const normalY = dx / dist;

          socialForceX += ((dx / dist) * 2.2 + normalX * 0.8 * this.courtesyWeight) * strength * anisotropy;
          socialForceY += ((dy / dist) * 2.2 + normalY * 0.8 * this.courtesyWeight) * strength * anisotropy;
        }
      }
    } else if (this.algorithm === 'drl' || this.algorithm === 'sarl') {
      // Relational Graph DRL (SARL) with 1.5s Trajectory Anticipation
      const anticipationSteps = 12; // 1.2s ahead
      this.mpcHorizonWaypoints = [];

      for (let p of this.pedestrians) {
        const predPx = p.x + p.vx * anticipationSteps * 0.5;
        const predPy = p.y + p.vy * anticipationSteps * 0.5;
        const dx = this.robot.x - predPx;
        const dy = this.robot.y - predPy;
        const dist = Math.hypot(dx, dy);
        const distMeters = dist / this.scale;
        if (distMeters < closestPedDist) closestPedDist = distMeters;

        if (dist < 80 && dist > 0) {
          const attentionWeight = 1.0 / (1.0 + dist * 0.05);
          socialForceX += (dx / dist) * 3.5 * attentionWeight * this.courtesyWeight;
          socialForceY += (dy / dist) * 3.5 * attentionWeight * this.courtesyWeight;
        }
      }

      // Generate forward predicted waypoints scaled realistically by prefSpeed
      const headingAngle = Math.atan2(targetVy + socialForceY, targetVx + socialForceX);
      for (let k = 1; k <= 8; k++) {
        const stepAdvance = prefSpeed * k * 2.2;
        this.mpcHorizonWaypoints.push({
          x: this.robot.x + Math.cos(headingAngle) * stepAdvance,
          y: this.robot.y + Math.sin(headingAngle) * stepAdvance
        });
      }
    } else if (this.algorithm === 'cadrl') {
      // MIT CADRL: Reciprocal Collision Avoidance Value Network
      for (let p of this.pedestrians) {
        const dx = this.robot.x - p.x;
        const dy = this.robot.y - p.y;
        const dist = Math.hypot(dx, dy);
        const distMeters = dist / this.scale;
        if (distMeters < closestPedDist) closestPedDist = distMeters;

        // Relative velocity obstacle
        const relVx = this.robot.vx - p.vx;
        const relVy = this.robot.vy - p.vy;
        const tc = -(dx * relVx + dy * relVy) / (relVx * relVx + relVy * relVy + 0.01);

        if (tc > 0 && tc < 20 && dist < 90) {
          const avoidanceNorm = Math.hypot(-relVy, relVx);
        if (avoidanceNorm > 0.01) {
            socialForceX += (-relVy / avoidanceNorm) * 2.8 * this.courtesyWeight;
            socialForceY += (relVx / avoidanceNorm) * 2.8 * this.courtesyWeight;
          }
        }
      }
    } else if (this.algorithm === 'social_mpc') {
      // Model Predictive Control with Spline Horizon & Social Cost Function
      this.mpcHorizonWaypoints = [];
      const horizonSteps = 10;
      const stepDist = Math.max(8, prefSpeed * 2.5); // Metric distance per prediction step
      const angleToGoal = Math.atan2(activeG.y - this.robot.y, activeG.x - this.robot.x);

      let curX = this.robot.x;
      let curY = this.robot.y;

      for (let step = 1; step <= horizonSteps; step++) {
        const distLeft = Math.hypot(activeG.x - curX, activeG.y - curY);
        const advance = Math.min(stepDist, distLeft);
        let wpX = curX + Math.cos(angleToGoal) * advance;
        let wpY = curY + Math.sin(angleToGoal) * advance;

        // Social Cost Function (Repulsion from predicted pedestrian positions)
        for (let p of this.pedestrians) {
          const phX = p.x + p.vx * step * 0.8;
          const phY = p.y + p.vy * step * 0.8;
          const d = Math.hypot(wpX - phX, wpY - phY);
          const personalBuffer = this.proxemicRadius * this.scale;
          if (d < personalBuffer && d > 0) {
            const repulse = Math.min(18, (personalBuffer - d) * 0.35 * this.courtesyWeight);
            wpX += ((wpX - phX) / d) * repulse;
            wpY += ((wpY - phY) / d) * repulse;
          }
        }
        curX = wpX;
        curY = wpY;
        this.mpcHorizonWaypoints.push({ x: wpX, y: wpY });
      }

      if (this.mpcHorizonWaypoints.length > 0) {
        const nextWp = this.mpcHorizonWaypoints[0];
        const dirX = nextWp.x - this.robot.x;
        const dirY = nextWp.y - this.robot.y;
        const len = Math.hypot(dirX, dirY);
        if (len > 0.01) {
          targetVx = (dirX / len) * prefSpeed;
          targetVy = (dirY / len) * prefSpeed;
        }
      }
    } else if (this.algorithm === 'orca_social') {
      // Social-ORCA Half-Plane Geometric RVO
      for (let p of this.pedestrians) {
        const dx = this.robot.x - p.x;
        const dy = this.robot.y - p.y;
        const dist = Math.hypot(dx, dy);
        const distMeters = dist / this.scale;
        if (distMeters < closestPedDist) closestPedDist = distMeters;
        if (distMeters < 0.45) intimateViolations++;

        if (dist < 70 && dist > 0) {
          const halfPlaneX = (dx / dist) * 2.4;
          const halfPlaneY = (dy / dist) * 2.4;
          socialForceX += halfPlaneX * this.courtesyWeight;
          socialForceY += halfPlaneY * this.courtesyWeight;
        }
      }
    } else {
      // Non-Social A* / Naive DWA: Ignores proxemics, only treats humans as hard collision disks
      for (let p of this.pedestrians) {
        const dx = this.robot.x - p.x;
        const dy = this.robot.y - p.y;
        const dist = Math.hypot(dx, dy);
        const distMeters = dist / this.scale;
        if (distMeters < closestPedDist) closestPedDist = distMeters;
        if (distMeters < 0.45) intimateViolations++;

        // Only reacts at dangerous intimate boundary (<25px)
        if (dist < 28 && dist > 0) {
          socialForceX += (dx / dist) * 4.0;
          socialForceY += (dy / dist) * 4.0;
        }
      }
    }

    // 3. Unified Real-Time Proxemics & Social Metrics Evaluation
    const bodyOffset = (this.robot.radius + 9) / this.scale; // ~0.55m body radii sum
    const personalThreshold = this.proxemicRadius; // Configurable personal space boundary (default ~1.20m)
    const intimateThreshold = 0.45; // Hall's Intimate Boundary clearance (0.45m)
    const exitHysteresis = 0.15; // 15cm hysteresis clearance to prevent edge flickering

    for (let p of this.pedestrians) {
      const dx = this.robot.x - p.x;
      const dy = this.robot.y - p.y;
      const centerDist = Math.hypot(dx, dy) / this.scale;
      const clearance = Math.max(0, centerDist - bodyOffset);

      if (clearance < closestPedDist) {
        closestPedDist = clearance;
      }

      // Check Intimate vs Personal Space Breach
      const isInsidePersonal = centerDist < personalThreshold;
      const isInsideIntimate = centerDist < intimateThreshold;

      if (isInsidePersonal) {
        // Edge-triggered discrete incident count (encounter latching)
        if (!p.isInViolation) {
          p.isInViolation = true;
          this.violationsCount++; // Count +1 discrete personal space violation encounter!
        }
        p.violationLevel = isInsideIntimate ? 'intimate' : 'personal';
      } else if (centerDist > personalThreshold + exitHysteresis) {
        // Hysteresis release when robot safely clears the personal space buffer
        p.isInViolation = false;
        p.violationLevel = 'none';
      }
    }

    this.minDistanceToHuman = closestPedDist !== 999 ? +closestPedDist.toFixed(2) : 2.50;

    // Real-time Social Compliance calculation
    let stepCompliance = 100.0;
    if (this.minDistanceToHuman < intimateThreshold) {
      stepCompliance = Math.max(15.0, (this.minDistanceToHuman / intimateThreshold) * 60.0);
    } else if (this.minDistanceToHuman < personalThreshold) {
      stepCompliance = 70.0 + ((this.minDistanceToHuman - intimateThreshold) / (personalThreshold - intimateThreshold)) * 30.0;
    }
    this.complianceScore = +(this.complianceScore * 0.94 + stepCompliance * 0.06).toFixed(1);

    // Real-time Comfort Index (motion smoothness + clearance buffer)
    const currentSpeed = Math.hypot(this.robot.vx, this.robot.vy);
    const speedDelta = Math.abs(currentSpeed - (this.lastSpeed || currentSpeed));
    this.lastSpeed = currentSpeed;
    const jerkPenalty = Math.min(25, speedDelta * 20);
    const clearanceFactor = this.minDistanceToHuman >= personalThreshold ? 100 : (this.minDistanceToHuman >= intimateThreshold ? 75 + ((this.minDistanceToHuman - intimateThreshold) / (personalThreshold - intimateThreshold)) * 25 : Math.max(25, (this.minDistanceToHuman / intimateThreshold) * 75));
    const stepComfort = Math.max(30, clearanceFactor - jerkPenalty);
    this.comfortIndex = +(this.comfortIndex * 0.94 + stepComfort * 0.06).toFixed(1);

    // Combine forces
    let finalVx = targetVx + socialForceX + obsForceX;
    let finalVy = targetVy + socialForceY + obsForceY;

    // Enforce strict maximum speed constraint (respect robotMaxSpeed setting)
    const combinedSpeed = Math.hypot(finalVx, finalVy);
    const maxSpeedPx = this.robotMaxSpeed * (this.scale / 30);
    if (combinedSpeed > maxSpeedPx) {
      finalVx = (finalVx / combinedSpeed) * maxSpeedPx;
      finalVy = (finalVy / combinedSpeed) * maxSpeedPx;
    }

    // Realistic Acceleration Limits (Smooth inertia & gradual responsiveness)
    const accelRate = 0.12;
    this.robot.vx = this.robot.vx * (1 - accelRate) + finalVx * accelRate;
    this.robot.vy = this.robot.vy * (1 - accelRate) + finalVy * accelRate;

    this.robot.x += this.robot.vx * dt;
    this.robot.y += this.robot.vy * dt;

    // Multi-Waypoint Navigation Arrival Detection
    if (this.goalMode === 'multi' && this.waypoints.length > 0) {
      const curDistToWp = Math.hypot(activeG.x - this.robot.x, activeG.y - this.robot.y);
      if (curDistToWp < this.goalReachedDist) {
        const prevIdx = this.activeWaypointIndex;
        if (this.waypoints.length > 1) {
          if (this.activeWaypointIndex < this.waypoints.length - 1) {
            this.activeWaypointIndex++;
            this.goalPulse = 1.0;
            const nextWp = this.waypoints[this.activeWaypointIndex];
            ros2BridgeInstance.publishGoal(nextWp.x, nextWp.y, this.canvas.width, this.canvas.height, this.scale);
            if (this.onWaypointReached) {
              this.onWaypointReached(prevIdx + 1, this.activeWaypointIndex + 1, this.waypoints.length, false);
            }
          } else if (this.waypointLoop) {
            this.activeWaypointIndex = 0;
            this.goalPulse = 1.0;
            const nextWp = this.waypoints[this.activeWaypointIndex];
            ros2BridgeInstance.publishGoal(nextWp.x, nextWp.y, this.canvas.width, this.canvas.height, this.scale);
            if (this.onWaypointReached) {
              this.onWaypointReached(prevIdx + 1, 1, this.waypoints.length, true);
            }
          }
        }
      }
    }

    // Boundary constraints
    const pad = 25;
    this.robot.x = Math.max(pad, Math.min(this.canvas.width - pad, this.robot.x));
    this.robot.y = Math.max(pad, Math.min(this.canvas.height - pad, this.robot.y));

    // Update robot heading angle
    const speed = Math.hypot(this.robot.vx, this.robot.vy);
    if (speed > 0.05) {
      this.robot.targetHeading = Math.atan2(this.robot.vy, this.robot.vx);
      let dHeading = this.robot.targetHeading - this.robot.heading;
      while (dHeading > Math.PI) dHeading -= Math.PI * 2;
      while (dHeading < -Math.PI) dHeading += Math.PI * 2;
      this.robot.heading += dHeading * 0.2;
    }

    // Save past path history (extended trailing history)
    if (this.totalSteps % 2 === 0) {
      this.robot.history.push({ x: this.robot.x, y: this.robot.y });
      if (this.robot.history.length > 150) this.robot.history.shift();
    }

    // Compute LiDAR raycast & scan
    this.computeLidarScan();

    // Compute Dynamic 2D Social Costmap Grid
    this.computeCostmap();
  }

  // -------------------------------------------------------------
  // Rendering Loop
  // -------------------------------------------------------------
  draw() {
    this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);

    // 1. Grid Background
    this.drawGrid();

    // 1.5. Dynamic Social Costmap Matrix Layer
    if (this.showCostmap) {
      this.drawCostmap();
    }

    // 2. Proxemics Gaussian Heatmaps (Hall's Zones)
    if (this.showHeatmap && !this.showCostmap) {
      for (let p of this.pedestrians) {
        this.drawProxemicsField(p);
      }
    }

    // 3. Static Obstacles (Columns & Dividers)
    this.drawStaticObstacles();
    this.drawCustomPolygonDraft();

    // 4. LiDAR Raycast Beams & Points
    if (this.showLidar) {
      this.drawLidar();
    }

    // 5. Robot Historical Trail & Forward Horizon
    if (this.showTrajectory) {
      this.drawRobotTrail();
      this.drawPredictiveHorizon();
    }

    // 6. Target Goal Flag
    this.drawGoal();

    // 7. Human Pedestrians
    for (let p of this.pedestrians) {
      this.drawPedestrian(p);
    }

    // 8. AMR Mobile Robot
    this.drawRobot();

    // 8.5. Active Rotation Gizmo Ring
    if (this.isRotatingRobot) {
      this.drawRotationGizmo(this.robot.x, this.robot.y, this.robot.radius + 18, this.robot.heading, '#00FF9D');
    } else if (this.rotatingPedestrian) {
      this.drawRotationGizmo(this.rotatingPedestrian.x, this.rotatingPedestrian.y, this.rotatingPedestrian.radius + 18, this.rotatingPedestrian.heading, '#00E5FF');
    } else if (this.rotatingObstacle) {
      this.drawRotationGizmo(this.rotatingObstacle.x, this.rotatingObstacle.y, (this.rotatingObstacle.radius || 25) + 16, this.lastRotateAngle, '#EAB308');
    }

    // 9. Paused Overlay Banner
    if (this.isPaused) {
      this.drawPausedBanner();
    }
  }

  computeCostmap() {
    const res = this.costmapResolution; // 0.2 meters
    const widthMeters = this.canvas.width / this.scale;
    const heightMeters = this.canvas.height / this.scale;
    const gridW = Math.floor(widthMeters / res);
    const gridH = Math.floor(heightMeters / res);
    const originX = -widthMeters / 2;
    const originY = -heightMeters / 2;

    const totalCells = gridW * gridH;
    if (!this.costmapData || this.costmapData.length !== totalCells) {
      this.costmapData = new Int8Array(totalCells);
    } else {
      this.costmapData.fill(0);
    }

    this.costmapInfo = {
      resolution: res,
      width: gridW,
      height: gridH,
      originX: originX,
      originY: originY
    };

    // Precompute pedestrian data in ROS frame for fast inner loop
    const pData = this.pedestrians.map(p => {
      const pos = ros2BridgeInstance.toROSCoords(p.x, p.y, this.canvas.width, this.canvas.height, this.scale);
      const theta = -p.heading;
      const speed = (Math.hypot(p.vx, p.vy) / this.scale) * 60; // m/s
      return {
        x: pos.x,
        y: pos.y,
        cosT: Math.cos(theta),
        sinT: Math.sin(theta),
        sigmaFront: 0.9 + 0.5 * Math.min(2.0, speed),
        sigmaBack: 0.45,
        sigmaY: 0.55
      };
    });

    // Precompute static obstacle data in ROS frame
    const obsData = this.obstacles.map(obs => {
      if (obs.type === 'rect') {
        const x1 = (obs.x - this.canvas.width / 2) / this.scale;
        const x2 = (obs.x + obs.width - this.canvas.width / 2) / this.scale;
        const y1 = -((obs.y + obs.height) - this.canvas.height / 2) / this.scale;
        const y2 = -(obs.y - this.canvas.height / 2) / this.scale;
        return {
          type: 'rect',
          minX: Math.min(x1, x2),
          maxX: Math.max(x1, x2),
          minY: Math.min(y1, y2),
          maxY: Math.max(y1, y2)
        };
      } else if (obs.type === 'polygon' && obs.points) {
        const ptsRos = obs.points.map(pt => {
          return {
            x: (obs.x + pt.x - this.canvas.width / 2) / this.scale,
            y: -((obs.y + pt.y) - this.canvas.height / 2) / this.scale
          };
        });
        const pos = ros2BridgeInstance.toROSCoords(obs.x, obs.y, this.canvas.width, this.canvas.height, this.scale);
        const r = (obs.radius || 25) / this.scale;
        return {
          type: 'polygon',
          x: pos.x,
          y: pos.y,
          r: r,
          pts: ptsRos
        };
      } else {
        const pos = ros2BridgeInstance.toROSCoords(obs.x, obs.y, this.canvas.width, this.canvas.height, this.scale);
        const r = (obs.radius || 20) / this.scale;
        return {
          type: 'circle',
          x: pos.x,
          y: pos.y,
          r: r,
          rInflated: r + 0.45
        };
      }
    });

    // Evaluate cost for each cell
    for (let gy = 0; gy < gridH; gy++) {
      const cellY = originY + (gy + 0.5) * res;
      const rowOffset = gy * gridW;

      for (let gx = 0; gx < gridW; gx++) {
        const cellX = originX + (gx + 0.5) * res;
        let cost = 0;

        // 1. Static obstacles layer (Circle Pillars, Rect Barriers, Polygons)
        for (let i = 0; i < obsData.length; i++) {
          const obs = obsData[i];
          if (obs.type === 'rect') {
            const dx = Math.max(obs.minX - cellX, 0, cellX - obs.maxX);
            const dy = Math.max(obs.minY - cellY, 0, cellY - obs.maxY);
            const dist = Math.hypot(dx, dy);

            if (dist === 0) {
              cost = 100;
              break;
            } else if (dist <= 0.45) {
              const inflCost = Math.floor(99 * Math.exp(-6 * dist));
              if (inflCost > cost) cost = inflCost;
            }
          } else if (obs.type === 'polygon') {
            const dCenter = Math.hypot(cellX - obs.x, cellY - obs.y);
            if (dCenter <= obs.r + 0.5) {
              const pts = obs.pts;
              const len = pts.length;
              let minDist = 999;
              for (let j = 0; j < len; j++) {
                const p1 = pts[j];
                const p2 = pts[(j + 1) % len];
                const seg = this.distToSegment(cellX, cellY, p1.x, p1.y, p2.x, p2.y);
                if (seg.dist < minDist) minDist = seg.dist;
              }
              if (minDist <= 0.15) {
                cost = 100;
                break;
              } else if (minDist <= 0.45) {
                const inflCost = Math.floor(99 * Math.exp(-6 * (minDist - 0.15)));
                if (inflCost > cost) cost = inflCost;
              }
            }
          } else {
            const d = Math.hypot(cellX - obs.x, cellY - obs.y);
            if (d <= obs.r) {
              cost = 100;
              break;
            } else if (d <= obs.rInflated) {
              const inflCost = Math.floor(99 * Math.exp(-6 * (d - obs.r)));
              if (inflCost > cost) cost = inflCost;
            }
          }
        }

        if (cost < 100) {
          // 2. Pedestrians Social Proxemics layer (Asymmetric Gaussian)
          for (let i = 0; i < pData.length; i++) {
            const p = pData[i];
            const dx = cellX - p.x;
            const dy = cellY - p.y;
            const dSq = dx * dx + dy * dy;

            // Physical body footprint
            if (dSq <= 0.12) { // ~0.35m radius
              cost = 100;
              break;
            }

            // Local rotated frame coordinates
            const xPrime = dx * p.cosT + dy * p.sinT;
            const yPrime = -dx * p.sinT + dy * p.cosT;

            const sigmaX = xPrime >= 0 ? p.sigmaFront : p.sigmaBack;
            const sigmaY = p.sigmaY;

            const exponent = -0.5 * ((xPrime / sigmaX) ** 2 + (yPrime / sigmaY) ** 2);
            if (exponent > -4.5) { // Only evaluate non-negligible cost
              const socialCost = Math.floor(90 * Math.exp(exponent));
              if (socialCost > cost) cost = socialCost;
            }
          }
        }

        this.costmapData[rowOffset + gx] = cost;
      }
    }

    // Publish to ROS2 bridge throttled at 5Hz
    const now = Date.now();
    if (now - this.lastCostmapPublishTime >= 200) {
      this.lastCostmapPublishTime = now;
      if (this.onCostmapUpdate) {
        this.onCostmapUpdate(this.costmapData, res, gridW, gridH, originX, originY);
      }
    }
  }

  drawCostmap() {
    if (!this.costmapData || !this.costmapInfo) return;

    const { resolution, width, height } = this.costmapInfo;
    const cellPx = resolution * this.scale; // e.g. 0.2 * 40 = 8px

    this.ctx.save();
    for (let gy = 0; gy < height; gy++) {
      const py = this.canvas.height - (gy + 1) * cellPx; // Invert for canvas
      const rowOffset = gy * width;

      for (let gx = 0; gx < width; gx++) {
        const cost = this.costmapData[rowOffset + gx];
        if (cost <= 6) continue; // Free space is transparent

        const px = gx * cellPx;

        if (cost >= 99) {
          // Lethal Obstacle / Body
          this.ctx.fillStyle = 'rgba(239, 68, 68, 0.7)';
        } else if (cost >= 65) {
          // Intimate / High Danger
          this.ctx.fillStyle = 'rgba(255, 0, 85, 0.5)';
        } else if (cost >= 35) {
          // Personal Space Warning
          this.ctx.fillStyle = 'rgba(245, 158, 11, 0.38)';
        } else {
          // Social Zone Field
          this.ctx.fillStyle = 'rgba(0, 229, 255, 0.25)';
        }

        this.ctx.fillRect(px, py, cellPx, cellPx);
      }
    }
    this.ctx.restore();
  }

  toggleCostmap(forceState = null) {
    this.showCostmap = forceState !== null ? forceState : !this.showCostmap;
    return this.showCostmap;
  }

  drawGrid() {
    const activeTheme = document.documentElement.getAttribute('data-theme');
    const isLight = ['light', 'solar_light', 'sakura_light', 'mint_light', 'coffee_latte'].includes(activeTheme);
    const step = this.scale; // 1-meter grid
    this.ctx.save();
    this.ctx.strokeStyle = isLight ? 'rgba(15, 23, 42, 0.09)' : 'rgba(255, 255, 255, 0.035)';
    this.ctx.lineWidth = 1;

    for (let x = 0; x < this.canvas.width; x += step) {
      this.ctx.beginPath();
      this.ctx.moveTo(x, 0);
      this.ctx.lineTo(x, this.canvas.height);
      this.ctx.stroke();
    }
    for (let y = 0; y < this.canvas.height; y += step) {
      this.ctx.beginPath();
      this.ctx.moveTo(0, y);
      this.ctx.lineTo(this.canvas.width, y);
      this.ctx.stroke();
    }

    // Scale reference text in bottom-right
    this.ctx.font = '10px "JetBrains Mono"';
    this.ctx.fillStyle = isLight ? 'rgba(15, 23, 42, 0.5)' : 'rgba(255, 255, 255, 0.25)';
    this.ctx.fillText(`Scale: 1m = ${this.scale}px`, this.canvas.width - 110, this.canvas.height - 12);
    this.ctx.restore();
  }

  drawStaticObstacles() {
    const activeTheme = document.documentElement.getAttribute('data-theme');
    const isLight = ['light', 'solar_light', 'sakura_light', 'mint_light', 'coffee_latte'].includes(activeTheme);

    for (let obs of this.obstacles) {
      this.ctx.save();
      if (obs.type === 'circle') {
        this.ctx.beginPath();
        this.ctx.arc(obs.x, obs.y, obs.radius, 0, Math.PI * 2);
        this.ctx.fillStyle = isLight ? '#e2e8f0' : '#161928';
        this.ctx.fill();
        this.ctx.strokeStyle = isLight ? '#94a3b8' : 'rgba(148, 163, 184, 0.6)';
        this.ctx.lineWidth = 2;
        this.ctx.shadowColor = isLight ? 'rgba(37, 99, 235, 0.2)' : 'rgba(0, 229, 255, 0.3)';
        this.ctx.shadowBlur = 10;
        this.ctx.stroke();

        this.ctx.beginPath();
        this.ctx.arc(obs.x, obs.y, obs.radius * 0.6, 0, Math.PI * 2);
        this.ctx.strokeStyle = isLight ? 'rgba(0, 0, 0, 0.1)' : 'rgba(255, 255, 255, 0.15)';
        this.ctx.stroke();

        this.ctx.font = '9px "JetBrains Mono"';
        this.ctx.fillStyle = isLight ? '#334155' : 'rgba(255, 255, 255, 0.6)';
        this.ctx.fillText(obs.label || 'PILLAR', obs.x - 18, obs.y + 3);
      } else if (obs.type === 'rect') {
        this.ctx.fillStyle = isLight ? '#e2e8f0' : '#161928';
        this.ctx.fillRect(obs.x, obs.y, obs.width, obs.height);
        this.ctx.strokeStyle = isLight ? '#94a3b8' : 'rgba(148, 163, 184, 0.6)';
        this.ctx.lineWidth = 2;
        this.ctx.shadowColor = isLight ? 'rgba(37, 99, 235, 0.2)' : 'rgba(0, 229, 255, 0.3)';
        this.ctx.shadowBlur = 10;
        this.ctx.strokeRect(obs.x, obs.y, obs.width, obs.height);

        this.ctx.strokeStyle = isLight ? 'rgba(0, 0, 0, 0.08)' : 'rgba(255, 255, 255, 0.08)';
        this.ctx.lineWidth = 1;
        for (let y = obs.y + 10; y < obs.y + obs.height; y += 16) {
          this.ctx.beginPath();
          this.ctx.moveTo(obs.x, y);
          this.ctx.lineTo(obs.x + obs.width, y);
          this.ctx.stroke();
        }

        this.ctx.save();
        this.ctx.translate(obs.x + obs.width / 2, obs.y + obs.height / 2);
        this.ctx.rotate(-Math.PI / 2);
        this.ctx.font = '9px "JetBrains Mono"';
        this.ctx.fillStyle = isLight ? '#334155' : 'rgba(255, 255, 255, 0.6)';
        this.ctx.fillText('BARRIER', -20, 3);
        this.ctx.restore();
      } else if (obs.type === 'polygon' && obs.points && obs.points.length > 2) {
        const pts = obs.points;
        this.ctx.beginPath();
        this.ctx.moveTo(obs.x + pts[0].x, obs.y + pts[0].y);
        for (let i = 1; i < pts.length; i++) {
          this.ctx.lineTo(obs.x + pts[i].x, obs.y + pts[i].y);
        }
        this.ctx.closePath();

        // Polygon Fill & Glowing Contour
        this.ctx.fillStyle = isLight ? '#e2e8f0' : '#141828';
        this.ctx.fill();
        this.ctx.strokeStyle = isLight ? '#0284c7' : 'rgba(0, 229, 255, 0.8)';
        this.ctx.lineWidth = 2.0;
        this.ctx.shadowColor = isLight ? 'rgba(37, 99, 235, 0.25)' : 'rgba(0, 229, 255, 0.4)';
        this.ctx.shadowBlur = 12;
        this.ctx.stroke();

        // Vertex Corner Accent Nodes
        for (let pt of pts) {
          this.ctx.beginPath();
          this.ctx.arc(obs.x + pt.x, obs.y + pt.y, 2.4, 0, Math.PI * 2);
          this.ctx.fillStyle = isLight ? '#0284c7' : '#00FF9D';
          this.ctx.fill();
        }

        // Polygon Center Label
        this.ctx.font = '700 8.5px "JetBrains Mono"';
        this.ctx.textAlign = 'center';
        this.ctx.textBaseline = 'middle';
        this.ctx.fillStyle = isLight ? '#334155' : 'rgba(255, 255, 255, 0.75)';
        this.ctx.shadowBlur = 0;
        this.ctx.fillText(obs.label || 'POLY', obs.x, obs.y);
      }
      this.ctx.restore();
    }
  }

  drawLidar() {
    const ox = this.robot.x;
    const oy = this.robot.y;

    this.ctx.save();

    // 1. Laser Rays (Electric Azure & Cobalt Blue 360° Raycast Fan)
    if (this.showLidarRays) {
      this.ctx.lineWidth = 1;
      for (let hit of this.laserHits) {
        this.ctx.beginPath();
        this.ctx.moveTo(ox, oy);
        this.ctx.lineTo(hit.x, hit.y);
        if (hit.type === 'human') {
          // Dynamic Human Encounter: Vibrant Cyan-Blue Ray
          this.ctx.strokeStyle = 'rgba(0, 210, 255, 0.18)';
        } else if (hit.type === 'obstacle') {
          // Static Barrier Encounter: Electric Azure Blue Ray
          this.ctx.strokeStyle = 'rgba(56, 189, 248, 0.14)';
        } else {
          // Free Space Max-Range: Deep Translucent Cobalt Blue
          this.ctx.strokeStyle = 'rgba(14, 165, 233, 0.055)';
        }
        this.ctx.stroke();
      }
    }

    // 2. Laser Point Cloud Hits (Illuminated Blue Return Dots)
    if (this.showLidarPoints) {
      for (let hit of this.laserHits) {
        if (hit.type !== 'max') {
          this.ctx.beginPath();
          this.ctx.arc(hit.x, hit.y, 2.5, 0, Math.PI * 2);
          if (hit.type === 'human') {
            // Human Return: Electric Cyan Blue with Soft Glow
            this.ctx.fillStyle = '#00E5FF';
            this.ctx.shadowColor = '#00E5FF';
          } else {
            // Obstacle/Wall Return: Vibrant Deep Azure Blue
            this.ctx.fillStyle = '#0099FF';
            this.ctx.shadowColor = '#0099FF';
          }
          this.ctx.shadowBlur = 8;
          this.ctx.fill();
        }
      }
    }

    this.ctx.restore();
  }

  drawProxemicsField(p) {
    this.ctx.save();
    this.ctx.translate(p.x, p.y);
    this.ctx.rotate(p.heading);

    const frontStretch = 1.0 + 0.6 * p.speed;
    const personalR = this.proxemicRadius * this.scale;

    // Social Zone (Outer - Cyan)
    this.ctx.beginPath();
    this.ctx.ellipse(8, 0, personalR * frontStretch * 1.3, personalR * 0.9, 0, 0, Math.PI * 2);
    this.ctx.fillStyle = 'rgba(0, 229, 255, 0.035)';
    this.ctx.strokeStyle = 'rgba(0, 229, 255, 0.15)';
    this.ctx.lineWidth = 1;
    this.ctx.fill();
    this.ctx.stroke();

    // Personal Zone (Middle - Amber)
    this.ctx.beginPath();
    this.ctx.ellipse(5, 0, personalR * 0.7 * frontStretch, personalR * 0.5, 0, 0, Math.PI * 2);
    this.ctx.fillStyle = 'rgba(245, 158, 11, 0.05)';
    this.ctx.strokeStyle = 'rgba(245, 158, 11, 0.25)';
    this.ctx.fill();
    this.ctx.stroke();

    // Intimate Zone (Inner - Red Warning)
    this.ctx.beginPath();
    this.ctx.ellipse(2, 0, 18 * frontStretch, 14, 0, 0, Math.PI * 2);
    this.ctx.fillStyle = 'rgba(255, 0, 85, 0.08)';
    this.ctx.strokeStyle = 'rgba(255, 0, 85, 0.4)';
    this.ctx.fill();
    this.ctx.stroke();

    this.ctx.restore();
  }

  drawGoal() {
    const time = Date.now() * 0.003;

    if (this.goalMode === 'multi' && this.waypoints.length > 0) {
      // 1. Draw Connective Flight Paths between Waypoints
      if (this.waypoints.length > 1) {
        this.ctx.save();
        this.ctx.setLineDash([4, 4]);
        this.ctx.strokeStyle = 'rgba(234, 179, 8, 0.45)';
        this.ctx.lineWidth = 1.8;
        this.ctx.beginPath();
        this.ctx.moveTo(this.waypoints[0].x, this.waypoints[0].y);
        for (let i = 1; i < this.waypoints.length; i++) {
          this.ctx.lineTo(this.waypoints[i].x, this.waypoints[i].y);
        }
        if (this.waypointLoop) {
          this.ctx.lineTo(this.waypoints[0].x, this.waypoints[0].y);
        }
        this.ctx.stroke();

        // Draw directional chevron midpoints
        for (let i = 0; i < this.waypoints.length; i++) {
          const nextIdx = (i + 1) % this.waypoints.length;
          if (nextIdx === 0 && !this.waypointLoop) continue;
          const p1 = this.waypoints[i];
          const p2 = this.waypoints[nextIdx];
          const mx = (p1.x + p2.x) / 2;
          const my = (p1.y + p2.y) / 2;
          const ang = Math.atan2(p2.y - p1.y, p2.x - p1.x);
          this.ctx.save();
          this.ctx.translate(mx, my);
          this.ctx.rotate(ang);
          this.ctx.fillStyle = '#EAB308';
          this.ctx.beginPath();
          this.ctx.moveTo(4, 0);
          this.ctx.lineTo(-3, -3);
          this.ctx.lineTo(-1, 0);
          this.ctx.lineTo(-3, 3);
          this.ctx.closePath();
          this.ctx.fill();
          this.ctx.restore();
        }
        this.ctx.restore();
      }

      // 2. Draw Each Waypoint Badge with Distinct Non-Duplicated Colors
      this.waypoints.forEach((wp, idx) => {
        const isActive = idx === this.activeWaypointIndex;
        const isVisited = idx < this.activeWaypointIndex;

        // Distinct Color Palette:
        // Active Goal: Vivid Electric Magenta (#FF007F)
        // Reached Goal: Cool Steel Slate Grey (#94A3B8) with checkmark
        // Pending Goal: Bright Golden Amber (#EAB308)
        let strokeColor = '#EAB308';
        let fillColor = 'rgba(234, 179, 8, 0.18)';
        let labelColor = '#EAB308';
        let labelText = `WP ${idx + 1}`;

        if (isActive) {
          strokeColor = '#FF007F';
          fillColor = 'rgba(255, 0, 127, 0.3)';
          labelColor = '#FF007F';
          labelText = `WP ${idx + 1} [ACTIVE]`;
        } else if (isVisited) {
          strokeColor = '#64748B';
          fillColor = 'rgba(100, 116, 139, 0.18)';
          labelColor = '#94A3B8';
          labelText = `WP ${idx + 1} ✓`;
        }

        this.ctx.save();
        this.ctx.translate(wp.x, wp.y);

        // Animated ripple ring for active target
        if (isActive) {
          const ringR = 18 + Math.sin(time) * 4;
          this.ctx.beginPath();
          this.ctx.arc(0, 0, ringR, 0, Math.PI * 2);
          this.ctx.strokeStyle = 'rgba(255, 0, 127, 0.55)';
          this.ctx.lineWidth = 1.8;
          this.ctx.stroke();
        }

        // Waypoint Outer Circle
        this.ctx.beginPath();
        this.ctx.arc(0, 0, wp.radius || 14, 0, Math.PI * 2);
        this.ctx.fillStyle = fillColor;
        this.ctx.fill();
        this.ctx.strokeStyle = strokeColor;
        this.ctx.lineWidth = isActive ? 2.5 : 1.8;
        this.ctx.shadowColor = strokeColor;
        this.ctx.shadowBlur = isActive ? 14 : 4;
        this.ctx.stroke();

        // Center dot
        this.ctx.beginPath();
        this.ctx.arc(0, 0, 3.5, 0, Math.PI * 2);
        this.ctx.fillStyle = isActive ? '#FFFFFF' : (isVisited ? '#94A3B8' : '#FFFFFF');
        this.ctx.fill();

        // Label Badge
        this.ctx.font = '700 9px "JetBrains Mono", monospace';
        this.ctx.textAlign = 'center';
        this.ctx.fillStyle = labelColor;
        this.ctx.shadowColor = isActive ? '#FF007F' : 'transparent';
        this.ctx.shadowBlur = isActive ? 8 : 0;
        this.ctx.fillText(labelText, 0, -18);

        this.ctx.restore();
      });
      return;
    }

    // Single Goal Mode (Standard)
    this.ctx.save();
    this.ctx.translate(this.goal.x, this.goal.y);

    // Animated ripple ring
    const ringR = 18 + Math.sin(time) * 4;

    this.ctx.beginPath();
    this.ctx.arc(0, 0, ringR, 0, Math.PI * 2);
    this.ctx.strokeStyle = 'rgba(234, 179, 8, 0.4)';
    this.ctx.lineWidth = 1.5;
    this.ctx.stroke();

    // Goal Target Circle
    this.ctx.beginPath();
    this.ctx.arc(0, 0, this.goal.radius, 0, Math.PI * 2);
    this.ctx.fillStyle = 'rgba(234, 179, 8, 0.2)';
    this.ctx.fill();
    this.ctx.strokeStyle = '#EAB308';
    this.ctx.lineWidth = 2;
    this.ctx.shadowColor = '#EAB308';
    this.ctx.shadowBlur = 12;
    this.ctx.stroke();

    // Center icon (Flag / Target Cross)
    this.ctx.beginPath();
    this.ctx.arc(0, 0, 4, 0, Math.PI * 2);
    this.ctx.fillStyle = '#FFFFFF';
    this.ctx.fill();

    this.ctx.font = '700 9px "JetBrains Mono"';
    this.ctx.fillStyle = '#EAB308';
    this.ctx.textAlign = 'center';
    this.ctx.fillText('GOAL', 0, -18);

    this.ctx.restore();
  }

  drawRobotTrail() {
    const hist = this.robot.history;
    if (hist.length < 2) return;
    this.ctx.save();
    const len = hist.length;

    // 1. Draw glowing segmented path with progressive alpha tapering
    for (let i = 0; i < len - 1; i++) {
      const progress = (i + 1) / len;
      const alpha = Math.max(0.04, progress * 0.65);
      this.ctx.beginPath();
      this.ctx.moveTo(hist[i].x, hist[i].y);
      this.ctx.lineTo(hist[i + 1].x, hist[i + 1].y);
      this.ctx.strokeStyle = `rgba(0, 255, 157, ${alpha.toFixed(3)})`;
      this.ctx.lineWidth = Math.max(1.2, progress * 2.5);
      this.ctx.stroke();
    }

    // 2. Subtle dashed cyber path centerline
    this.ctx.beginPath();
    this.ctx.moveTo(hist[0].x, hist[0].y);
    for (let i = 1; i < len; i++) {
      this.ctx.lineTo(hist[i].x, hist[i].y);
    }
    this.ctx.strokeStyle = 'rgba(0, 255, 157, 0.22)';
    this.ctx.lineWidth = 1.0;
    this.ctx.setLineDash([4, 4]);
    this.ctx.stroke();

    // 3. Small glowing historic waypoint dots every 10 samples
    for (let i = 0; i < len; i += 10) {
      const p = hist[i];
      const progress = (i + 1) / len;
      this.ctx.beginPath();
      this.ctx.arc(p.x, p.y, 1.8 * progress, 0, Math.PI * 2);
      this.ctx.fillStyle = `rgba(0, 255, 157, ${(progress * 0.5).toFixed(2)})`;
      this.ctx.fill();
    }

    this.ctx.restore();
  }

  drawPredictiveHorizon() {
    if (!this.mpcHorizonWaypoints || this.mpcHorizonWaypoints.length < 2) return;
    this.ctx.save();
    this.ctx.beginPath();
    this.ctx.moveTo(this.robot.x, this.robot.y);
    for (let wp of this.mpcHorizonWaypoints) {
      this.ctx.lineTo(wp.x, wp.y);
    }
    this.ctx.strokeStyle = 'rgba(0, 229, 255, 0.6)';
    this.ctx.lineWidth = 2;
    this.ctx.shadowColor = '#00E5FF';
    this.ctx.shadowBlur = 8;
    this.ctx.stroke();

    // Waypoint dots
    for (let wp of this.mpcHorizonWaypoints) {
      this.ctx.beginPath();
      this.ctx.arc(wp.x, wp.y, 2, 0, Math.PI * 2);
      this.ctx.fillStyle = '#00E5FF';
      this.ctx.fill();
    }
    this.ctx.restore();
  }

  drawFoot(ctx, x, y, isLeft, lift, color) {
    ctx.save();
    ctx.translate(x, y);

    // Natural human toe-out angle (~4 degrees)
    const toeOut = isLeft ? -0.07 : 0.07;
    ctx.rotate(toeOut);

    const scale = 1.0 + lift * 0.15;
    ctx.scale(scale, scale);

    // Anatomical Foot / Shoe Sole Shape
    ctx.beginPath();
    ctx.moveTo(-4.8, 0);
    ctx.bezierCurveTo(-4.8, -2.0, -3.8, -2.6, -2.2, -2.6);
    ctx.bezierCurveTo(0, isLeft ? -1.8 : -2.8, 2.5, -3.2, 4.6, -2.4);
    ctx.bezierCurveTo(6.0, -1.6, 6.0, 1.6, 4.6, 2.4);
    ctx.bezierCurveTo(2.5, 3.2, 0, isLeft ? 2.8 : 1.8, -2.2, 2.6);
    ctx.bezierCurveTo(-3.8, 2.6, -4.8, 2.0, -4.8, 0);
    ctx.closePath();

    // Sole Depth Fill & Glowing Border
    ctx.fillStyle = lift > 0.3 ? '#142c40' : '#081522';
    ctx.fill();

    ctx.strokeStyle = color;
    ctx.lineWidth = 1.4;
    if (lift > 0.25) {
      ctx.shadowColor = color;
      ctx.shadowBlur = 6;
    }
    ctx.stroke();

    // Toe Cap Highlight
    ctx.beginPath();
    ctx.arc(3.4, 0, 1.2, 0, Math.PI * 2);
    ctx.fillStyle = '#ffffff';
    ctx.fill();

    // Heel Grip Accent
    ctx.beginPath();
    ctx.arc(-2.4, 0, 0.9, 0, Math.PI * 2);
    ctx.fillStyle = color;
    ctx.fill();

    ctx.restore();
  }

  drawArrow(fromX, fromY, toX, toY, color = '#00E5FF', label = '', lineWidth = 2.2, headSize = 8) {
    const dx = toX - fromX;
    const dy = toY - fromY;
    const length = Math.hypot(dx, dy);
    if (length < 2) return;

    const angle = Math.atan2(dy, dx);
    this.ctx.save();

    // 1. Glowing Arrow Shaft
    this.ctx.beginPath();
    this.ctx.moveTo(fromX, fromY);
    this.ctx.lineTo(toX, toY);
    this.ctx.strokeStyle = color;
    this.ctx.lineWidth = lineWidth;
    this.ctx.shadowColor = color;
    this.ctx.shadowBlur = 8;
    this.ctx.stroke();

    // 2. Sharp Solid Arrowhead
    this.ctx.beginPath();
    this.ctx.moveTo(toX, toY);
    this.ctx.lineTo(
      toX - headSize * Math.cos(angle - Math.PI / 6),
      toY - headSize * Math.sin(angle - Math.PI / 6)
    );
    this.ctx.lineTo(
      toX - (headSize * 0.6) * Math.cos(angle),
      toY - (headSize * 0.6) * Math.sin(angle)
    );
    this.ctx.lineTo(
      toX - headSize * Math.cos(angle + Math.PI / 6),
      toY - headSize * Math.sin(angle + Math.PI / 6)
    );
    this.ctx.closePath();
    this.ctx.fillStyle = color;
    this.ctx.fill();

    // 3. Crisp Numeric HUD Speed Badge
    if (label) {
      this.ctx.font = '700 9px "JetBrains Mono", monospace';
      this.ctx.textAlign = 'center';
      this.ctx.textBaseline = 'middle';
      const labelX = toX + 16 * Math.cos(angle);
      const labelY = toY + 16 * Math.sin(angle);

      const txtWidth = this.ctx.measureText(label).width;
      this.ctx.fillStyle = 'rgba(7, 8, 12, 0.88)';
      this.ctx.fillRect(labelX - txtWidth / 2 - 4, labelY - 7, txtWidth + 8, 14);
      this.ctx.strokeStyle = color;
      this.ctx.lineWidth = 1;
      this.ctx.strokeRect(labelX - txtWidth / 2 - 4, labelY - 7, txtWidth + 8, 14);

      this.ctx.fillStyle = color;
      this.ctx.shadowBlur = 0;
      this.ctx.fillText(label, labelX, labelY);
    }

    this.ctx.restore();
  }

  drawPedestrian(p) {
    this.ctx.save();

    // Trajectory Tail
    if (p.history.length > 1) {
      this.ctx.beginPath();
      this.ctx.moveTo(p.history[0].x, p.history[0].y);
      for (let i = 1; i < p.history.length; i++) {
        this.ctx.lineTo(p.history[i].x, p.history[i].y);
      }
      this.ctx.strokeStyle = 'rgba(0, 229, 255, 0.15)';
      this.ctx.lineWidth = 1.5;
      this.ctx.stroke();
    }

    this.ctx.translate(p.x, p.y);
    this.ctx.rotate(p.heading);

    const speed = Math.hypot(p.vx, p.vy);
    const color = p.color || '#00E5FF';

    // 1. Soft Translucent Proxemic Torso / Aura Contour
    this.ctx.beginPath();
    this.ctx.ellipse(0, 0, p.radius + 2, p.radius - 2, 0, 0, Math.PI * 2);
    this.ctx.fillStyle = 'rgba(14, 30, 44, 0.45)';
    this.ctx.fill();
    this.ctx.strokeStyle = `rgba(${color === '#00FF9D' ? '0,255,157' : color === '#FFB800' ? '255,184,0' : '0,229,255'}, 0.25)`;
    this.ctx.lineWidth = 1;
    this.ctx.setLineDash([2, 2]);
    this.ctx.stroke();
    this.ctx.setLineDash([]);

    // 1b. Active Proxemic Violation Alert Halo (Amber for Personal, Coral-Red for Intimate)
    if (p.isInViolation) {
      const pulseTime = Date.now() * 0.008;
      const isIntimate = p.violationLevel === 'intimate';
      const warnColor = isIntimate ? '#FF0055' : '#F59E0B';
      const warnFill = isIntimate ? 'rgba(255, 0, 85, 0.22)' : 'rgba(245, 158, 11, 0.18)';
      const pulseR = p.radius + 5 + Math.sin(pulseTime) * 3;

      this.ctx.beginPath();
      this.ctx.arc(0, 0, pulseR, 0, Math.PI * 2);
      this.ctx.fillStyle = warnFill;
      this.ctx.fill();
      this.ctx.strokeStyle = warnColor;
      this.ctx.lineWidth = 1.8;
      this.ctx.shadowColor = warnColor;
      this.ctx.shadowBlur = 10;
      this.ctx.stroke();
      this.ctx.shadowBlur = 0;
    }

    // 2. Animated Walking Gait Calculations
    if (p.walkPhase === undefined) {
      p.walkPhase = (p.id ? p.id.charCodeAt(0) * 1.7 : Math.random() * Math.PI * 2);
    }

    if (speed > 0.05 && !this.isPaused) {
      p.walkPhase = (p.walkPhase + Math.max(0.06, speed * 0.26)) % (Math.PI * 2);
    }

    const isMoving = speed > 0.05;
    const stride = isMoving ? Math.min(8.5, Math.max(3.2, speed * 2.8)) : 0;
    const lateralSpacing = 5.2;

    // Stride offsets & Swing Lift Modulation
    const leftOffsetX = stride * Math.sin(p.walkPhase);
    const leftLift = isMoving ? Math.max(0, Math.cos(p.walkPhase)) : 0;

    const rightOffsetX = stride * Math.sin(p.walkPhase + Math.PI);
    const rightLift = isMoving ? Math.max(0, Math.cos(p.walkPhase + Math.PI)) : 0;

    // 3. Draw Left & Right Feet (Animated Moving Footsteps)
    this.drawFoot(this.ctx, leftOffsetX, -lateralSpacing, true, leftLift, color);
    this.drawFoot(this.ctx, rightOffsetX, lateralSpacing, false, rightLift, color);

    // 4. Directional Heading Sight Nose Pointer
    this.ctx.beginPath();
    this.ctx.moveTo(p.radius + 2, 0);
    this.ctx.lineTo(p.radius + 7, 0);
    this.ctx.strokeStyle = '#FFFFFF';
    this.ctx.lineWidth = 1.8;
    this.ctx.stroke();

    this.ctx.restore();

    // High-Visibility Velocity Vector Arrow
    if (this.showVectors) {
      const pSpeed = Math.hypot(p.vx, p.vy);
      if (pSpeed > 0.04) {
        const arrowScale = 22;
        const toX = p.x + p.vx * arrowScale;
        const toY = p.y + p.vy * arrowScale;
        const realSpeed = (pSpeed * (30 / this.scale)).toFixed(1);
        this.drawArrow(p.x, p.y, toX, toY, color, `${realSpeed} m/s`, 2.4, 8);
      }
    }
  }

  drawRobot() {
    this.ctx.save();
    this.ctx.translate(this.robot.x, this.robot.y);
    this.ctx.rotate(this.robot.heading);

    // Jackal AMR Chassis (Rectangular with rounded edges)
    const w = 26;
    const h = 20;

    this.ctx.beginPath();
    this.ctx.roundRect(-w / 2, -h / 2, w, h, 4);
    this.ctx.fillStyle = '#061a14';
    this.ctx.fill();
    this.ctx.strokeStyle = '#00FF9D';
    this.ctx.lineWidth = 2;
    this.ctx.shadowColor = '#00FF9D';
    this.ctx.shadowBlur = 12;
    this.ctx.stroke();

    // AMR 4 Wheels
    this.ctx.fillStyle = '#1e293b';
    this.ctx.fillRect(-w / 2 + 2, -h / 2 - 3, 7, 3);
    this.ctx.fillRect(w / 2 - 9, -h / 2 - 3, 7, 3);
    this.ctx.fillRect(-w / 2 + 2, h / 2, 7, 3);
    this.ctx.fillRect(w / 2 - 9, h / 2, 7, 3);

    // LiDAR Puck (Center top)
    this.ctx.beginPath();
    this.ctx.arc(0, 0, 5, 0, Math.PI * 2);
    this.ctx.fillStyle = '#0f766e';
    this.ctx.fill();
    this.ctx.strokeStyle = '#00FF9D';
    this.ctx.lineWidth = 1.5;
    this.ctx.stroke();

    // Directional Beam (Headlight)
    this.ctx.beginPath();
    this.ctx.moveTo(w / 2, 0);
    this.ctx.lineTo(w / 2 + 8, 0);
    this.ctx.strokeStyle = '#FFFFFF';
    this.ctx.lineWidth = 2.5;
    this.ctx.stroke();

    this.ctx.restore();

    // High-Visibility Robot Vectors (Velocity & Goal Direction)
    if (this.showVectors) {
      const rSpeed = Math.hypot(this.robot.vx, this.robot.vy);
      if (rSpeed > 0.04) {
        const arrowScale = 28;
        const toX = this.robot.x + this.robot.vx * arrowScale;
        const toY = this.robot.y + this.robot.vy * arrowScale;
        const realSpeed = (rSpeed * (30 / this.scale)).toFixed(1);
        this.drawArrow(this.robot.x, this.robot.y, toX, toY, '#00FF9D', `v: ${realSpeed} m/s`, 3.0, 9);
      }

      // Goal Attraction Guide Vector
      const activeG = this.getActiveGoal();
      const gx = activeG.x - this.robot.x;
      const gy = activeG.y - this.robot.y;
      const gDist = Math.hypot(gx, gy);
      if (gDist > 20) {
        const guideLen = Math.min(50, gDist * 0.45);
        const guideX = this.robot.x + (gx / gDist) * guideLen;
        const guideY = this.robot.y + (gy / gDist) * guideLen;
        const guideColor = this.goalMode === 'multi' ? 'rgba(255, 0, 127, 0.85)' : 'rgba(234, 179, 8, 0.75)';
        this.ctx.save();
        this.ctx.setLineDash([3, 3]);
        this.drawArrow(this.robot.x, this.robot.y, guideX, guideY, guideColor, '', 1.8, 7);
        this.ctx.restore();
      }
    }
  }

  drawPausedBanner() {
    this.ctx.save();
    this.ctx.fillStyle = 'rgba(7, 8, 12, 0.65)';
    this.ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);

    this.ctx.font = '700 20px "Space Grotesk", sans-serif';
    this.ctx.fillStyle = '#00FF9D';
    this.ctx.textAlign = 'center';
    this.ctx.shadowColor = '#00FF9D';
    this.ctx.shadowBlur = 12;
    this.ctx.fillText('⏸ SIMULATION PAUSED', this.canvas.width / 2, this.canvas.height / 2 - 10);

    this.ctx.font = '12px "JetBrains Mono"';
    this.ctx.fillStyle = 'rgba(255, 255, 255, 0.7)';
    this.ctx.shadowBlur = 0;
    this.ctx.fillText('Click Play/Pause or type "sim resume" in CLI to continue', this.canvas.width / 2, this.canvas.height / 2 + 18);
    this.ctx.restore();
  }

  updateMetricsUI() {
    const elCompliance = document.getElementById('sim-metric-compliance');
    const elViolations = document.getElementById('sim-metric-violations');
    const elMinDist = document.getElementById('sim-metric-mindist');
    const elVelocity = document.getElementById('sim-metric-velocity');
    const elComfort = document.getElementById('sim-metric-comfort');
    const elPose = document.getElementById('sim-metric-pose');

    if (elCompliance) elCompliance.textContent = `${this.complianceScore}%`;
    if (elViolations) elViolations.textContent = this.violationsCount;
    if (elMinDist) elMinDist.textContent = `${this.minDistanceToHuman}m`;
    if (elComfort) elComfort.textContent = `${this.comfortIndex}%`;
    
    const v = (Math.hypot(this.robot.vx, this.robot.vy) * (30 / this.scale)).toFixed(2);
    if (elVelocity) elVelocity.textContent = `${v} m/s`;
    const speedFill = document.getElementById('robot-speed-fill');
    if (speedFill) {
      const speedPct = Math.min(100, (parseFloat(v) / (this.robotMaxSpeed || 1.2)) * 100);
      speedFill.style.width = `${speedPct.toFixed(0)}%`;
    }

    // Push real-time telemetry stream to Analytics Engine
    telemetryAnalytics.pushTelemetry({
      complianceScore: this.complianceScore,
      comfortIndex: this.comfortIndex,
      minDistanceToHuman: this.minDistanceToHuman,
      violationsCount: this.violationsCount,
      velocity: parseFloat(v)
    });

    const robotRos = ros2BridgeInstance.toROSCoords(this.robot.x, this.robot.y, this.canvas.width, this.canvas.height, this.scale);
    let yawDeg = -((this.robot.heading * 180 / Math.PI) % 360);
    if (yawDeg < -180) yawDeg += 360;
    if (yawDeg > 180) yawDeg -= 360;

    if (elPose) {
      elPose.textContent = `x: ${robotRos.x.toFixed(2)}m, y: ${robotRos.y.toFixed(2)}m, θ: ${yawDeg.toFixed(0)}°`;
    }

    // Direct real-time updates for ROS2 Topic preview cards
    const prevRobot = document.getElementById('preview-robot-pose');
    const prevHumans = document.getElementById('preview-humans-pose');
    const prevGoal = document.getElementById('preview-goal-pose');

    if (prevRobot) {
      prevRobot.textContent = `x: ${robotRos.x.toFixed(2)}m, y: ${robotRos.y.toFixed(2)}m, yaw: ${yawDeg.toFixed(1)}°`;
    }
    if (prevHumans) {
      prevHumans.textContent = `Tracking ${this.pedestrians.length} active pedestrians`;
    }
    if (prevGoal) {
      const goalRos = ros2BridgeInstance.toROSCoords(this.goal.x, this.goal.y, this.canvas.width, this.canvas.height, this.scale);
      prevGoal.textContent = `Target: x: ${goalRos.x.toFixed(2)}m, y: ${goalRos.y.toFixed(2)}m`;
    }
  }

  getStatus() {
    const v = +(Math.hypot(this.robot.vx, this.robot.vy) * (30 / this.scale)).toFixed(2);
    const rosCoords = ros2BridgeInstance.toROSCoords(this.robot.x, this.robot.y, this.canvas.width, this.canvas.height, this.scale);
    let yawDeg = -((this.robot.heading * 180 / Math.PI) % 360);
    if (yawDeg < -180) yawDeg += 360;
    if (yawDeg > 180) yawDeg -= 360;

    return {
      algorithm: this.algorithm,
      scenario: this.currentScenario,
      isPaused: this.isPaused,
      pedestrianCount: this.pedestrians.length,
      robotSpeed: this.robotMaxSpeed,
      courtesyWeight: this.courtesyWeight,
      complianceScore: this.complianceScore,
      violationsCount: this.violationsCount,
      minDistanceToHuman: this.minDistanceToHuman,
      comfortIndex: this.comfortIndex,
      currentVelocity: v,
      robotPose: {
        x: rosCoords.x,
        y: rosCoords.y,
        yawDeg: +yawDeg.toFixed(2)
      }
    };
  }

  startLoop() {
    let frameCount = 0;
    let lastFpsUpdate = performance.now();
    const elFps = document.getElementById('hud-fps-val');

    const loop = (timestamp) => {
      try {
        if (!this.isPaused) {
          this.updatePedestrians();
          this.updateRobot();
        }
        this.draw();

        // Calculate and stream live playground FPS
        frameCount++;
        if (timestamp - lastFpsUpdate >= 350) {
          const fps = Math.min(60, Math.round((frameCount * 1000) / (timestamp - lastFpsUpdate)));
          if (elFps) elFps.textContent = `${fps} FPS`;
          frameCount = 0;
          lastFpsUpdate = timestamp;
        }

        // Publish live state & LiDAR scan to ROS2 Bridge
        ros2BridgeInstance.publishSimState(
          this.robot,
          this.pedestrians,
          this.goal,
          this.obstacles,
          this.laserScanData,
          this.canvas.width,
          this.canvas.height,
          this.scale
        );

        if (this.totalSteps % 4 === 0) {
          this.updateMetricsUI();
        }
      } catch (err) {
        console.error("Simulation loop error:", err);
      }
      this.animationFrameId = requestAnimationFrame(loop);
    };
    this.animationFrameId = requestAnimationFrame(loop);
  }

  destroy() {
    if (this.animationFrameId) {
      cancelAnimationFrame(this.animationFrameId);
    }
  }
}

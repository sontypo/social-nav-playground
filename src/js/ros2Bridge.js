// ROS2 WebSocket Bridge Manager (Compatible with rosbridge_suite / rosbridge_websocket)

export class ROS2Bridge {
  constructor() {
    this.wsUrl = 'ws://localhost:9090';
    this.socket = null;
    this.isConnected = false;
    this.isConnecting = false;
    this.publishRateHz = 20; // 20 Hz
    this.lastPublishTime = 0;
    this.packetsSent = 0;

    // Callbacks for UI & Simulation
    this.onStatusChange = null;
    this.onPacketSent = null;
    this.onGoalReceived = null;
    this.onInitialPoseReceived = null;
    this.onCmdVelReceived = null;

    // Track our own published goal to avoid self-echo loops
    this.lastWebGoalPos = null;

    // Topic configurations (Conforming to Standard ROS2 Nav2 Ecosystem)
    this.topics = {
      robotPose: '/robot_pose',
      robotOdom: '/odom',
      trackedHumans: '/tracked_humans',
      goalPose: '/goal_pose',
      initialPose: '/initialpose',
      clickedPoint: '/clicked_point',
      moveBaseGoal: '/move_base_simple/goal',
      laserScan: '/scan',
      socialCostmap: '/social_costmap',
      cmdVel: '/cmd_vel',
      metrics: '/social_nav/metrics'
    };
  }

  connect(url = 'ws://localhost:9090') {
    this.wsUrl = url;
    if (this.socket) {
      try { this.socket.close(); } catch (e) {}
    }

    this.isConnecting = true;
    this.notifyStatus('CONNECTING', 'Connecting to ' + url + '...');

    try {
      this.socket = new WebSocket(this.wsUrl);

      this.socket.onopen = () => {
        this.isConnected = true;
        this.isConnecting = false;
        this.notifyStatus('CONNECTED', `Connected to ${this.wsUrl}`);
        this.advertiseTopics();
        this.subscribeTopics();
      };

      this.socket.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          if (data.op === 'publish') {
            const topic = data.topic;
            const msg = data.msg;

            // 1. Handle RViz2 2D Goal Pose (/goal_pose or /move_base_simple/goal)
            if ((topic === this.topics.goalPose || topic === this.topics.moveBaseGoal) && msg) {
              const pos = msg.pose?.position || msg.pose?.pose?.position;
              if (pos && typeof pos.x === 'number' && typeof pos.y === 'number') {
                // Check if this is an echo of our own web-published goal
                if (this.lastWebGoalPos) {
                  const dx = Math.abs(pos.x - this.lastWebGoalPos.x);
                  const dy = Math.abs(pos.y - this.lastWebGoalPos.y);
                  if (dx < 0.02 && dy < 0.02) {
                    return; // Ignore self-echo
                  }
                }

                if (this.onGoalReceived) {
                  this.onGoalReceived(pos.x, pos.y, 'RViz2 2D Goal Pose');
                }
              }
            }

            // 2. Handle Clicked Point (/clicked_point)
            else if (topic === this.topics.clickedPoint && msg && msg.point) {
              const pt = msg.point;
              if (this.onGoalReceived) {
                this.onGoalReceived(pt.x, pt.y, 'RViz2 Clicked Point');
              }
            }

            // 3. Handle 2D Pose Estimate (/initialpose)
            else if (topic === this.topics.initialPose && msg) {
              const pos = msg.pose?.pose?.position || msg.pose?.position;
              const ori = msg.pose?.pose?.orientation || msg.pose?.orientation;
              if (pos && typeof pos.x === 'number' && typeof pos.y === 'number') {
                let yaw = 0;
                if (ori && typeof ori.z === 'number' && typeof ori.w === 'number') {
                  yaw = 2 * Math.atan2(ori.z, ori.w);
                }
                if (this.onInitialPoseReceived) {
                  this.onInitialPoseReceived(pos.x, pos.y, yaw);
                }
              }
            }

            // 4. Handle External Velocity Commands (/cmd_vel)
            else if (topic === this.topics.cmdVel && msg) {
              const linearX = typeof msg.linear?.x === 'number' ? msg.linear.x : 0.0;
              const angularZ = typeof msg.angular?.z === 'number' ? msg.angular.z : 0.0;
              if (this.onCmdVelReceived) {
                this.onCmdVelReceived(linearX, angularZ);
              }
            }
          }
        } catch (e) {
          console.warn("WebSocket parse warning:", e);
        }
      };

      this.socket.onclose = () => {
        this.isConnected = false;
        this.isConnecting = false;
        this.notifyStatus('DISCONNECTED', 'Disconnected from ROS2 Bridge');
      };

      this.socket.onerror = (err) => {
        this.isConnected = false;
        this.isConnecting = false;
        this.notifyStatus('ERROR', 'Connection failed (Ensure rosbridge_server is running on ' + this.wsUrl + ')');
      };
    } catch (e) {
      this.isConnected = false;
      this.isConnecting = false;
      this.notifyStatus('ERROR', e.message);
    }
  }

  disconnect() {
    if (this.socket) {
      try { this.socket.close(); } catch (e) {}
    }
    this.isConnected = false;
    this.isConnecting = false;
    this.notifyStatus('DISCONNECTED', 'Disconnected');
  }

  advertiseTopics() {
    if (!this.isConnected || !this.socket) return;

    const ads = [
      { op: 'advertise', topic: this.topics.robotPose, type: 'geometry_msgs/msg/PoseStamped' },
      { op: 'advertise', topic: this.topics.trackedHumans, type: 'geometry_msgs/msg/PoseArray' },
      { op: 'advertise', topic: this.topics.goalPose, type: 'geometry_msgs/msg/PoseStamped' },
      { op: 'advertise', topic: this.topics.laserScan, type: 'sensor_msgs/msg/LaserScan' },
      { op: 'advertise', topic: this.topics.robotOdom, type: 'nav_msgs/msg/Odometry' },
      { op: 'advertise', topic: this.topics.socialCostmap, type: 'nav_msgs/msg/OccupancyGrid' }
    ];

    ads.forEach(ad => {
      if (this.socket.readyState === WebSocket.OPEN) {
        this.socket.send(JSON.stringify(ad));
      }
    });
  }

  subscribeTopics() {
    if (!this.isConnected || !this.socket) return;

    // Subscribe to RViz2 Nav2 Goal, Point, InitialPose, and Velocity Controller topics
    const subs = [
      { op: 'subscribe', topic: this.topics.goalPose, type: 'geometry_msgs/msg/PoseStamped' },
      { op: 'subscribe', topic: this.topics.moveBaseGoal, type: 'geometry_msgs/msg/PoseStamped' },
      { op: 'subscribe', topic: this.topics.clickedPoint, type: 'geometry_msgs/msg/PointStamped' },
      { op: 'subscribe', topic: this.topics.initialPose, type: 'geometry_msgs/msg/PoseWithCovarianceStamped' },
      { op: 'subscribe', topic: this.topics.cmdVel, type: 'geometry_msgs/msg/Twist' }
    ];

    subs.forEach(sub => {
      if (this.socket.readyState === WebSocket.OPEN) {
        this.socket.send(JSON.stringify(sub));
      }
    });
  }

  // Convert ROS2 metric coordinates (meters, center origin) back to Canvas pixel coordinates
  toCanvasCoords(rosX, rosY, canvasWidth, canvasHeight, scale = 40) {
    const px = rosX * scale + canvasWidth / 2;
    const py = -rosY * scale + canvasHeight / 2; // Invert Y
    return {
      x: Math.max(25, Math.min(canvasWidth - 25, px)),
      y: Math.max(25, Math.min(canvasHeight - 25, py))
    };
  }

  // Convert canvas pixel coordinates to ROS2 metric coordinates (meters, center origin)
  toROSCoords(px, py, canvasWidth, canvasHeight, scale = 40) {
    const x = (px - canvasWidth / 2) / scale;
    const y = -(py - canvasHeight / 2) / scale; // Invert Y for ROS Cartesian frame
    return { x: +x.toFixed(3), y: +y.toFixed(3), z: 0.0 };
  }

  toRosCoords(px, py, canvasWidth, canvasHeight, scale = 40) {
    return this.toROSCoords(px, py, canvasWidth, canvasHeight, scale);
  }

  // Convert 2D angle to ROS Quaternion
  toQuaternion(heading) {
    const rosHeading = -heading; // Invert yaw for ROS frame
    const qz = Math.sin(rosHeading / 2);
    const qw = Math.cos(rosHeading / 2);
    return { x: 0.0, y: 0.0, z: +qz.toFixed(4), w: +qw.toFixed(4) };
  }

  // Publish Goal Pose when set on web (latched event)
  publishGoal(goalX, goalY, canvasWidth = 800, canvasHeight = 500, scale = 40) {
    const goalPos = this.toROSCoords(goalX, goalY, canvasWidth, canvasHeight, scale);
    this.lastWebGoalPos = goalPos;

    const timeSec = Math.floor(Date.now() / 1000);
    const timeNsec = (Date.now() % 1000) * 1000000;

    const goalPoseMsg = {
      op: 'publish',
      topic: this.topics.goalPose,
      msg: {
        header: { stamp: { sec: timeSec, nanosec: timeNsec }, frame_id: 'map' },
        pose: {
          position: goalPos,
          orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
        }
      }
    };

    if (this.isConnected && this.socket && this.socket.readyState === WebSocket.OPEN) {
      this.socket.send(JSON.stringify(goalPoseMsg));
    }
  }

  // Publish dynamic Social Costmap (nav_msgs/msg/OccupancyGrid)
  publishCostmap(gridData, resolution = 0.1, width = 100, height = 75, originX = -10, originY = -6.25) {
    if (!this.isConnected || !this.socket || this.socket.readyState !== WebSocket.OPEN) return;

    const now = Date.now();
    const timeSec = Math.floor(now / 1000);
    const timeNsec = (now % 1000) * 1000000;

    // Convert Int8Array or regular array to plain array if needed
    const dataArray = Array.isArray(gridData) ? gridData : Array.from(gridData);

    const costmapMsg = {
      op: 'publish',
      topic: this.topics.socialCostmap,
      msg: {
        header: {
          stamp: { sec: timeSec, nanosec: timeNsec },
          frame_id: 'map'
        },
        info: {
          map_load_time: { sec: timeSec, nanosec: timeNsec },
          resolution: resolution,
          width: width,
          height: height,
          origin: {
            position: { x: originX, y: originY, z: 0.0 },
            orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
          }
        },
        data: dataArray
      }
    };

    this.socket.send(JSON.stringify(costmapMsg));
    this.packetsSent++;
  }

  // Publish dynamic simulation entities (Robot pose, humans, LiDAR scan)
  publishSimState(robot, pedestrians, goal, obstacles = [], laserScanData = null, canvasWidth = 800, canvasHeight = 500, scale = 40) {
    const now = Date.now();
    const intervalMs = 1000 / this.publishRateHz;

    if (now - this.lastPublishTime < intervalMs) {
      return;
    }
    this.lastPublishTime = now;

    const timeSec = Math.floor(now / 1000);
    const timeNsec = (now % 1000) * 1000000;

    const header = {
      stamp: { sec: timeSec, nanosec: timeNsec },
      frame_id: 'map'
    };

    // 1. Robot PoseStamped (/robot_pose)
    const robotPos = this.toROSCoords(robot.x, robot.y, canvasWidth, canvasHeight, scale);
    const robotQuat = this.toQuaternion(robot.heading);
    const robotPoseMsg = {
      op: 'publish',
      topic: this.topics.robotPose,
      msg: {
        header: header,
        pose: {
          position: robotPos,
          orientation: robotQuat
        }
      }
    };

    // 2. Robot Odometry (/odom)
    const robotOdomMsg = {
      op: 'publish',
      topic: this.topics.robotOdom,
      msg: {
        header: { stamp: { sec: timeSec, nanosec: timeNsec }, frame_id: 'odom' },
        child_frame_id: 'base_link',
        pose: {
          pose: {
            position: robotPos,
            orientation: robotQuat
          },
          covariance: [
            0.001, 0, 0, 0, 0, 0,
            0, 0.001, 0, 0, 0, 0,
            0, 0, 1000000, 0, 0, 0,
            0, 0, 0, 1000000, 0, 0,
            0, 0, 0, 0, 1000000, 0,
            0, 0, 0, 0, 0, 0.001
          ]
        },
        twist: {
          twist: {
            linear: { x: +((robot.vx || 0) / scale).toFixed(3), y: +(-(robot.vy || 0) / scale).toFixed(3), z: 0.0 },
            angular: { x: 0.0, y: 0.0, z: 0.0 }
          },
          covariance: [
            0.001, 0, 0, 0, 0, 0,
            0, 0.001, 0, 0, 0, 0,
            0, 0, 1000000, 0, 0, 0,
            0, 0, 0, 1000000, 0, 0,
            0, 0, 0, 0, 1000000, 0,
            0, 0, 0, 0, 0, 0.001
          ]
        }
      }
    };

    // 3. Tracked Humans PoseArray (/tracked_humans)
    const humanPoses = pedestrians.map(p => ({
      position: this.toROSCoords(p.x, p.y, canvasWidth, canvasHeight, scale),
      orientation: this.toQuaternion(p.heading)
    }));

    const humansPoseArrayMsg = {
      op: 'publish',
      topic: this.topics.trackedHumans,
      msg: {
        header: header,
        poses: humanPoses
      }
    };

    // 4. LiDAR LaserScan Message (/scan)
    let laserScanMsg = null;
    if (laserScanData) {
      laserScanMsg = {
        op: 'publish',
        topic: this.topics.laserScan,
        msg: {
          header: { stamp: { sec: timeSec, nanosec: timeNsec }, frame_id: 'laser_link' },
          angle_min: laserScanData.angleMin,
          angle_max: laserScanData.angleMax,
          angle_increment: laserScanData.angleIncrement,
          time_increment: 0.0,
          scan_time: +(1.0 / this.publishRateHz).toFixed(4),
          range_min: 0.1,
          range_max: laserScanData.rangeMax,
          ranges: laserScanData.ranges
        }
      };
    }

    // Send high-rate telemetry packets
    if (this.isConnected && this.socket && this.socket.readyState === WebSocket.OPEN) {
      this.socket.send(JSON.stringify(robotPoseMsg));
      this.socket.send(JSON.stringify(robotOdomMsg));
      this.socket.send(JSON.stringify(humansPoseArrayMsg));
      if (laserScanMsg) {
        this.socket.send(JSON.stringify(laserScanMsg));
        this.packetsSent += 4;
      } else {
        this.packetsSent += 3;
      }

      if (this.onPacketSent) {
        this.onPacketSent({
          packetsSent: this.packetsSent,
          robotPos,
          humanCount: pedestrians.length,
          obstacleCount: obstacles.length,
          rateHz: this.publishRateHz
        });
      }
    }

    // Keep cached last state
    const goalPos = this.toROSCoords(goal.x, goal.y, canvasWidth, canvasHeight, scale);
    this.lastCachedState = {
      robotPoseMsg,
      robotOdomMsg,
      humansPoseArrayMsg,
      laserScanMsg,
      laserScanData,
      robotPos,
      robotQuat,
      humanPoses,
      goalPos
    };
  }

  notifyStatus(status, message) {
    if (this.onStatusChange) {
      this.onStatusChange(status, message, this.isConnected);
    }
  }
}

export const ros2BridgeInstance = new ROS2Bridge();

// ROS2 WebSocket Bridge Manager (Compatible with rosbridge_suite / rosbridge_websocket)

export class ROS2Bridge {
  constructor(options = {}) {
    this.isIngestMode = !!options.isIngestMode; // True when used on live.html
    this.autoReconnect = options.autoReconnect !== false;
    this.reconnectInterval = options.reconnectInterval || 3000;
    this.reconnectTimer = null;

    this.wsUrl = options.wsUrl || 'ws://localhost:9090';
    this.socket = null;
    this.isConnected = false;
    this.isConnecting = false;
    this.publishRateHz = 20; // 20 Hz
    this.lastPublishTime = 0;
    this.packetsSent = 0;
    this.packetsReceived = 0;

    // Callbacks for UI & Simulation
    this.onStatusChange = null;
    this.onPacketSent = null;
    this.onPacketReceived = null;
    this.onGoalReceived = null;
    this.onInitialPoseReceived = null;
    this.onCmdVelReceived = null;

    // Dynamic Topic Subscription Callbacks (for Live Ingest Mode)
    this.customSubscribers = new Map(); // topicName -> Array of callback functions
    this.topicStats = new Map(); // topicName -> { hz, count, lastTime, lastMsg, type }

    // Service Call Handlers
    this.pendingServices = new Map(); // id -> callback

    // Discovered Topics & Resolved Mappings
    this.discoveredTopics = {}; // { topicName: msgType }
    this.resolvedMapping = {};  // { roleKey: { topic, type, candidates, autoMatched } }
    this.onTopicsDiscovered = null; // (mapping, allTopics) => {}
    this.removedTopics = new Set(); // Permanently ignored/unsubscribed topics

    try {
      this.autoDetectTopics = localStorage.getItem('socialnav_auto_detect_topics') !== 'false';
    } catch {
      this.autoDetectTopics = true;
    }

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
      pointCloud: '/velodyne_points',
      socialCostmap: '/social_costmap',
      cmdVel: '/cmd_vel',
      metrics: '/social_nav/metrics',
      map: '/map',
      globalCostmap: '/global_costmap/costmap',
      localCostmap: '/local_costmap/costmap',
      globalPlan: '/plan',
      localPlan: '/local_plan',
      cameraCompressed: '/camera/image_raw/compressed',
      battery: '/battery_state',
      imu: '/imu/data',
      diagnostics: '/diagnostics'
    };
  }

  connect(url = null) {
    if (url) this.wsUrl = url;
    if (this.socket) {
      try { this.socket.close(); } catch (e) {}
    }

    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }

    this.isConnecting = true;
    this.notifyStatus('CONNECTING', 'Connecting to ' + this.wsUrl + '...');

    try {
      this.socket = new WebSocket(this.wsUrl);

      this.socket.onopen = () => {
        this.isConnected = true;
        this.isConnecting = false;
        if (this.reconnectTimer) {
          clearTimeout(this.reconnectTimer);
          this.reconnectTimer = null;
        }
        this.notifyStatus('CONNECTED', `Connected to ${this.wsUrl}`);

        if (!this.isIngestMode) {
          this.advertiseTopics();
        }
        this.subscribeTopics();
        this.resubscribeCustom();

        // Automatically query ROS2 topics and message types if auto-detect option is enabled
        if (this.autoDetectTopics) {
          setTimeout(() => {
            this.discoverTopicsAndTypes();
          }, 300);
        }
      };

      this.socket.onmessage = (event) => {
        try {
          const rawLength = event.data ? event.data.length : 0;
          const data = JSON.parse(event.data);
          this.packetsReceived++;

          // 1. Handle Service Call Responses (/rosapi)
          if (data.op === 'service_response') {
            const cb = this.pendingServices.get(data.id);
            if (cb) {
              this.pendingServices.delete(data.id);
              cb(data.values, data.result !== false);
            }
            return;
          }

          // 2. Handle Published Topic Data
          if (data.op === 'publish') {
            const topic = data.topic;

            // Completely ignore messages for user-removed topics
            if (this.removedTopics.has(topic)) {
              return;
            }

            const msg = data.msg;

            // Track topic statistics for inspector without JSON.stringify overhead
            this.recordTopicStat(topic, msg, rawLength);

            // Execute custom subscribers if any
            if (this.customSubscribers.has(topic)) {
              const cbs = this.customSubscribers.get(topic);
              for (let i = 0; i < cbs.length; i++) {
                try { cbs[i](msg, topic); } catch (err) { console.error(`Error in subscriber for ${topic}:`, err); }
              }
            }

            if (this.onPacketReceived) {
              this.onPacketReceived(topic, msg);
            }

            // 1. Handle RViz2 2D Goal Pose (/goal_pose or /move_base_simple/goal)
            if ((topic === this.topics.goalPose || topic === this.topics.moveBaseGoal) && msg) {
              const pos = msg.pose?.position || msg.pose?.pose?.position;
              if (pos && typeof pos.x === 'number' && typeof pos.y === 'number') {
                if (this.lastWebGoalPos) {
                  const dx = Math.abs(pos.x - this.lastWebGoalPos.x);
                  const dy = Math.abs(pos.y - this.lastWebGoalPos.y);
                  if (dx < 0.02 && dy < 0.02) return; // Ignore self-echo
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
        if (this.autoReconnect && !this.reconnectTimer) {
          this.reconnectTimer = setTimeout(() => {
            this.reconnectTimer = null;
            if (!this.isConnected) {
              this.connect(this.wsUrl);
            }
          }, this.reconnectInterval);
        }
      };

      this.socket.onerror = (err) => {
        this.isConnected = false;
        this.isConnecting = false;
        this.notifyStatus('ERROR', 'Connection failed (' + this.wsUrl + ')');
        if (this.autoReconnect && !this.reconnectTimer) {
          this.reconnectTimer = setTimeout(() => {
            this.reconnectTimer = null;
            if (!this.isConnected) {
              this.connect(this.wsUrl);
            }
          }, this.reconnectInterval);
        }
      };
    } catch (e) {
      this.isConnected = false;
      this.isConnecting = false;
      this.notifyStatus('ERROR', e.message);
    }
  }

  recordTopicStat(topic, msg, rawLength = 0) {
    if (this.removedTopics.has(topic)) return;

    const now = performance.now();
    const bytes = rawLength || (msg?.data ? (typeof msg.data === 'string' ? msg.data.length : msg.data.length * 4) : 256);

    if (!this.topicStats.has(topic)) {
      this.topicStats.set(topic, {
        count: 1,
        lastTime: now,
        hz: 0,
        hzAccum: [],
        lastMsg: msg,
        bytes: bytes
      });
    } else {
      const stat = this.topicStats.get(topic);
      const dt = (now - stat.lastTime) / 1000;
      stat.count++;
      if (dt > 0.001) {
        const instantHz = 1 / dt;
        stat.hzAccum.push(instantHz);
        if (stat.hzAccum.length > 8) stat.hzAccum.shift();
        stat.hz = +(stat.hzAccum.reduce((a, b) => a + b, 0) / stat.hzAccum.length).toFixed(1);
      }
      stat.lastTime = now;
      stat.lastMsg = msg;
      stat.bytes = bytes;
    }
  }

  subscribeCustom(topic, type, callback) {
    this.removedTopics.delete(topic);

    if (!this.customSubscribers.has(topic)) {
      this.customSubscribers.set(topic, []);
    }
    if (callback) {
      this.customSubscribers.get(topic).push(callback);
    }

    if (this.isConnected && this.socket && this.socket.readyState === WebSocket.OPEN) {
      const sub = { op: 'subscribe', topic: topic, throttle_rate: 0, queue_length: 1 };
      this.socket.send(JSON.stringify(sub));
    }
  }

  resubscribeCustom() {
    if (!this.isConnected || !this.socket || this.socket.readyState !== WebSocket.OPEN) return;
    for (const [topic] of this.customSubscribers) {
      if (!this.removedTopics.has(topic)) {
        this.socket.send(JSON.stringify({ op: 'subscribe', topic: topic, throttle_rate: 0, queue_length: 1 }));
      }
    }
  }

  publishCmdVel(linearX = 0, angularZ = 0) {
    if (!this.isConnected || !this.socket || this.socket.readyState !== WebSocket.OPEN) return;

    const twistMsg = {
      op: 'publish',
      topic: this.topics.cmdVel,
      msg: {
        linear: { x: +linearX.toFixed(3), y: 0.0, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: +angularZ.toFixed(3) }
      }
    };
    this.socket.send(JSON.stringify(twistMsg));
    this.packetsSent++;
  }

  disconnect() {
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
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

    // Standard Nav2 interaction subscriptions
    const subTopics = [
      this.topics.goalPose,
      this.topics.moveBaseGoal,
      this.topics.clickedPoint,
      this.topics.initialPose,
      this.topics.cmdVel
    ];

    // In Live Ingest mode, auto-subscribe to all sensor feeds from the hardware robot
    if (this.isIngestMode) {
      subTopics.push(
        this.topics.robotOdom,
        this.topics.robotPose,
        this.topics.laserScan,
        this.topics.trackedHumans,
        this.topics.map,
        this.topics.globalPlan,
        this.topics.localPlan,
        this.topics.battery,
        this.topics.imu,
        this.topics.cameraCompressed,
        this.topics.diagnostics
      );
    }

    subTopics.forEach(top => {
      if (this.socket.readyState === WebSocket.OPEN && !this.removedTopics.has(top)) {
        this.socket.send(JSON.stringify({ op: 'subscribe', topic: top, throttle_rate: 0, queue_length: 1 }));
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

  /**
   * Generic ROS2 Service Caller over rosbridge_suite
   */
  callService(serviceName, args = {}, callback = null) {
    if (!this.isConnected || !this.socket || this.socket.readyState !== WebSocket.OPEN) {
      if (callback) callback(null, false);
      return;
    }

    const serviceId = `srv_${Date.now()}_${Math.random().toString(36).substr(2, 6)}`;
    if (callback) {
      this.pendingServices.set(serviceId, callback);
      setTimeout(() => {
        if (this.pendingServices.has(serviceId)) {
          this.pendingServices.delete(serviceId);
          callback(null, false);
        }
      }, 6000);
    }

    this.socket.send(JSON.stringify({
      op: 'call_service',
      id: serviceId,
      service: serviceName,
      args: args
    }));
  }

  /**
   * Query all available ROS2 topics and message types from the robot via /rosapi
   */
  discoverTopicsAndTypes(callback = null) {
    if (!this.isConnected) {
      if (callback) callback({}, {});
      return;
    }

    // 1. Try modern /rosapi/topics_and_raw_types
    this.callService('/rosapi/topics_and_raw_types', {}, (values, ok) => {
      if (ok && values && Array.isArray(values.topics) && Array.isArray(values.types)) {
        const topicsMap = {};
        for (let i = 0; i < values.topics.length; i++) {
          topicsMap[values.topics[i]] = values.types[i];
        }
        this.discoveredTopics = topicsMap;
        this.resolvedMapping = this.autoResolveTopics(topicsMap);
        if (this.onTopicsDiscovered) {
          this.onTopicsDiscovered(this.resolvedMapping, topicsMap);
        }
        if (callback) callback(this.resolvedMapping, topicsMap);
        return;
      }

      // 2. Fallback to /rosapi/topics
      this.callService('/rosapi/topics', {}, (valTopics, okTopics) => {
        if (okTopics && valTopics && Array.isArray(valTopics.topics)) {
          const topicsMap = {};
          const topicsList = valTopics.topics;
          let remaining = topicsList.length;

          if (remaining === 0) {
            this.discoveredTopics = {};
            this.resolvedMapping = this.autoResolveTopics({});
            if (callback) callback(this.resolvedMapping, {});
            return;
          }

          topicsList.forEach(t => {
            this.callService('/rosapi/topic_type', { topic: t }, (valType, okType) => {
              if (okType && valType && valType.type) {
                topicsMap[t] = valType.type;
              } else {
                topicsMap[t] = 'unknown';
              }
              remaining--;
              if (remaining <= 0) {
                this.discoveredTopics = topicsMap;
                this.resolvedMapping = this.autoResolveTopics(topicsMap);
                if (this.onTopicsDiscovered) {
                  this.onTopicsDiscovered(this.resolvedMapping, topicsMap);
                }
                if (callback) callback(this.resolvedMapping, topicsMap);
              }
            });
          });
        } else {
          // If rosapi service is unavailable, evaluate currently received packet stats
          const statsMap = {};
          for (const [topic, stat] of this.topicStats) {
            statsMap[topic] = stat.type || 'auto-sensed';
          }
          this.discoveredTopics = statsMap;
          this.resolvedMapping = this.autoResolveTopics(statsMap);
          if (this.onTopicsDiscovered) {
            this.onTopicsDiscovered(this.resolvedMapping, statsMap);
          }
          if (callback) callback(this.resolvedMapping, statsMap);
        }
      });
    });
  }

  /**
   * Smart Topic Resolver: Matches detected topics to required visualization roles
   */
  autoResolveTopics(topicsMap) {
    const roles = {
      robotOdom: {
        label: 'Odometry / Visual SLAM Pose',
        types: ['nav_msgs/msg/Odometry', 'geometry_msgs/msg/PoseWithCovarianceStamped', 'geometry_msgs/msg/PoseStamped'],
        defaults: [
          '/zed/zed_node/odom',
          '/zed_node/odom',
          '/zedx/zed_node/odom',
          '/zed2i/zed_node/odom',
          '/odometry/filtered',
          '/odom',
          '/robot_pose',
          '/jackal_velocity_controller/odom',
          '/diff_drive_controller/odom',
          '/turtlebot4/odom'
        ],
        keywords: ['odom', 'pose', 'filtered', 'zed', 'visual_odom'],
        current: this.topics.robotOdom
      },
      laserScan: {
        label: '2D LiDAR / Livox Scan',
        types: ['sensor_msgs/msg/LaserScan'],
        defaults: [
          '/scan',
          '/livox/scan',
          '/livox_scan',
          '/front/scan',
          '/lidar/scan',
          '/rplidar/scan',
          '/scan_raw',
          '/laser_scan'
        ],
        keywords: ['scan', 'laser', 'lidar', 'livox'],
        current: this.topics.laserScan
      },
      pointCloud: {
        label: '3D PointCloud (Livox / ZED X / LiDAR)',
        types: ['sensor_msgs/msg/PointCloud2'],
        defaults: [
          '/livox/lidar',
          '/livox/lidar/pointcloud',
          '/livox/points',
          '/livox_points',
          '/livox_lidar',
          '/zed/zed_node/point_cloud/cloud_registered',
          '/zed_node/point_cloud/cloud_registered',
          '/zedx/zed_node/point_cloud/cloud_registered',
          '/zed2i/zed_node/point_cloud/cloud_registered',
          '/zed/point_cloud/cloud_registered',
          '/velodyne_points',
          '/ouster/points',
          '/points',
          '/pointcloud',
          '/cloud',
          '/camera/depth/color/points',
          '/lidar_points'
        ],
        keywords: ['livox', 'zed', 'cloud_registered', 'cloud', 'points', 'pointcloud', 'point_cloud', 'velodyne', 'ouster', 'lidar'],
        current: this.topics.pointCloud || '/livox/lidar'
      },
      cameraCompressed: {
        label: 'FPV Camera Feed (ZED X / USB / ROS)',
        types: ['sensor_msgs/msg/CompressedImage', 'sensor_msgs/msg/Image'],
        defaults: [
          '/zed/zed_node/rgb/image_rect_color/compressed',
          '/zed/zed_node/left/image_rect_color/compressed',
          '/zedx/zed_node/rgb/image_rect_color/compressed',
          '/zed_node/rgb/image_rect_color/compressed',
          '/zed2i/zed_node/rgb/image_rect_color/compressed',
          '/zed/rgb/image_rect_color/compressed',
          '/zed/zed_node/rgb/image_raw/compressed',
          '/zed/zed_node/rgb/image_rect_color',
          '/camera/image_raw/compressed',
          '/camera/color/image_raw/compressed',
          '/image_raw/compressed',
          '/usb_cam/image_raw/compressed',
          '/front_camera/image/compressed',
          '/camera/image_raw'
        ],
        keywords: ['zed', 'rgb', 'color', 'image_rect', 'left', 'compressed', 'image', 'camera', 'front'],
        current: this.topics.cameraCompressed
      },
      map: {
        label: 'SLAM Occupancy Grid Map',
        types: ['nav_msgs/msg/OccupancyGrid'],
        defaults: ['/map', '/global_costmap/costmap', '/local_costmap/costmap', '/costmap'],
        keywords: ['map', 'costmap'],
        current: this.topics.map
      },
      globalPlan: {
        label: 'Nav2 Global Plan Path',
        types: ['nav_msgs/msg/Path'],
        defaults: ['/plan', '/global_plan', '/local_plan', '/transformed_global_plan'],
        keywords: ['plan', 'path', 'nav_path'],
        current: this.topics.globalPlan
      },
      trackedHumans: {
        label: 'Tracked Social Agents / ZED AI',
        types: ['geometry_msgs/msg/PoseArray', 'visualization_msgs/msg/MarkerArray', 'zed_msgs/msg/ObjectsStamped', 'pedestrian_msgs/msg/TrackedPersons', 'people_msgs/msg/People'],
        defaults: [
          '/zed/zed_node/obj_det/objects',
          '/zed_node/obj_det/objects',
          '/zedx/zed_node/obj_det/objects',
          '/zed/zed_node/body_trk/skeletons',
          '/tracked_humans',
          '/pedestrians',
          '/people',
          '/social_agents',
          '/human_poses'
        ],
        keywords: ['human', 'people', 'pedestrian', 'person', 'agent', 'obj_det', 'objects', 'body_trk', 'skeletons', 'zed'],
        current: this.topics.trackedHumans
      },
      cmdVel: {
        label: 'Teleop Velocity Command',
        types: ['geometry_msgs/msg/Twist', 'geometry_msgs/msg/TwistStamped'],
        defaults: ['/cmd_vel', '/cmd_vel_unstamped', '/jackal_velocity_controller/cmd_vel_unstamped', '/diff_drive_controller/cmd_vel_unstamped'],
        keywords: ['cmd_vel', 'twist', 'velocity'],
        current: this.topics.cmdVel
      },
      imu: {
        label: 'IMU Sensor (Livox Mid-360 / ZED X)',
        types: ['sensor_msgs/msg/Imu'],
        defaults: [
          '/livox/imu',
          '/livox/imu_data',
          '/livox/lidar/imu',
          '/zed/zed_node/imu/data',
          '/zed_node/imu/data',
          '/zedx/zed_node/imu/data',
          '/imu/data',
          '/imu'
        ],
        keywords: ['imu', 'livox', 'zed', 'accel', 'gyro'],
        current: this.topics.imu
      },
      battery: {
        label: 'Battery & Power State',
        types: ['sensor_msgs/msg/BatteryState'],
        defaults: ['/battery_state', '/battery'],
        keywords: ['battery', 'power'],
        current: this.topics.battery
      }
    };

    const mapping = {};
    const allTopicNames = Object.keys(topicsMap);

    for (const [roleKey, role] of Object.entries(roles)) {
      // 1. Find all candidate topics matching the expected types
      let matchingTopics = allTopicNames.filter(t => {
        const type = topicsMap[t];
        return role.types.some(expected => type === expected || (expected.includes('/') && type.endsWith(expected.split('/').pop())));
      });

      // If no exact type matched, try keyword matching on topic name
      if (matchingTopics.length === 0) {
        matchingTopics = allTopicNames.filter(t => {
          const lower = t.toLowerCase();
          return role.keywords.some(k => lower.includes(k));
        });
      }

      // Rank matching candidates by preference
      matchingTopics.sort((a, b) => {
        const aDefaultIdx = role.defaults.indexOf(a);
        const bDefaultIdx = role.defaults.indexOf(b);
        if (aDefaultIdx !== -1 && bDefaultIdx !== -1) return aDefaultIdx - bDefaultIdx;
        if (aDefaultIdx !== -1) return -1;
        if (bDefaultIdx !== -1) return 1;

        // Custom keyword weight scoring
        const scoreTopic = (tName) => {
          let s = 0;
          const lower = tName.toLowerCase();
          role.keywords.forEach((k, idx) => {
            if (lower.includes(k)) s += (100 - idx * 5);
          });
          return s;
        };

        return scoreTopic(b) - scoreTopic(a);
      });

      // Best match
      let selected = role.current;
      let autoMatched = false;

      if (matchingTopics.length > 0) {
        selected = matchingTopics[0];
        autoMatched = true;
      }

      this.topics[roleKey] = selected;
      mapping[roleKey] = {
        roleKey: roleKey,
        label: role.label,
        topic: selected,
        type: topicsMap[selected] || role.types[0],
        candidates: matchingTopics,
        autoMatched: autoMatched
      };
    }

    return mapping;
  }

  /**
   * Unsubscribe from a custom topic and permanently purge it from the inspector and active roles
   */
  unsubscribeCustom(topic) {
    this.removedTopics.add(topic);
    this.customSubscribers.delete(topic);
    this.topicStats.delete(topic);
    delete this.discoveredTopics[topic];

    // Clear any role that was assigned to this topic
    for (const [roleKey, roleTopic] of Object.entries(this.topics)) {
      if (roleTopic === topic) {
        this.topics[roleKey] = '';
        if (this.resolvedMapping[roleKey]) {
          this.resolvedMapping[roleKey].topic = '';
          this.resolvedMapping[roleKey].autoMatched = false;
        }
      }
    }

    if (this.isConnected && this.socket && this.socket.readyState === WebSocket.OPEN) {
      try {
        this.socket.send(JSON.stringify({ op: 'unsubscribe', topic: topic }));
        this.socket.send(JSON.stringify({ op: 'unsubscribe', id: topic }));
      } catch (err) {
        console.warn('Error sending unsubscribe:', err);
      }
    }
  }

  /**
   * Dynamically switch an active topic for a role and update subscribers
   */
  remapTopic(roleKey, newTopic, callback = null) {
    const oldTopic = this.topics[roleKey];
    if (oldTopic && oldTopic !== newTopic) {
      this.unsubscribeCustom(oldTopic);
    }
    this.removedTopics.delete(newTopic);
    this.topics[roleKey] = newTopic;
    if (callback) {
      this.subscribeCustom(newTopic, 'auto', callback);
    }
    if (this.resolvedMapping[roleKey]) {
      this.resolvedMapping[roleKey].topic = newTopic;
      this.resolvedMapping[roleKey].autoMatched = false;
    }
  }
}

export const ros2BridgeInstance = new ROS2Bridge();


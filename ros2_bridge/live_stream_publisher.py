#!/usr/bin/env python3
"""
SocialNav Studio — Live Robot Stream Demo Publisher Node (Multi-Obstacle & Camera FPV)
Publishes realistic 20 Hz ROS2 telemetry topics:
  - /odom (nav_msgs/Odometry)
  - /scan (sensor_msgs/LaserScan with static obstacles & dynamic pedestrians)
  - /map (nav_msgs/OccupancyGrid with static pillars, tables, dividers, and arena walls)
  - /tracked_humans (geometry_msgs/PoseArray)
  - /battery_state (sensor_msgs/BatteryState)
  - /imu/data (sensor_msgs/Imu)
  - /camera/image_raw/compressed (sensor_msgs/CompressedImage with 3D FPV rendering & AI HUD)
Subscribes to:
  - /cmd_vel (geometry_msgs/Twist) for web joystick teleoperation
  - /goal_pose (geometry_msgs/PoseStamped) for 2D nav goal dispatching
"""

import sys
import time
import math
import io
import numpy as np
import cv2

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Header
    from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
    from sensor_msgs.msg import LaserScan, BatteryState, Imu, CompressedImage, PointCloud2, PointField
    from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Twist, Point, Quaternion
except ImportError:
    print("⚠️ ROS2 python packages not found in current environment.")
    print("Please source ROS2 (e.g. source /opt/ros/humble/setup.bash) or use Web In-Browser Demo.")
    sys.exit(1)


# Static Obstacle Definitions in World Frame (meters)
STATIC_OBSTACLES = {
    'boxes': [
        {'x': 2.2, 'y': 2.2, 'w': 0.8, 'h': 0.8, 'name': 'Pillar NE'},
        {'x': -2.2, 'y': 2.2, 'w': 0.8, 'h': 0.8, 'name': 'Pillar NW'},
        {'x': -2.2, 'y': -2.2, 'w': 0.8, 'h': 0.8, 'name': 'Pillar SW'},
        {'x': 2.2, 'y': -2.2, 'w': 0.8, 'h': 0.8, 'name': 'Pillar SE'},
        {'x': 0.0, 'y': 2.6, 'w': 1.6, 'h': 0.6, 'name': 'Bench North'},
        {'x': 0.0, 'y': -2.6, 'w': 1.6, 'h': 0.6, 'name': 'Bench South'}
    ],
    'circles': [
        {'x': -3.6, 'y': 0.0, 'r': 0.45, 'name': 'Planter West'},
        {'x': 3.6, 'y': 0.0, 'r': 0.45, 'name': 'Planter East'},
        {'x': 0.0, 'y': 0.0, 'r': 0.55, 'name': 'Center Kiosk'}
    ],
    'walls': [
        # Bounding Arena Walls ([-5.5, 5.5])
        ((-5.5, -5.5), (5.5, -5.5)),
        ((5.5, -5.5), (5.5, 5.5)),
        ((5.5, 5.5), (-5.5, 5.5)),
        ((-5.5, 5.5), (-5.5, -5.5))
    ]
}


def create_pointcloud2_msg(header, points):
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * len(points)
    msg.is_dense = True
    if len(points) > 0:
        data_arr = np.array(points, dtype=np.float32)
        msg.data = data_arr.tobytes()
    else:
        msg.data = b''
    return msg


def ray_circle_intersect(ox, oy, cos_a, sin_a, cx, cy, radius):
    fx = ox - cx
    fy = oy - cy
    b = 2.0 * (fx * cos_a + fy * sin_a)
    c = (fx * fx + fy * fy) - radius * radius
    discriminant = b * b - 4.0 * c
    if discriminant < 0:
        return None
    sqrt_d = math.sqrt(discriminant)
    t1 = (-b - sqrt_d) / 2.0
    t2 = (-b + sqrt_d) / 2.0
    if t1 > 0.05: return t1
    if t2 > 0.05: return t2
    return None


def ray_rect_intersect(ox, oy, cos_a, sin_a, rx, ry, rw, rh):
    bx1 = rx - rw / 2.0
    bx2 = rx + rw / 2.0
    by1 = ry - rh / 2.0
    by2 = ry + rh / 2.0

    tmin = 0.0
    tmax = 10000.0

    if abs(cos_a) > 1e-5:
        t1 = (bx1 - ox) / cos_a
        t2 = (bx2 - ox) / cos_a
        if t1 > t2: t1, t2 = t2, t1
        tmin = max(tmin, t1)
        tmax = min(tmax, t2)
        if tmin > tmax: return None
    elif ox < bx1 or ox > bx2:
        return None

    if abs(sin_a) > 1e-5:
        t1 = (by1 - oy) / sin_a
        t2 = (by2 - oy) / sin_a
        if t1 > t2: t1, t2 = t2, t1
        tmin = max(tmin, t1)
        tmax = min(tmax, t2)
        if tmin > tmax: return None
    elif oy < by1 or oy > by2:
        return None

    return tmin if tmin > 0.05 else None


def ray_segment_intersect(ox, oy, cos_a, sin_a, x1, y1, x2, y2):
    sx = x2 - x1
    sy = y2 - y1
    denom = cos_a * sy - sin_a * sx
    if abs(denom) < 1e-8:
        return None
    t = ((x1 - ox) * sy - (y1 - oy) * sx) / denom
    u = ((x1 - ox) * sin_a - (y1 - oy) * cos_a) / denom
    if t > 0.05 and 0.0 <= u <= 1.0:
        return t
    return None


class LiveRobotStreamDemo(Node):
    def __init__(self):
        super().__init__('socialnav_live_stream_demo')

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.points_pub = self.create_publisher(PointCloud2, '/points', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.humans_pub = self.create_publisher(PoseArray, '/tracked_humans', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.camera_pub = self.create_publisher(CompressedImage, '/camera/image_raw/compressed', 10)

        # Subscribers (from Web Teleop & 2D Nav Goal)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)

        # Robot Physical State
        self.x = -1.0
        self.y = -1.0
        self.yaw = 0.5
        self.linear_v = 0.0
        self.angular_w = 0.0

        # Command velocity from Web Teleop
        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0
        self.last_cmd_time = time.time()

        # Goal target
        self.goal_x = None
        self.goal_y = None

        # Human pedestrian agents
        self.humans = [
            {'id': 1, 'x': 2.8, 'y': 1.0, 'vx': -0.35, 'vy': 0.15, 'yaw': 3.14},
            {'id': 2, 'x': -1.8, 'y': 1.6, 'vx': 0.30, 'vy': -0.20, 'yaw': -0.5},
            {'id': 3, 'x': 1.2, 'y': -1.8, 'vx': -0.10, 'vy': 0.40, 'yaw': 1.8},
            {'id': 4, 'x': -1.2, 'y': -3.2, 'vx': 0.35, 'vy': 0.10, 'yaw': 0.2}
        ]

        # Battery state
        self.battery_pct = 0.92
        self.battery_voltage = 24.6

        # OccupancyGrid Cache
        self.occupancy_grid_msg = self.generate_occupancy_grid()

        # Timers: 20 Hz Telemetry, 10 Hz Camera, 1 Hz Map
        self.timer_telemetry = self.create_timer(0.05, self.publish_telemetry)
        self.timer_camera = self.create_timer(0.08, self.publish_camera_frame) # ~12.5 FPS
        self.timer_map = self.create_timer(1.0, self.publish_map)

        self.get_logger().info("🚀 Live Robot Stream Demo Node ACTIVE on port 9091 (/odom, /scan, /map, /camera, /tracked_humans, /battery_state, /imu/data)")

    def cmd_vel_callback(self, msg):
        self.cmd_linear_x = msg.linear.x
        self.cmd_angular_z = msg.angular.z
        self.last_cmd_time = time.time()
        self.get_logger().info(f"🕹️ Web Teleop received: v={msg.linear.x:.2f} m/s, w={msg.angular.z:.2f} rad/s")

    def goal_pose_callback(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.get_logger().info(f"🎯 2D Nav Goal set on web: ({self.goal_x:.2f}, {self.goal_y:.2f})")

    def generate_occupancy_grid(self):
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        width = 240
        height = 240
        res = 0.05
        origin_x = -6.0
        origin_y = -6.0

        msg.info.resolution = res
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = origin_x
        msg.info.origin.position.y = origin_y

        grid = np.zeros((height, width), dtype=np.int8)

        def world_to_grid(wx, wy):
            gx = int((wx - origin_x) / res)
            gy = int((wy - origin_y) / res)
            return gx, gy

        # 1. Draw Perimeter & Divider Walls
        for p1, p2 in STATIC_OBSTACLES['walls']:
            gx1, gy1 = world_to_grid(p1[0], p1[1])
            gx2, gy2 = world_to_grid(p2[0], p2[1])
            cv2.line(grid, (gx1, gy1), (gx2, gy2), 100, thickness=2)

        # 2. Draw Rectangular Pillars & Benches
        for b in STATIC_OBSTACLES['boxes']:
            gx1, gy1 = world_to_grid(b['x'] - b['w'] / 2, b['y'] - b['h'] / 2)
            gx2, gy2 = world_to_grid(b['x'] + b['w'] / 2, b['y'] + b['h'] / 2)
            cv2.rectangle(grid, (gx1, gy1), (gx2, gy2), 100, -1)

        # 3. Draw Circular Planters & Kiosks
        for c in STATIC_OBSTACLES['circles']:
            cx, cy = world_to_grid(c['x'], c['y'])
            r_px = int(c['r'] / res)
            cv2.circle(grid, (cx, cy), r_px, 100, -1)

        msg.data = grid.flatten().tolist()
        return msg

    def publish_map(self):
        now = self.get_clock().now()
        self.occupancy_grid_msg.header.stamp = now.to_msg()
        self.map_pub.publish(self.occupancy_grid_msg)

    def publish_telemetry(self):
        dt = 0.05
        now = self.get_clock().now()

        # 1. Kinematics Controller
        if time.time() - self.last_cmd_time < 0.5:
            # Web Teleop Active
            self.linear_v = self.cmd_linear_x
            self.angular_w = self.cmd_angular_z
        elif self.goal_x is not None and self.goal_y is not None:
            # Autonomous Goal Approach
            dx = self.goal_x - self.x
            dy = self.goal_y - self.y
            dist = math.hypot(dx, dy)
            target_angle = math.atan2(dy, dx)
            angle_diff = (target_angle - self.yaw + math.pi) % (2 * math.pi) - math.pi

            if dist > 0.25:
                self.angular_w = max(-1.6, min(1.6, angle_diff * 2.5))
                self.linear_v = max(0.0, min(0.9, dist * 0.7)) if abs(angle_diff) < 0.7 else 0.05
            else:
                self.linear_v = 0.0
                self.angular_w = 0.0
                self.goal_x = None
                self.goal_y = None
        else:
            # Smooth autonomous patrol
            t = time.time()
            self.linear_v = 0.65
            self.angular_w = 0.35 * math.sin(t * 0.4) + 0.15 * math.cos(t * 0.8)

        # Update pose
        self.yaw += self.angular_w * dt
        next_x = self.x + self.linear_v * math.cos(self.yaw) * dt
        next_y = self.y + self.linear_v * math.sin(self.yaw) * dt

        # Collision avoidance boundary
        if abs(next_x) < 4.8 and abs(next_y) < 4.8:
            self.x = next_x
            self.y = next_y
        else:
            self.yaw += math.pi * 0.5

        # 2. Publish /odom
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)
        odom_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)
        odom_msg.twist.twist.linear.x = self.linear_v
        odom_msg.twist.twist.angular.z = self.angular_w
        self.odom_pub.publish(odom_msg)

        # 3. Update & Publish /tracked_humans
        humans_msg = PoseArray()
        humans_msg.header.stamp = now.to_msg()
        humans_msg.header.frame_id = 'map'
        for h in self.humans:
            h['x'] += h['vx'] * dt
            h['y'] += h['vy'] * dt
            if abs(h['x']) > 4.5: h['vx'] *= -1
            if abs(h['y']) > 4.5: h['vy'] *= -1
            h['yaw'] = math.atan2(h['vy'], h['vx'])

            p = Pose()
            p.position.x = h['x']
            p.position.y = h['y']
            p.orientation.z = math.sin(h['yaw'] / 2.0)
            p.orientation.w = math.cos(h['yaw'] / 2.0)
            humans_msg.poses.append(p)
        self.humans_pub.publish(humans_msg)

        # 4. Raycast 360-ray /scan matching Simulation Playground architecture
        scan_msg = LaserScan()
        scan_msg.header.stamp = now.to_msg()
        scan_msg.header.frame_id = 'base_link'
        scan_msg.angle_min = -math.pi
        scan_msg.angle_max = math.pi
        num_rays = 180
        scan_msg.angle_increment = (2 * math.pi) / num_rays
        scan_msg.range_min = 0.05
        scan_msg.range_max = 6.0

        ranges = []
        rx, ry, ryaw = self.x, self.y, self.yaw
        max_range_m = 6.0

        for i in range(num_rays):
            local_angle = scan_msg.angle_min + i * scan_msg.angle_increment
            ray_angle = ryaw + local_angle
            cos_a = math.cos(ray_angle)
            sin_a = math.sin(ray_angle)
            closest_dist = max_range_m

            # 1. Walls
            for (wx1, wy1), (wx2, wy2) in STATIC_OBSTACLES['walls']:
                d = ray_segment_intersect(rx, ry, cos_a, sin_a, wx1, wy1, wx2, wy2)
                if d is not None and d < closest_dist:
                    closest_dist = d

            # 2. Boxes
            for b in STATIC_OBSTACLES['boxes']:
                d = ray_rect_intersect(rx, ry, cos_a, sin_a, b['x'], b['y'], b['w'], b['h'])
                if d is not None and d < closest_dist:
                    closest_dist = d

            # 3. Circles
            for c in STATIC_OBSTACLES['circles']:
                d = ray_circle_intersect(rx, ry, cos_a, sin_a, c['x'], c['y'], c['r'])
                if d is not None and d < closest_dist:
                    closest_dist = d

            # 4. Humans
            for h in self.humans:
                d = ray_circle_intersect(rx, ry, cos_a, sin_a, h['x'], h['y'], 0.35)
                if d is not None and d < closest_dist:
                    closest_dist = d

            if closest_dist < (max_range_m - 0.05):
                noise = (np.random.rand() - 0.5) * 0.015
                ranges.append(float(closest_dist + noise))
            else:
                ranges.append(float('inf'))

        scan_msg.ranges = ranges
        self.scan_pub.publish(scan_msg)

        # 4.1 Raycast 3D LiDAR PointCloud2 (16-beam Velodyne style)
        cloud_points = []
        num_rings = 16
        num_azimuths = 72
        sensor_z = 0.35

        for ring in range(num_rings):
            vert_angle = math.radians(-15.0 + (ring / float(num_rings - 1)) * 30.0)
            cos_v = math.cos(vert_angle)
            sin_v = math.sin(vert_angle)

            for az in range(num_azimuths):
                az_angle = ryaw - math.pi + (az / float(num_azimuths)) * (2.0 * math.pi)
                cos_a = math.cos(az_angle)
                sin_a = math.sin(az_angle)

                closest_2d = max_range_m
                hit_height = 1.0
                hit_intensity = 45.0

                # 1. Walls
                for (wx1, wy1), (wx2, wy2) in STATIC_OBSTACLES['walls']:
                    d = ray_segment_intersect(rx, ry, cos_a, sin_a, wx1, wy1, wx2, wy2)
                    if d is not None and d < closest_2d:
                        closest_2d = d
                        hit_height = 2.4
                        hit_intensity = 75.0

                # 2. Boxes
                for b in STATIC_OBSTACLES['boxes']:
                    d = ray_rect_intersect(rx, ry, cos_a, sin_a, b['x'], b['y'], b['w'], b['h'])
                    if d is not None and d < closest_2d:
                        closest_2d = d
                        hit_height = 2.0 if 'Pillar' in b['name'] else 0.45
                        hit_intensity = 85.0 if 'Pillar' in b['name'] else 60.0

                # 3. Circles
                for c in STATIC_OBSTACLES['circles']:
                    d = ray_circle_intersect(rx, ry, cos_a, sin_a, c['x'], c['y'], c['r'])
                    if d is not None and d < closest_2d:
                        closest_2d = d
                        hit_height = 0.8
                        hit_intensity = 65.0

                # 4. Humans
                for h in self.humans:
                    d = ray_circle_intersect(rx, ry, cos_a, sin_a, h['x'], h['y'], 0.35)
                    if d is not None and d < closest_2d:
                        closest_2d = d
                        hit_height = 1.75
                        hit_intensity = 95.0

                # 3D Ground Hit (Z = 0)
                if sin_v < -0.01:
                    ground_dist = sensor_z / -sin_v
                    ground_2d = ground_dist * cos_v
                    if ground_2d < closest_2d and ground_2d < max_range_m:
                        px = rx + ground_2d * cos_a
                        py = ry + ground_2d * sin_a
                        cloud_points.append((float(px), float(py), 0.0, 25.0))
                        continue

                if closest_2d < (max_range_m - 0.1):
                    hit_z = sensor_z + closest_2d * (sin_v / (cos_v if abs(cos_v) > 1e-4 else 1e-4))
                    if 0.0 <= hit_z <= hit_height:
                        px = rx + closest_2d * cos_a
                        py = ry + closest_2d * sin_a
                        cloud_points.append((float(px), float(py), float(hit_z), float(hit_intensity)))

        header_points = Header()
        header_points.stamp = now.to_msg()
        header_points.frame_id = 'map'
        points_msg = create_pointcloud2_msg(header_points, cloud_points)
        self.points_pub.publish(points_msg)

        # 5. Publish /battery_state
        self.battery_pct = max(0.1, self.battery_pct - 0.000008)
        batt_msg = BatteryState()
        batt_msg.header.stamp = now.to_msg()
        batt_msg.percentage = float(self.battery_pct)
        batt_msg.voltage = float(24.0 + self.battery_pct * 1.2)
        batt_msg.temperature = 32.8
        self.battery_pub.publish(batt_msg)

        # 6. Publish /imu/data
        imu_msg = Imu()
        imu_msg.header.stamp = now.to_msg()
        imu_msg.header.frame_id = 'base_link'
        imu_msg.linear_acceleration.x = float(self.linear_v * 0.15 * math.cos(time.time() * 2.0))
        imu_msg.linear_acceleration.y = float(self.linear_v * 0.15 * math.sin(time.time() * 2.0))
        imu_msg.linear_acceleration.z = 9.81
        imu_msg.angular_velocity.z = float(self.angular_w)
        self.imu_pub.publish(imu_msg)

    def publish_camera_frame(self):
        """Generates realistic synthetic 3D perspective FPV camera image with HUD reticle."""
        width = 384
        height = 216
        img = np.zeros((height, width, 3), dtype=np.uint8)

        # 1. Dark Sky & Ground Gradient with Horizon Roll
        horizon_y = int(height * 0.48)
        # Sky: Dark Blue/Slate gradient
        img[:horizon_y, :] = (20, 15, 10)
        # Ground: Dark Emerald/Teal floor
        img[horizon_y:, :] = (25, 30, 20)

        # Perspective Floor Grid Lines
        for gx in range(-10, 11, 2):
            # Ground grid vanishing to center horizon
            cx_vanish = width // 2
            x_bottom = int(cx_vanish + gx * 45 - self.yaw * 30)
            cv2.line(img, (cx_vanish, horizon_y), (x_bottom, height), (35, 60, 45), 1)

        for depth_step in [1.2, 2.0, 3.2, 5.0, 8.0]:
            y_plane = int(horizon_y + (height - horizon_y) / depth_step)
            cv2.line(img, (0, y_plane), (width, y_plane), (30, 50, 40), 1)

        # 2. Render 3D Projected Static Pillars in View Frustum
        rx, ry, ryaw = self.x, self.y, self.yaw
        fov_rad = 1.2 # ~70 deg FOV
        f_len = width / (2.0 * math.tan(fov_rad / 2.0))

        # Project Obstacles
        for b in STATIC_OBSTACLES['boxes']:
            dx = b['x'] - rx
            dy = b['y'] - ry
            # Transform to robot camera frame (Forward = X_rel, Left = Y_rel)
            x_rel = dx * math.cos(ryaw) + dy * math.sin(ryaw)
            y_rel = -dx * math.sin(ryaw) + dy * math.cos(ryaw)

            if x_rel > 0.4: # in front of camera
                u = int(width / 2 - (y_rel / x_rel) * f_len)
                w_px = int((b['w'] / x_rel) * f_len)
                h_px = int((1.8 / x_rel) * f_len)
                v_bottom = int(horizon_y + (0.9 / x_rel) * f_len)
                v_top = v_bottom - h_px

                if -100 < u < width + 100:
                    # Draw shaded 3D pillar box
                    cv2.rectangle(img, (u - w_px // 2, max(0, v_top)), (u + w_px // 2, min(height, v_bottom)), (70, 75, 60), -1)
                    cv2.rectangle(img, (u - w_px // 2, max(0, v_top)), (u + w_px // 2, min(height, v_bottom)), (120, 200, 160), 1)
                    cv2.putText(img, b['name'], (u - w_px // 2, max(15, v_top - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (120, 200, 160), 1)

        # 3. Render 3D Projected Human Pedestrians with AI Bounding Boxes
        for h in self.humans:
            dx = h['x'] - rx
            dy = h['y'] - ry
            x_rel = dx * math.cos(ryaw) + dy * math.sin(ryaw)
            y_rel = -dx * math.sin(ryaw) + dy * math.cos(ryaw)
            dist = math.hypot(x_rel, y_rel)

            if x_rel > 0.4:
                u = int(width / 2 - (y_rel / x_rel) * f_len)
                w_px = int((0.55 / x_rel) * f_len)
                h_px = int((1.75 / x_rel) * f_len)
                v_bottom = int(horizon_y + (0.9 / x_rel) * f_len)
                v_top = v_bottom - h_px

                if 0 <= u <= width:
                    # Human silhouette
                    cv2.ellipse(img, (u, v_top + h_px // 5), (w_px // 3, h_px // 5), 0, 0, 360, (0, 230, 255), -1)
                    cv2.rectangle(img, (u - w_px // 3, v_top + h_px // 3), (u + w_px // 3, v_bottom), (0, 180, 200), -1)

                    # Green AI Detection Bounding Box
                    cv2.rectangle(img, (u - w_px // 2, max(0, v_top)), (u + w_px // 2, min(height, v_bottom)), (0, 255, 157), 1)
                    label = f"HUMAN #{h['id']} | {dist:.1f}m"
                    cv2.putText(img, label, (max(5, u - w_px // 2), max(12, v_top - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.30, (0, 255, 157), 1)

        # 4. Cyber HUD Reticle Overlay
        cx, cy = width // 2, height // 2
        # Crosshairs
        cv2.line(img, (cx - 15, cy), (cx - 4, cy), (0, 229, 255), 1)
        cv2.line(img, (cx + 4, cy), (cx + 15, cy), (0, 229, 255), 1)
        cv2.line(img, (cx, cy - 15), (cx, cy - 4), (0, 229, 255), 1)
        cv2.line(img, (cx, cy + 4), (cx, cy + 15), (0, 229, 255), 1)
        cv2.circle(img, (cx, cy), 20, (0, 229, 255), 1)

        # Top Header Bar
        cv2.rectangle(img, (0, 0), (width, 18), (10, 15, 15), -1)
        cv2.putText(img, f"CAM: FPV-FRONT | ROS2 /camera/image_raw/compressed | 15 FPS", (8, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (0, 229, 255), 1)

        # Bottom Telemetry Bar
        cv2.rectangle(img, (0, height - 20), (width, height), (10, 15, 15), -1)
        telemetry_str = f"SPEED: {self.linear_v:.2f} m/s | YAW: {(self.yaw * 180 / math.pi) % 360:.0f} deg | BATT: {int(self.battery_pct * 100)}%"
        cv2.putText(img, telemetry_str, (8, height - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.30, (0, 255, 157), 1)

        # Encode frame as JPEG
        success, encoded_jpg = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if success:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_link'
            msg.format = 'jpeg'
            msg.data = encoded_jpg.tobytes()
            self.camera_pub.publish(msg)


def main():
    rclpy.init()
    node = LiveRobotStreamDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

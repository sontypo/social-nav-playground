#!/usr/bin/env python3
"""
================================================================================
Social Navigation Robot Autonomous Controller Node for ROS2 / Gazebo
================================================================================
Drives the robot inside Gazebo (Classic 11 & Ignition) using Social Forces (SFM)
and Hall's Proxemics cost functions:
  - Subscribes: /goal_pose, /odom, /scan, /tracked_humans
  - Computes: Goal attraction, Human Proxemics repulsion, Static obstacle avoidance
  - Publishes: /cmd_vel (geometry_msgs/msg/Twist)
================================================================================
"""

import sys
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class SocialRobotController(Node):
    def __init__(self):
        super().__init__('social_robot_controller')

        # Parameters
        self.max_speed = 1.2 # m/s
        self.max_w = 1.5     # rad/s
        self.goal_tolerance = 0.35 # m
        self.courtesy_weight = 1.2

        # State
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.goal_x = 4.0
        self.goal_y = 3.0
        self.has_goal = True
        self.humans = []
        self.scan_ranges = []

        # ROS2 Subscriptions
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_goal = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.sub_humans = self.create_subscription(PoseArray, '/tracked_humans', self.humans_callback, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # ROS2 Publisher
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # Control Loop Timer (20 Hz)
        self.timer = self.create_timer(0.05, self.control_step)
        self.get_logger().info("🤖 Social Robot Autonomous Controller running on /cmd_vel (20 Hz)...")

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        ori = msg.pose.pose.orientation
        # Compute yaw from quaternion
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def goal_callback(self, msg: PoseStamped):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.has_goal = True
        self.get_logger().info(f"🎯 New Goal Received: ({self.goal_x:.2f}, {self.goal_y:.2f})")

    def humans_callback(self, msg: PoseArray):
        self.humans = []
        for p in msg.poses:
            self.humans.append({'x': p.position.x, 'y': p.position.y})

    def scan_callback(self, msg: LaserScan):
        self.scan_ranges = list(msg.ranges)

    def control_step(self):
        cmd = Twist()

        if not self.has_goal:
            self.pub_cmd.publish(cmd)
            return

        # Distance to Goal
        gx = self.goal_x - self.robot_x
        gy = self.goal_y - self.robot_y
        dist_to_goal = math.hypot(gx, gy)

        if dist_to_goal < self.goal_tolerance:
            # Reached Goal
            self.pub_cmd.publish(cmd) # Stop
            return

        # 1. Goal Attractive Force
        desired_heading = math.atan2(gy, gx)
        f_goal_x = math.cos(desired_heading) * 1.5
        f_goal_y = math.sin(desired_heading) * 1.5

        # 2. Human Proxemics Repulsion Force
        f_human_x, f_human_y = 0.0, 0.0
        for h in self.humans:
            dx = self.robot_x - h['x']
            dy = self.robot_y - h['y']
            dist = math.hypot(dx, dy)
            if dist < 2.5 and dist > 0.01:
                # Anisotropic Hall's Proxemics
                rep_mag = self.courtesy_weight * 3.0 * math.exp((0.6 - dist) / 0.45)
                f_human_x += (dx / dist) * rep_mag
                f_human_y += (dy / dist) * rep_mag

        # 3. LiDAR Obstacle Repulsion Force
        f_obs_x, f_obs_y = 0.0, 0.0
        if self.scan_ranges:
            num_samples = len(self.scan_ranges)
            for i in range(0, num_samples, 10): # Sample every 10 rays
                r = self.scan_ranges[i]
                if 0.1 < r < 1.5:
                    angle = self.robot_yaw + (-math.pi + i * (2.0 * math.pi / num_samples))
                    obs_x = self.robot_x + r * math.cos(angle)
                    obs_y = self.robot_y + r * math.sin(angle)
                    dx = self.robot_x - obs_x
                    dy = self.robot_y - obs_y
                    dist = math.hypot(dx, dy)
                    if dist > 0.01:
                        rep_mag = 2.0 * (1.5 - r) / 1.5
                        f_obs_x += (dx / dist) * rep_mag
                        f_obs_y += (dy / dist) * rep_mag

        # Net Force Vector
        net_fx = f_goal_x + f_human_x + f_obs_x
        net_fy = f_goal_y + f_human_y + f_obs_y

        target_heading = math.atan2(net_fy, net_fx)
        heading_error = target_heading - self.robot_yaw

        # Normalize heading error to [-pi, pi]
        while heading_error > math.pi: heading_error -= 2.0 * math.pi
        while heading_error < -math.pi: heading_error += 2.0 * math.pi

        # Velocity Controller
        if abs(heading_error) > 0.8:
            # Rotate in place
            cmd.linear.x = 0.15
            cmd.angular.z = max(-self.max_w, min(self.max_w, heading_error * 1.8))
        else:
            # Move forward and turn smoothly
            speed_factor = max(0.2, math.cos(heading_error))
            cmd.linear.x = min(self.max_speed * speed_factor, dist_to_goal)
            cmd.angular.z = max(-self.max_w, min(self.max_w, heading_error * 2.2))

        self.pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SocialRobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

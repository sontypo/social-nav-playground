#!/usr/bin/env python3
"""
================================================================================
Gazebo Crowd & Human Pedestrian Controller Node for ROS2
================================================================================
Controls dynamic human pedestrians in Gazebo (Classic 11 & Ignition / Gz Sim).
Features:
  1. Social Force Model (SFM) autonomous pedestrian movement in Gazebo.
  2. Dataset Trajectory Replayer mode (ETH/UCY/TrajNet/SCAND datasets).
  3. Real-time synchronized synchronization with Gazebo physics (/gazebo/set_entity_state).
  4. Publishes `/tracked_humans` (geometry_msgs/msg/PoseArray) for Nav2 & RViz2.
  5. Publishes individual human velocity / poses for Proxemics evaluation.
================================================================================
"""

import sys
import os
import math
import time
import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion, Twist
from nav_msgs.msg import Odometry

try:
    from gazebo_msgs.srv import SetEntityState, SetModelState
    GAZEBO_MSGS_AVAILABLE = True
except ImportError:
    GAZEBO_MSGS_AVAILABLE = False


class HumanAgent:
    """Represents a simulated pedestrian agent with SFM dynamics."""
    def __init__(self, agent_id, x, y, target_x, target_y, speed=1.1, radius=0.28):
        self.id = agent_id
        self.x = float(x)
        self.y = float(y)
        self.vx = 0.0
        self.vy = 0.0
        self.heading = 0.0
        self.target_x = float(target_x)
        self.target_y = float(target_y)
        self.desired_speed = float(speed)
        self.radius = float(radius)
        self.relaxation_time = 0.5
        self.waypoints = [(target_x, target_y), (x, y)] # Patrol back and forth
        self.wp_index = 0

    def update_sfm(self, dt, other_humans, robot_pos, obstacles):
        """Compute Helbing Social Force Model for this pedestrian."""
        # 1. Goal Driving Force
        cur_target = self.waypoints[self.wp_index]
        dx = cur_target[0] - self.x
        dy = cur_target[1] - self.y
        dist_to_target = math.hypot(dx, dy)

        if dist_to_target < 0.6:
            # Switch waypoint
            self.wp_index = (self.wp_index + 1) % len(self.waypoints)
            cur_target = self.waypoints[self.wp_index]
            dx = cur_target[0] - self.x
            dy = cur_target[1] - self.y
            dist_to_target = math.hypot(dx, dy)

        if dist_to_target > 0.01:
            desired_vx = (dx / dist_to_target) * self.desired_speed
            desired_vy = (dy / dist_to_target) * self.desired_speed
        else:
            desired_vx, desired_vy = 0.0, 0.0

        f_goal_x = (desired_vx - self.vx) / self.relaxation_time
        f_goal_y = (desired_vy - self.vy) / self.relaxation_time

        # 2. Repulsion from Other Humans
        f_humans_x, f_humans_y = 0.0, 0.0
        for h in other_humans:
            if h.id == self.id:
                continue
            hdx = self.x - h.x
            hdy = self.y - h.y
            hdist = math.hypot(hdx, hdy)
            min_dist = self.radius + h.radius
            if hdist < 2.5 and hdist > 0.001:
                # Anisotropic Exponential Repulsion
                rep_mag = 2.0 * math.exp((min_dist - hdist) / 0.5)
                f_humans_x += (hdx / hdist) * rep_mag
                f_humans_y += (hdy / hdist) * rep_mag

        # 3. Repulsion from Robot (Hall's Proxemics Personal Breach avoidance)
        f_robot_x, f_robot_y = 0.0, 0.0
        if robot_pos:
            rdx = self.x - robot_pos['x']
            rdy = self.y - robot_pos['y']
            rdist = math.hypot(rdx, rdy)
            if rdist < 2.0 and rdist > 0.001:
                rep_mag = 3.5 * math.exp((0.55 - rdist) / 0.4)
                f_robot_x += (rdx / rdist) * rep_mag
                f_robot_y += (rdy / rdist) * rep_mag

        # 4. Integrate acceleration and velocity
        ax = f_goal_x + f_humans_x + f_robot_x
        ay = f_goal_y + f_humans_y + f_robot_y

        self.vx += ax * dt
        self.vy += ay * dt

        # Speed cap
        cur_speed = math.hypot(self.vx, self.vy)
        max_speed = self.desired_speed * 1.3
        if cur_speed > max_speed:
            self.vx = (self.vx / cur_speed) * max_speed
            self.vy = (self.vy / cur_speed) * max_speed

        self.x += self.vx * dt
        self.y += self.vy * dt

        if cur_speed > 0.05:
            self.heading = math.atan2(self.vy, self.vx)


class GazeboHumanControllerNode(Node):
    def __init__(self, num_humans=4, rate_hz=20.0, dataset_path=None):
        super().__init__('gazebo_human_controller')

        self.rate_hz = float(rate_hz)
        self.dt = 1.0 / self.rate_hz
        self.dataset_path = dataset_path
        self.robot_pos = {'x': 0.0, 'y': 0.0}

        # Publishers & Subscribers
        self.pub_tracked_humans = self.create_publisher(PoseArray, '/tracked_humans', 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Gazebo Set Entity State Client
        self.cli_set_entity = None
        if GAZEBO_MSGS_AVAILABLE:
            self.cli_set_entity = self.create_client(SetEntityState, '/gazebo/set_entity_state')

        # Initialize Humans
        self.humans = []
        self.dataset_frames = {}
        self.frame_keys = []
        self.frame_idx = 0

        if dataset_path and os.path.exists(dataset_path):
            self.load_dataset(dataset_path)
            self.get_logger().info(f"📂 Dataset Mode: Replaying {len(self.frame_keys)} frames from {dataset_path}")
        else:
            self.init_sfm_humans(num_humans)
            self.get_logger().info(f"🤖 SFM Physics Mode: Simulating {len(self.humans)} autonomous agents with Social Forces")

        self.timer = self.create_timer(self.dt, self.loop_step)
        self.get_logger().info(f"✅ Gazebo Human Controller running at {self.rate_hz} Hz...")

    def odom_callback(self, msg: Odometry):
        self.robot_pos['x'] = msg.pose.pose.position.x
        self.robot_pos['y'] = msg.pose.pose.position.y

    def init_sfm_humans(self, count):
        """Create sample crossing pedestrians in a square arena."""
        configs = [
            (-4.0, -2.5, 4.0, 2.5, 1.15),
            (3.5, -2.8, -3.5, 2.8, 1.05),
            (-3.0, 3.0, 3.0, -3.0, 1.20),
            (0.0, -3.5, 0.0, 3.5, 0.95),
            (-4.5, 0.0, 4.5, 0.0, 1.10),
            (2.0, 3.5, -2.0, -3.5, 1.00),
        ]
        for i in range(min(count, len(configs))):
            x, y, tx, ty, spd = configs[i]
            self.humans.append(HumanAgent(f"pedestrian_{i+1}", x, y, tx, ty, speed=spd))

    def load_dataset(self, path):
        """Parse frame_id ped_id x y from dataset file."""
        with open(path, 'r') as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) >= 4 and not parts[0].startswith('#'):
                    try:
                        fid = int(float(parts[0]))
                        pid = int(float(parts[1]))
                        x = float(parts[2])
                        y = float(parts[3])
                        if fid not in self.dataset_frames:
                            self.dataset_frames[fid] = []
                        self.dataset_frames[fid].append({'id': pid, 'x': x, 'y': y})
                    except ValueError:
                        continue
        self.frame_keys = sorted(self.dataset_frames.keys())

    def loop_step(self):
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'map'

        if self.dataset_path and self.frame_keys:
            # Dataset Replay Mode
            cur_frame = self.frame_keys[self.frame_idx]
            peds = self.dataset_frames[cur_frame]

            for p in peds:
                pose = Pose()
                pose.position.x = p['x']
                pose.position.y = p['y']
                pose.position.z = 0.85
                pose.orientation.w = 1.0
                pose_array.poses.append(pose)
                self.sync_gazebo_entity(f"pedestrian_{p['id']}", p['x'], p['y'], 0.0)

            self.frame_idx = (self.frame_idx + 1) % len(self.frame_keys)
        else:
            # SFM Autonomous Physics Mode
            for h in self.humans:
                h.update_sfm(self.dt, self.humans, self.robot_pos, [])

                pose = Pose()
                pose.position.x = h.x
                pose.position.y = h.y
                pose.position.z = 0.85

                qz = math.sin(h.heading / 2.0)
                qw = math.cos(h.heading / 2.0)
                pose.orientation.z = qz
                pose.orientation.w = qw
                pose_array.poses.append(pose)

                self.sync_gazebo_entity(h.id, h.x, h.y, h.heading)

        # Publish to ROS2 Topic
        self.pub_tracked_humans.publish(pose_array)

    def sync_gazebo_entity(self, entity_name, x, y, yaw):
        """Update 3D model pose directly in Gazebo simulation world."""
        if not self.cli_set_entity or not self.cli_set_entity.service_is_ready():
            return

        req = SetEntityState.Request()
        req.state.name = entity_name
        req.state.pose.position.x = float(x)
        req.state.pose.position.y = float(y)
        req.state.pose.position.z = 0.85
        req.state.pose.orientation.z = math.sin(yaw / 2.0)
        req.state.pose.orientation.w = math.cos(yaw / 2.0)
        req.state.reference_frame = 'world'

        self.cli_set_entity.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description="Gazebo Crowd & Human Controller Node for ROS2")
    parser.add_argument('--humans', type=int, default=4, help="Number of SFM pedestrians (default: 4)")
    parser.add_argument('--rate', type=float, default=20.0, help="Publish rate in Hz (default: 20)")
    parser.add_argument('--dataset', type=str, default='', help="Optional path to dataset file (.txt)")

    parsed_args, unknown = parser.parse_known_args()

    node = GazeboHumanControllerNode(
        num_humans=parsed_args.humans,
        rate_hz=parsed_args.rate,
        dataset_path=parsed_args.dataset if parsed_args.dataset else None
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

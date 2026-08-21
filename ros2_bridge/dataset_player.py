#!/usr/bin/env python3
"""
ROS2 Crowd Dataset Replayer Node (ETH / UCY / TrajNet)
Reads benchmark pedestrian trajectory files (.txt / .csv) and streams them
at 10Hz/20Hz to `/tracked_humans` (PoseArray) for testing algorithms in RViz2 and Web.
"""

import sys
import os
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose

class CrowdDatasetPlayer(Node):
    def __init__(self, dataset_path, fps=10.0):
        super().__init__('crowd_dataset_player')

        self.dataset_path = dataset_path
        self.fps = fps
        self.interval = 1.0 / fps

        self.pub_humans = self.create_publisher(PoseArray, '/tracked_humans', 10)
        self.frames = self.load_dataset(dataset_path)

        if not self.frames:
            self.get_logger().error(f"❌ Failed to parse valid frames from: {dataset_path}")
            sys.exit(1)

        self.frame_keys = sorted(self.frames.keys())
        self.current_idx = 0

        self.get_logger().info(f"📊 Loaded {len(self.frame_keys)} frames from {os.path.basename(dataset_path)}")
        self.get_logger().info(f"Streaming crowd trajectories to /tracked_humans at {self.fps} Hz...")

        self.timer = self.create_timer(self.interval, self.timer_callback)

    def load_dataset(self, path):
        """Parse standard ETH/UCY format: frame_id ped_id x y"""
        frames = {}
        with open(path, 'r') as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) >= 4 and not parts[0].startswith('#'):
                    try:
                        frame_id = int(float(parts[0]))
                        ped_id = int(float(parts[1]))
                        x = float(parts[2])
                        y = float(parts[3])

                        if frame_id not in frames:
                            frames[frame_id] = []
                        frames[frame_id].append((ped_id, x, y))
                    except ValueError:
                        continue
        return frames

    def timer_callback(self):
        if not self.frame_keys:
            return

        frame_id = self.frame_keys[self.current_idx]
        pedestrians = self.frames[frame_id]

        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        for ped_id, x, y in pedestrians:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.pub_humans.publish(msg)

        # Loop playback
        self.current_idx = (self.current_idx + 1) % len(self.frame_keys)

def main(args=None):
    rclpy.init(args=args)

    dataset_file = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), 'datasets', 'eth_hotel_sample.txt')
    fps = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0

    node = CrowdDatasetPlayer(dataset_file, fps)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

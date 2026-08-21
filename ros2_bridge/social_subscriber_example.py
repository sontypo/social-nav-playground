#!/usr/bin/env python3
"""
ROS2 Social Navigation Bridge Visualizer & TF Broadcaster
Subscribes to `/robot_pose`, `/tracked_humans`, `/scan`, `/goal_pose` from the web simulator,
broadcasts TF transforms (map -> base_link -> laser_link), and publishes 3D Markers for RViz2.
"""

import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import PoseStamped, PoseArray, TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

class SocialNavBridgeListener(Node):
    def __init__(self):
        super().__init__('social_nav_bridge_listener')

        # 1. TF Broadcasters (Crucial for RViz2 to display /scan and robot model)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.broadcast_static_tf()

        # 2. Subscriptions
        self.sub_robot = self.create_subscription(
            PoseStamped,
            '/robot_pose',
            self.robot_pose_callback,
            10
        )

        self.sub_humans = self.create_subscription(
            PoseArray,
            '/tracked_humans',
            self.humans_callback,
            10
        )

        self.sub_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.sub_goal = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        self.sub_costmap = self.create_subscription(
            OccupancyGrid,
            '/social_costmap',
            self.costmap_callback,
            10
        )

        # 3. Publisher for RViz2 3D Markers (Humans, Robot Body, Goal)
        self.pub_markers = self.create_publisher(MarkerArray, '/social_nav/rviz_markers', 10)

        # Keep track of robot pose
        self.current_robot_pose = None
        self.costmap_count = 0

        self.get_logger().info('===============================================================')
        self.get_logger().info('🤖 Social Navigation RViz2 Bridge & TF Broadcaster ACTIVE!')
        self.get_logger().info('Broadcasting TF: map -> base_link -> laser_link')
        self.get_logger().info('Listening on: /robot_pose, /tracked_humans, /scan, /goal_pose')
        self.get_logger().info('===============================================================')

    def broadcast_static_tf(self):
        """Broadcast static transform between base_link and laser_link."""
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = 'base_link'
        static_tf.child_frame_id = 'laser_link'
        static_tf.transform.translation.x = 0.0
        static_tf.transform.translation.y = 0.0
        static_tf.transform.translation.z = 0.15
        static_tf.transform.rotation.x = 0.0
        static_tf.transform.rotation.y = 0.0
        static_tf.transform.rotation.z = 0.0
        static_tf.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(static_tf)

    def robot_pose_callback(self, msg: PoseStamped):
        self.current_robot_pose = msg
        now = self.get_clock().now().to_msg()

        # 1. Broadcast Dynamic TF: map -> base_link
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = 'map'
        tf_msg.child_frame_id = 'base_link'
        tf_msg.transform.translation.x = msg.pose.position.x
        tf_msg.transform.translation.y = msg.pose.position.y
        tf_msg.transform.translation.z = msg.pose.position.z
        tf_msg.transform.rotation = msg.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)

        # 2. Also broadcast static transform with current timestamp
        static_tf = TransformStamped()
        static_tf.header.stamp = now
        static_tf.header.frame_id = 'base_link'
        static_tf.child_frame_id = 'laser_link'
        static_tf.transform.translation.x = 0.0
        static_tf.transform.translation.y = 0.0
        static_tf.transform.translation.z = 0.15
        static_tf.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(static_tf)

        # 3. Publish 3D Robot AMR Chassis Marker in RViz
        marker_array = MarkerArray()

        # Robot Base Box
        robot_marker = Marker()
        robot_marker.header.stamp = now
        robot_marker.header.frame_id = 'base_link'
        robot_marker.ns = 'robot_body'
        robot_marker.id = 1000
        robot_marker.type = Marker.CUBE
        robot_marker.action = Marker.ADD
        robot_marker.pose.position.z = 0.10
        robot_marker.pose.orientation.w = 1.0
        robot_marker.scale.x = 0.50 # 50cm long
        robot_marker.scale.y = 0.40 # 40cm wide
        robot_marker.scale.z = 0.20 # 20cm high
        robot_marker.color.r = 0.0
        robot_marker.color.g = 1.0 # Neon Green
        robot_marker.color.b = 0.6
        robot_marker.color.a = 0.9
        marker_array.markers.append(robot_marker)

        # Robot LiDAR Puck Top
        lidar_marker = Marker()
        lidar_marker.header.stamp = now
        lidar_marker.header.frame_id = 'laser_link'
        lidar_marker.ns = 'robot_lidar'
        lidar_marker.id = 1001
        lidar_marker.type = Marker.CYLINDER
        lidar_marker.action = Marker.ADD
        lidar_marker.pose.orientation.w = 1.0
        lidar_marker.scale.x = 0.12
        lidar_marker.scale.y = 0.12
        lidar_marker.scale.z = 0.08
        lidar_marker.color.r = 0.0
        lidar_marker.color.g = 0.9
        lidar_marker.color.b = 1.0
        lidar_marker.color.a = 0.95
        marker_array.markers.append(lidar_marker)

        self.pub_markers.publish(marker_array)

        self.get_logger().info(
            f"📍 [TF & Robot] x={msg.pose.position.x:+.2f}m, y={msg.pose.position.y:+.2f}m",
            throttle_duration_sec=2.0
        )

    def scan_callback(self, msg: LaserScan):
        if len(msg.ranges) > 0:
            min_dist = min(msg.ranges)
            self.get_logger().info(
                f"📡 [LaserScan] {len(msg.ranges)} rays (frame: {msg.header.frame_id}) | Min obstacle: {min_dist:.2f}m",
                throttle_duration_sec=2.0
            )

    def humans_callback(self, msg: PoseArray):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        for idx, pose in enumerate(msg.poses):
            # Human 3D Body (Cylinder)
            body = Marker()
            body.header.stamp = now
            body.header.frame_id = 'map'
            body.ns = 'humans'
            body.id = idx * 2
            body.type = Marker.CYLINDER
            body.action = Marker.ADD
            body.pose.position.x = pose.position.x
            body.pose.position.y = pose.position.y
            body.pose.position.z = 0.85 # Half of human height
            body.pose.orientation = pose.orientation
            body.scale.x = 0.40 # 40cm body diameter
            body.scale.y = 0.40
            body.scale.z = 1.70 # 1.7m human height
            body.color.r = 0.0
            body.color.g = 0.85
            body.color.b = 1.0 # Neon Cyan
            body.color.a = 0.85
            marker_array.markers.append(body)

            # Human Head (Sphere)
            head = Marker()
            head.header.stamp = now
            head.header.frame_id = 'map'
            head.ns = 'human_heads'
            head.id = idx * 2 + 1
            head.type = Marker.SPHERE
            head.action = Marker.ADD
            head.pose.position.x = pose.position.x
            head.pose.position.y = pose.position.y
            head.pose.position.z = 1.70 + 0.12
            head.pose.orientation = pose.orientation
            head.scale.x = 0.25
            head.scale.y = 0.25
            head.scale.z = 0.25
            head.color.r = 1.0
            head.color.g = 1.0
            head.color.b = 1.0
            head.color.a = 0.95
            marker_array.markers.append(head)

        self.pub_markers.publish(marker_array)

    def goal_callback(self, msg: PoseStamped):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Goal Flag / Pole Marker in RViz
        goal_marker = Marker()
        goal_marker.header.stamp = now
        goal_marker.header.frame_id = 'map'
        goal_marker.ns = 'navigation_goal'
        goal_marker.id = 9999
        goal_marker.type = Marker.CYLINDER
        goal_marker.action = Marker.ADD
        goal_marker.pose.position.x = msg.pose.position.x
        goal_marker.pose.position.y = msg.pose.position.y
        goal_marker.pose.position.z = 0.75
        goal_marker.pose.orientation.w = 1.0
        goal_marker.scale.x = 0.15
        goal_marker.scale.y = 0.15
        goal_marker.scale.z = 1.5
        goal_marker.color.r = 1.0
        goal_marker.color.g = 0.8
        goal_marker.color.b = 0.0 # Yellow/Gold
        # Goal Top Sphere
        goal_top = Marker()
        goal_top.header.stamp = now
        goal_top.header.frame_id = 'map'
        goal_top.ns = 'navigation_goal'
        goal_top.id = 10000
        goal_top.type = Marker.SPHERE
        goal_top.action = Marker.ADD
        goal_top.pose.position.x = msg.pose.position.x
        goal_top.pose.position.y = msg.pose.position.y
        goal_top.pose.position.z = 1.55
        goal_top.pose.orientation.w = 1.0
        goal_top.scale.x = 0.35
        goal_top.scale.y = 0.35
        goal_top.scale.z = 0.35
        goal_top.color.r = 1.0
        goal_top.color.g = 0.8
        goal_top.color.b = 0.0
        goal_top.color.a = 0.95
        marker_array.markers.append(goal_top)

        self.pub_markers.publish(marker_array)
        self.get_logger().info(f"🎯 [Goal Received] x={msg.pose.position.x:+.2f}m, y={msg.pose.position.y:+.2f}m")

    def costmap_callback(self, msg: OccupancyGrid):
        self.costmap_count += 1
        if self.costmap_count % 15 == 1:
            w = msg.info.width
            h = msg.info.height
            res = msg.info.resolution
            self.get_logger().info(f"🗺️ [Costmap Stream] {w}x{h} cells @ {res:.2f}m/cell | Frame: {msg.header.frame_id}")

def main(args=None):
    rclpy.init(args=args)
    node = SocialNavBridgeListener()
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

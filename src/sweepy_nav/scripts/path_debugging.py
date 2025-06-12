#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from visualization_msgs.msg import Marker
import math
import os

class DualPathLogger(Node):
    def __init__(self):
        super().__init__('dual_path_logger')
        self.get_logger().info("Initializing DualPathLogger node...")

        # Path subscriptions
        self.plan_sub = self.create_subscription(Path, '/plan', self.plan_callback, 10)
        self.smoothed_sub = self.create_subscription(Path, '/plan_smoothed', self.smoothed_callback, 10)
        self.interpolated_sub = self.create_subscription(Path, '/plan_interpolated', self.interpolated_callback, 10)

        # Robot pose subscription
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)

        # Marker publisher
        self.marker_pub = self.create_publisher(Marker, 'path_markers', 10)

        # Internal state
        self.received_plan = False
        self.received_smoothed = False
        self.received_interpolated = False
        self.latest_interpolated_path = []

        self.base_dir = "/home/robot/sweepy_dev_ws/miscellaneous/fcpp_data"

    def plan_callback(self, msg):
        self.log_path(msg, "path.txt", "[PLAN]")
        self.received_plan = True
        self.check_shutdown()

    def smoothed_callback(self, msg):
        self.log_path(msg, "smoothed_path.txt", "[SMOOTHED]")
        self.publish_marker(msg)
        self.received_smoothed = True
        self.check_shutdown()

    def interpolated_callback(self, msg):
        self.latest_interpolated_path = msg.poses
        self.log_path(msg, "interpolated_path.txt", "[INTERPOLATED]")
        self.received_interpolated = True
        self.check_shutdown()

    def pose_callback(self, msg):
        if not self.latest_interpolated_path:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        closest_idx = -1
        min_dist = float('inf')

        for i, pose in enumerate(self.latest_interpolated_path):
            px = pose.pose.position.x
            py = pose.pose.position.y
            dist = math.hypot(px - x, py - y)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        self.get_logger().info(f"[POSE] Robot is closest to pose {closest_idx} (dist {min_dist:.2f} m)")

    def log_path(self, msg, filename, tag):
        self.get_logger().info(f"{tag} Received path with {len(msg.poses)} poses.")
        
        # Ensure the directory exists
        os.makedirs(self.base_dir, exist_ok=True)

        # Define the full file path
        file_path = os.path.join(self.base_dir, filename)

        with open(file_path, "w") as f:
            for i, pose in enumerate(msg.poses):
                x = pose.pose.position.x
                y = pose.pose.position.y
                f.write(f"Pose {i}: x={x}, y={y}\n")
        self.get_logger().info(f"{tag} Logged path to {file_path}.")

    def publish_marker(self, msg):
        # General path marker (greenish)
        marker = Marker()
        marker.header = msg.header
        marker.ns = "path_points"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 0.0
        marker.color.g = 0.4
        marker.color.b = 0.2
        marker.color.a = 1.0
        marker.lifetime.sec = 0

        for pose in msg.poses:
            p = Point()
            p.x = pose.pose.position.x
            p.y = pose.pose.position.y
            p.z = 0.0
            marker.points.append(p)

        self.marker_pub.publish(marker)
        self.get_logger().info(f"[MARKERS] Published {len(marker.points)} path markers.")

        # Last waypoint marker (purple)
        if msg.poses:
            last_pose = msg.poses[-1].pose.position
            last_marker = Marker()
            last_marker.header = msg.header
            last_marker.ns = "last_point"
            last_marker.id = 1
            last_marker.type = Marker.SPHERE
            last_marker.action = Marker.ADD
            last_marker.pose.position = last_pose
            last_marker.pose.orientation.w = 1.0
            last_marker.scale.x = 0.4
            last_marker.scale.y = 0.4
            last_marker.scale.z = 0.4
            last_marker.color.r = 0.6
            last_marker.color.g = 0.0
            last_marker.color.b = 0.8
            last_marker.color.a = 1.0
            last_marker.lifetime.sec = 0

            self.marker_pub.publish(last_marker)
            self.get_logger().info("[MARKERS] Published last waypoint marker in purple.")

    def check_shutdown(self):
        if self.received_plan and self.received_smoothed and self.received_interpolated:
            self.get_logger().info("[INFO] All paths received and visualized. Tracking pose...")

def main():
    rclpy.init()
    node = DualPathLogger()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

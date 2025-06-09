#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

class DualPathLogger(Node):
    def __init__(self):
        super().__init__('dual_path_logger')

        self.get_logger().info("Initializing DualPathLogger node...")

        self.plan_sub = self.create_subscription(
            Path,
            '/plan',
            self.plan_callback,
            10)
        
        self.smoothed_sub = self.create_subscription(
            Path,
            '/plan_smoothed',
            self.smoothed_callback,
            10)

        self.interpolated_sub = self.create_subscription(
            Path,
            '/plan_interpolated',
            self.interpolated_callback,
            10)

        self.received_plan = False
        self.received_smoothed = False
        self.received_interpolated = False
        self.latest_interpolated_path = []

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10)

    def plan_callback(self, msg):
        self.get_logger().info(f"[PLAN] Received /plan with {len(msg.poses)} poses.")
        filepath = "path.txt"
        with open(filepath, "w") as f:
            for i, pose in enumerate(msg.poses):
                x = pose.pose.position.x
                y = pose.pose.position.y
                f.write(f"Pose {i}: x={x}, y={y}\n")
        self.get_logger().info(f"[PLAN] Logged path to {filepath}.")
        self.received_plan = True
        self.check_shutdown()

    def smoothed_callback(self, msg):
        self.get_logger().info(f"[SMOOTHED] Received /plan_smoothed with {len(msg.poses)} poses.")
        filepath = "smoothed_path.txt"
        with open(filepath, "w") as f:
            for i, pose in enumerate(msg.poses):
                x = pose.pose.position.x
                y = pose.pose.position.y
                f.write(f"Pose {i}: x={x}, y={y}\n")
        self.get_logger().info(f"[SMOOTHED] Logged smoothed path to {filepath}.")
        self.received_smoothed = True
        self.check_shutdown()

    def interpolated_callback(self, msg):
        self.get_logger().info(f"[INTERPOLATED] Received /plan_interpolated with {len(msg.poses)} poses.")
        self.latest_interpolated_path = msg.poses  # Save for use in pose_callback

        filepath = "interpolated_path.txt"
        with open(filepath, "w") as f:
            for i, pose in enumerate(msg.poses):
                x = pose.pose.position.x
                y = pose.pose.position.y
                f.write(f"Pose {i}: x={x}, y={y}\n")
        self.get_logger().info(f"[INTERPOLATED] Logged interpolated path to {filepath}.")
        self.received_interpolated = True
        self.check_shutdown()

    def check_shutdown(self):
        if self.received_plan and self.received_smoothed and self.received_interpolated:
            self.get_logger().info("[INFO] All paths received and logged. Ready to track robot pose.")
            # self.get_logger().info("[SHUTDOWN] All paths received and logged. Shutting down node.")
            # rclpy.shutdown()

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
        
def main():
    rclpy.init()
    node = DualPathLogger()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

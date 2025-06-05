#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path

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
            self.get_logger().info("[SHUTDOWN] All paths received and logged. Shutting down node.")
            rclpy.shutdown()

def main():
    rclpy.init()
    node = DualPathLogger()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

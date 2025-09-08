#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path


class PlanListener(Node):
    def __init__(self):
        super().__init__('plan_listener')
        self.subscription = self.create_subscription(
            Path,
            '/coverage_server/coverage_plan',
            self.plan_callback,
            10
        )

    def plan_callback(self, msg: Path):
        if not msg.poses:
            self.get_logger().warn("Received empty global plan")
            return

        # First waypoint
        first_pose = msg.poses[0].pose.position
        # Last waypoint
        last_pose = msg.poses[-1].pose.position

        self.get_logger().info(
            f"First waypoint: x={first_pose.x:.2f}, y={first_pose.y:.2f}"
        )
        self.get_logger().info(
            f"Last waypoint:  x={last_pose.x:.2f}, y={last_pose.y:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PlanListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

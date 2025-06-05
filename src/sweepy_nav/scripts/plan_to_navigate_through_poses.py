#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses

class PlanToNavThroughPoses(Node):
    def __init__(self):
        super().__init__('plan_to_navigate_through_poses')
        self.client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.subscription = self.create_subscription(Path, '/plan', self.plan_callback, 10)
        self.sent = False

    def plan_callback(self, msg):
        if not self.sent and len(msg.poses) > 1:
            goal_msg = NavigateThroughPoses.Goal()
            goal_msg.poses = msg.poses
            goal_msg.behavior_tree = ''  # Or set to custom BT if needed

            self.client.wait_for_server()
            self.get_logger().info(f"Sending {len(msg.poses)} poses to NavigateThroughPoses...")
            self.client.send_goal_async(goal_msg)
            self.sent = True  # Prevent sending multiple times

def main():
    rclpy.init()
    node = PlanToNavThroughPoses()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

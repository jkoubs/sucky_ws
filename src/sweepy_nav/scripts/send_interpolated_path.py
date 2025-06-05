#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower_client')

        # Set up action client to /follow_path
        self._action_client = ActionClient(self, FollowPath, '/follow_path')

        # Subscribe to interpolated path
        self._sub = self.create_subscription(Path, '/plan_interpolated', self.path_callback, 10)
        self._last_sent = None
        self.get_logger().info('Waiting for interpolated path to send to MPPI...')

    def path_callback(self, msg):
        if len(msg.poses) < 2:
            self.get_logger().warn('Received path too short to send.')
            return

        if self._action_client.wait_for_server(timeout_sec=2.0) is False:
            self.get_logger().warn('FollowPath action server not available.')
            return

        # Avoid sending duplicate path
        if self._last_sent and self.paths_equal(self._last_sent, msg):
            return

        goal = FollowPath.Goal()
        goal.path = msg
        goal.controller_id = 'FollowPath'  # Make sure this matches your nav2_params.yaml
        goal.goal_checker_id = 'general_goal_checker'

        self.get_logger().info(f'Sending path with {len(msg.poses)} poses to FollowPath action...')
        self._action_client.send_goal_async(goal)
        self._last_sent = msg

    def paths_equal(self, p1, p2):
        if len(p1.poses) != len(p2.poses):
            return False
        for a, b in zip(p1.poses, p2.poses):
            if a.pose.position != b.pose.position:
                return False
        return True

def main():
    rclpy.init()
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

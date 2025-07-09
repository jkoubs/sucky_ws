#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose

import tf2_ros
from tf2_ros import TransformException


class SpiralPathLoader(Node):
    def __init__(self):
        super().__init__('spiral_path_loader')

        self.client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.pub = self.create_publisher(Path, '/precomputed_path', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.declare_parameter('goal_x', 5.0)
        self.declare_parameter('goal_y', 5.0)

    def wait_for_transform(self):
        parent = 'map'
        child = 'base_link'
        try:
            self.tf_buffer.lookup_transform(
                parent,
                child,
                rclpy.time.Time(),
                timeout=Duration(seconds=3).to_msg()
            )
            self.get_logger().info('TF available: map → base_link')
            return True
        except TransformException as ex:
            self.get_logger().error(f'TF not available: {ex}')
            return False

    def send_goal(self):
        if not self.wait_for_transform():
            self.get_logger().error('TF not available. Aborting path computation.')
            rclpy.shutdown()
            return

        goal_msg = ComputePathToPose.Goal()
        goal_msg.planner_id = 'SpiralSTC'

        goal_msg.goal.header.frame_id = 'map'
        goal_msg.goal.header.stamp = self.get_clock().now().to_msg()  # ✅ Add timestamp

        goal_msg.goal.pose.position.x = self.get_parameter('goal_x').value
        goal_msg.goal.pose.position.y = self.get_parameter('goal_y').value
        goal_msg.goal.pose.orientation.w = 1.0

        self.client.wait_for_server()
        self.get_logger().info("Sending goal to SpiralSTC planner...")
        self.future = self.client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('SpiralSTC goal was rejected by planner.')
            rclpy.shutdown()
            return

        self.get_logger().info('SpiralSTC goal accepted. Waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.pub.publish(result.path)
        self.get_logger().info('SpiralSTC path published to /precomputed_path.')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = SpiralPathLoader()
    node.send_goal()
    rclpy.spin(node)


if __name__ == '__main__':
    main()

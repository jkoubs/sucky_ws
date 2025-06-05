#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose, NavigateThroughPoses

class FullNavPipeline(Node):
    def __init__(self):
        super().__init__('full_nav_pipeline')

        self.plan_pub = self.create_publisher(Path, '/plan', 10)

        self.planner_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.nav_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

        self.get_logger().info('Waiting for planner and navigation action servers...')
        self.planner_client.wait_for_server()
        self.nav_client.wait_for_server()
        self.get_logger().info('Action servers ready.')

        self.send_planner_goal()

    def send_planner_goal(self):
        goal = ComputePathToPose.Goal()
        goal.goal = self.create_pose(2.0, 2.0)  # Replace with your actual goal
        goal.planner_id = 'GridBased'  # Or 'GridBased' if needed

        self.get_logger().info('Sending goal to ComputePathToPose...')
        future = self.planner_client.send_goal_async(goal)
        future.add_done_callback(self.planner_response_callback)

    def planner_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Planner goal rejected.')
            return

        self.get_logger().info('Planner goal accepted.')
        goal_handle.get_result_async().add_done_callback(self.planner_result_callback)

    def planner_result_callback(self, future):
        result = future.result().result
        plan = result.path
        self.get_logger().info(f'Planner succeeded with {len(plan.poses)} poses.')

        self.plan_pub.publish(plan)
        self.send_navigate_through_poses(plan)

    def send_navigate_through_poses(self, plan):
        if len(plan.poses) < 2:
            self.get_logger().warn('Too few poses to navigate.')
            return

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = plan.poses
        goal_msg.behavior_tree = ''  # Or set custom BT XML filename here

        self.get_logger().info('Sending poses to NavigateThroughPoses...')
        self.nav_client.send_goal_async(goal_msg)
        self.get_logger().info('Goal sent. Pipeline complete.')

    def create_pose(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0  # Facing forward
        return pose

def main():
    rclpy.init()
    node = FullNavPipeline()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

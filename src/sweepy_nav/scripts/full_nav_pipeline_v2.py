#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient

class FullNavPipeline(Node):
    def __init__(self):
        super().__init__('full_nav_pipeline')

        self.plan_pub = self.create_publisher(Path, '/plan', 10)

        self.planner_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.get_logger().info('Waiting for planner action server...')
        self.planner_client.wait_for_server()
        self.get_logger().info('Planner action server ready.')

        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Navigator ready.')

        self.send_planner_goal()

    def send_planner_goal(self):
        goal = ComputePathToPose.Goal()
        goal.goal = self.create_pose(2.0, 2.0)  # Replace with your actual goal
        goal.planner_id = 'GridBased'

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

        # Send poses in batches
        BATCH_SIZE = 10
        for i in range(0, len(plan.poses), BATCH_SIZE):
            sub_plan = plan.poses[i:i+BATCH_SIZE]
            if not sub_plan:
                continue

            self.get_logger().info(f'Sending batch from pose {i} to {i+len(sub_plan)-1}...')
            self.navigator.goThroughPoses(sub_plan)
            result = self.navigator.waitUntilNav2GoalReached()

            if not result:
                self.get_logger().warn(f'Navigation failed at batch starting at pose {i}')
                break

        self.get_logger().info('Navigation complete.')
        rclpy.shutdown()

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

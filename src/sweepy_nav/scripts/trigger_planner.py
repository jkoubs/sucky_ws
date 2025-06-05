#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped

class PlannerTriggerNode(Node):
    def __init__(self):
        super().__init__('planner_trigger_node')

        self.client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.client.wait_for_server()

        self.send_goal()

    def send_goal(self):
        goal = ComputePathToPose.Goal()
        goal.goal = self.create_pose(2.0, 2.0)  # Change to your goal
        goal.planner_id = 'GridBased'  # Use 'SpiralSTC' if needed

        self.get_logger().info(f'Sending goal to planner...')
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Planner goal was rejected.')
            return

        self.get_logger().info('Planner goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Got plan with {len(result.path.poses)} poses.')
        self.get_logger().info('Planner succeeded. Shutting down...')
        rclpy.shutdown()

    def create_pose(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0  # No rotation
        return pose

def main():
    rclpy.init()
    node = PlannerTriggerNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration
import numpy as np
import math

class PathInterpolator(Node):
    def __init__(self):
        super().__init__('path_interpolator')

        # Declare parameters
        self.declare_parameter('input_topic', '/plan_smoothed')
        self.declare_parameter('output_topic', '/plan_interpolated')
        self.declare_parameter('interpolation_steps', 10)

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.steps = self.get_parameter('interpolation_steps').get_parameter_value().integer_value

        self.subscription = self.create_subscription(Path, input_topic, self.interpolate_path, 10)
        self.publisher = self.create_publisher(Path, output_topic, 10)
        self.marker_pub = self.create_publisher(Marker, '/interpolated_path_markers', 10)

        self.get_logger().info(f'Interpolating from "{input_topic}" to "{output_topic}" with {self.steps} steps.')

    def interpolate_path(self, msg):
        if len(msg.poses) < 2:
            self.get_logger().warn('Not enough poses to interpolate.')
            return

        interp_path = Path()
        interp_path.header = msg.header

        for i in range(len(msg.poses) - 1):
            start = msg.poses[i]
            end = msg.poses[i+1]

            for t in np.linspace(0, 1, self.steps, endpoint=False):
                interp_pose = PoseStamped()
                interp_pose.header = msg.header
                interp_pose.pose.position.x = start.pose.position.x + t * (end.pose.position.x - start.pose.position.x)
                interp_pose.pose.position.y = start.pose.position.y + t * (end.pose.position.y - start.pose.position.y)
                interp_pose.pose.position.z = start.pose.position.z + t * (end.pose.position.z - start.pose.position.z)

                start_yaw = self.quaternion_to_yaw(start.pose.orientation)
                end_yaw = self.quaternion_to_yaw(end.pose.orientation)
                yaw = self.interpolate_angle(start_yaw, end_yaw, t)
                interp_pose.pose.orientation = self.yaw_to_quaternion(yaw)

                interp_path.poses.append(interp_pose)

        # Add final pose
        interp_path.poses.append(msg.poses[-1])
        self.publisher.publish(interp_path)
        self.publish_marker(interp_path)

    def quaternion_to_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.w = math.cos(yaw / 2)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2)
        return q

    def interpolate_angle(self, a, b, t):
        diff = ((b - a + np.pi) % (2 * np.pi)) - np.pi
        return a + diff * t

    def publish_marker(self, path_msg):
        marker = Marker()
        marker.header.frame_id = path_msg.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "interpolated_path"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 1.0
        marker.color.g = 0.4
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = Duration(sec=0)

        for pose in path_msg.poses:
            marker.points.append(pose.pose.position)

        self.marker_pub.publish(marker)


def main():
    rclpy.init()
    node = PathInterpolator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
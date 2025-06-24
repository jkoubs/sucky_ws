#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from tf_transformations import quaternion_from_euler, euler_from_quaternion

X, Y, Z = 0, 1, 2

class InterpolatorNode(Node):
    def __init__(self):
        super().__init__('path_interpolator')

        self.declare_parameter('resolution', 0.2)  # meters
        self.resolution = self.get_parameter('resolution').get_parameter_value().double_value

        self.sub = self.create_subscription(Path, '/plan_smoothed', self.on_path_received, 10)
        self.pub = self.create_publisher(Path, '/plan_interpolated', 10)
        self.marker_pub = self.create_publisher(Marker, '/interpolated_path_markers', 10)

        self.get_logger().info('Path Interpolator ready. Waiting for /plan_smoothed...')

    def on_path_received(self, msg):
        if len(msg.poses) < 2:
            self.get_logger().warn('Received path has fewer than 2 poses. Skipping.')
            return

        interpolated_path = Path()
        interpolated_path.header = msg.header
        all_points = []

        for i in range(len(msg.poses) - 1):
            from_pose = msg.poses[i]
            to_pose = msg.poses[i + 1]

            section = SectionInterpolator(from_pose, to_pose)
            poses = section.generate_interpolated_poses(self.resolution)
            interpolated_path.poses.extend(poses)
            all_points.extend(poses)

        interpolated_path.poses.append(msg.poses[-1])
        all_points.append(msg.poses[-1])

        self.pub.publish(interpolated_path)
        self.publish_marker(all_points, msg.header.frame_id)
        self.get_logger().info(f'Published interpolated path with {len(interpolated_path.poses)} poses.')

    def publish_marker(self, poses, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'interpolated_path'
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 1.0
        marker.color.g = 0.4
        marker.color.b = 0.0
        marker.color.a = 1.0

        for p in poses:
            marker.points.append(p.pose.position)

        self.marker_pub.publish(marker)


class SectionInterpolator:
    def __init__(self, from_pose, to_pose):
        self.from_pose = from_pose
        self.to_pose = to_pose

        self.start = np.array([from_pose.pose.position.x, from_pose.pose.position.y, from_pose.pose.position.z])
        self.end = np.array([to_pose.pose.position.x, to_pose.pose.position.y, to_pose.pose.position.z])
        self.delta = self.end - self.start
        self.length = np.linalg.norm(self.delta)

        start_q = from_pose.pose.orientation
        end_q = to_pose.pose.orientation
        self.start_yaw = euler_from_quaternion([start_q.x, start_q.y, start_q.z, start_q.w])[2]
        self.end_yaw = euler_from_quaternion([end_q.x, end_q.y, end_q.z, end_q.w])[2]
        self.delta_yaw = (self.end_yaw - self.start_yaw + np.pi) % (2 * np.pi) - np.pi

    def generate_interpolated_poses(self, resolution):
        if self.length == 0:
            return []

        num_steps = max(2, int(self.length / resolution))
        poses = []
        for i in range(num_steps):
            t = i / num_steps
            p = self.start + self.delta * t
            yaw = self.start_yaw + self.delta_yaw * t
            q = quaternion_from_euler(0, 0, yaw)

            ps = PoseStamped()
            ps.header.frame_id = self.from_pose.header.frame_id
            ps.pose.position.x = p[X]
            ps.pose.position.y = p[Y]
            ps.pose.position.z = p[Z]
            ps.pose.orientation.x = q[0]
            ps.pose.orientation.y = q[1]
            ps.pose.orientation.z = q[2]
            ps.pose.orientation.w = q[3]

            poses.append(ps)

        return poses


def main():
    rclpy.init()
    node = InterpolatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

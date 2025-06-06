#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import Path

class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')
        self.marker_pub = self.create_publisher(Marker, 'path_markers', 10)
        self.path_sub = self.create_subscription(Path, '/plan_smoothed', self.path_callback, 10)
        self.get_logger().info("Initializing PathVisualizer node...")

    def path_callback(self, msg):
        marker = Marker()
        marker.header = msg.header
        marker.ns = "path_points"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.3  # Sphere radius
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 0.0
        marker.color.g = 0.4
        marker.color.b = 0.2
        marker.color.a = 1.0
        marker.lifetime.sec = 0  # Forever

        for i, pose in enumerate(msg.poses):
            if i % 2 == 1:  # Only visualize odd-indexed points
                p = Point()
                p.x = pose.pose.position.x
                p.y = pose.pose.position.y
                p.z = 0.0
                marker.points.append(p)

        self.get_logger().info(f"Publishing {len(marker.points)} markers (only odd-indexed poses).")
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = PathVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

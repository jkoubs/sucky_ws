#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from sensor_msgs_py.point_cloud2 import create_cloud
import numpy as np

class PointCloudYFilter(Node):
    def __init__(self):
        super().__init__('pointcloud_y_filter')

        # Declare parameters
        self.declare_parameter('min_y', -0.409)
        self.declare_parameter('max_y', 0.591)

        # Set up subscription and publisher
        self.subscription = self.create_subscription(
            PointCloud2,
            '/depth/color/points',
            self.callback,
            10)
        self.publisher = self.create_publisher(
            PointCloud2,
            '/filtered_cloud_height',
            10)

    def callback(self, msg):
        try:
            cloud_array = point_cloud2.read_points_numpy(msg, skip_nans=True)
            total_points = cloud_array.shape[0]
            self.get_logger().info(f'Received raw point cloud with {total_points} valid points')

            y_values = cloud_array[:, 1]
            self.get_logger().info(f'Min Y: {np.min(y_values):.2f}, Max Y: {np.max(y_values):.2f}')

            # Log histogram of Y values
            # hist, bins = np.histogram(y_values, bins=20)
            # for h, b_start, b_end in zip(hist, bins[:-1], bins[1:]):
            #     self.get_logger().info(f"Y from {b_start:.2f} to {b_end:.2f}: {h} points")

            # Get Y filter range from parameters
            min_y = self.get_parameter('min_y').get_parameter_value().double_value
            max_y = self.get_parameter('max_y').get_parameter_value().double_value

            # Filter in Y range
            mask = (y_values >= min_y) & (y_values <= max_y)
            filtered_array = cloud_array[mask]
            filtered_count = filtered_array.shape[0]
            self.get_logger().info(f'Filtered point cloud has {filtered_count} points with {min_y} m <= Y <= {max_y} m')

            if filtered_count == 0:
                self.get_logger().warn('Filtered point cloud is empty! Nothing published.')
                return

            filtered_msg = create_cloud(msg.header, msg.fields, filtered_array)
            self.publisher.publish(filtered_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudYFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

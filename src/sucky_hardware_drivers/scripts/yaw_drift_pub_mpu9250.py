#!/usr/bin/env python3

"""
Run Madgwick Filter:

  ros2 run imu_filter_madgwick imu_filter_madgwick_node \
  --ros-args \
  -p use_mag:=true \
  -p publish_tf:=false \
  -p world_frame:="enu"

  
It publishes to /imu/data
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion
import math

class YawPublisher(Node):
    def __init__(self):
        super().__init__('yaw_mpu9250_publisher')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.initial_yaw = None
        self.initial_time = None

    def imu_callback(self, msg):
        # Convert quaternion to yaw
        q = msg.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quaternion)
        yaw_deg = math.degrees(yaw)

        # Get current timestamp in seconds
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Store initial values on first message
        if self.initial_yaw is None:
            self.initial_yaw = yaw_deg
            self.initial_time = now
            self.get_logger().info(f'Initial yaw set to {self.initial_yaw:.2f}° at {self.initial_time:.2f}s')
            return

        # Compute yaw drift
        drift = yaw_deg - self.initial_yaw
        drift = (drift + 180) % 360 - 180  # normalize to [-180, 180]

        # Compute elapsed time
        elapsed_time = now - self.initial_time
        drift_rate = drift / elapsed_time if elapsed_time > 0 else 0.0

        self.get_logger().info(
            f'Yaw: {yaw_deg:.2f}°, Drift: {drift:.2f}°, Time: {elapsed_time:.1f}s, Drift rate: {drift_rate:.3f}°/s'
        )

def main(args=None):
    rclpy.init(args=args)
    node = YawPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()







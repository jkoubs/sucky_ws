#!/usr/bin/env python3

"""
Subscribe to Madgwick-filtered IMU data and magnetometer data:

  ros2 run imu_filter_madgwick imu_filter_madgwick_node \
  --ros-args \
  -p use_mag:=true \
  -p publish_tf:=false \
  -p world_frame:="enu"

This script subscribes to:
- /imu/data  (orientation with quaternion)
- /imu/mag   (magnetometer vector)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from tf_transformations import euler_from_quaternion
import math

class YawMagPublisher(Node):
    def __init__(self):
        super().__init__('yaw_mag_mpu9250_publisher')

        # Subscriptions
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.create_subscription(MagneticField, '/imu/mag', self.mag_callback, 10)

    def imu_callback(self, msg):
        # Convert quaternion to yaw (ENU frame)
        q = msg.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quaternion)
        yaw_deg = math.degrees(yaw)
        self.get_logger().info(f'Yaw (Madgwick): {yaw_deg:.2f}°')

    def mag_callback(self, msg):
        mx = msg.magnetic_field.x * 1e6  # convert Tesla to µT
        my = msg.magnetic_field.y * 1e6
        mz = msg.magnetic_field.z * 1e6

        self.get_logger().info(f'Magnetometer [µT]: X={mx:.2f}, Y={my:.2f}, Z={mz:.2f}')
        
        # Check for abnormal values
        for axis, value in zip(['X', 'Y', 'Z'], [mx, my, mz]):
            if abs(value) > 65:
                self.get_logger().error(f'[Mag Error] {axis} axis exceeds ±65 µT: {value:.2f} µT')

def main(args=None):
    rclpy.init(args=args)
    node = YawMagPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import math
import time

class ImuSerialPublisher(Node):
    def __init__(self):
        super().__init__('imu_serial_publisher')
        self.publisher_ = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        time.sleep(2)  # Give Arduino time to reboot

    def read_and_publish(self):
        if self.serial_port.in_waiting:
            line = self.serial_port.readline().decode('utf-8').strip()

            if "Accel" in line and "Gyro" in line:
                try:
                    # Parse the string
                    acc_str = line.split("Accel [g]: ")[1].split(" | ")[0]
                    gyro_str = line.split("Gyro [°/s]: ")[1].split(" | ")[0]
                    ax, ay, az = map(float, acc_str.split(", "))
                    gx, gy, gz = map(float, gyro_str.split(", "))

                    imu_msg = Imu()
                    imu_msg.header.frame_id = 'imu_link'
                    imu_msg.header.stamp = self.get_clock().now().to_msg()

                    # Accelerations: convert from g to m/s²
                    imu_msg.linear_acceleration.x = ax * 9.80665
                    imu_msg.linear_acceleration.y = ay * 9.80665
                    imu_msg.linear_acceleration.z = az * 9.80665

                    # Angular velocity: convert from deg/s to rad/s
                    imu_msg.angular_velocity.x = math.radians(gx)
                    imu_msg.angular_velocity.y = math.radians(gy)
                    imu_msg.angular_velocity.z = math.radians(gz)

                    # Leave orientation empty (imu_filter will compute it)
                    imu_msg.orientation_covariance[0] = -1.0

                    self.publisher_.publish(imu_msg)
                except Exception as e:
                    self.get_logger().warn(f"Parse error: {e}")

def main():
    rclpy.init()
    node = ImuSerialPublisher()
    try:
        while rclpy.ok():
            node.read_and_publish()
    except KeyboardInterrupt:
        pass
    finally:
        node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

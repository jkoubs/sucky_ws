#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from smbus2 import SMBus
import math
import time

class MPU9250:
    def __init__(self, bus_number=7, address=0x68, gyro_range=0, accel_range=0, dlpf_bandwidth=2):
        self.bus = SMBus(bus_number)
        self.address = address
        self.bus.write_byte_data(self.address, 0x6B, 0x00)  # Wake up MPU9250

        # Set ranges and DLPF
        self.set_gyro_range(gyro_range)
        self.set_accel_range(accel_range)
        self.set_dlpf_bandwidth(dlpf_bandwidth)

        # Calibration offsets
        self.accel_offset = [0.0, 0.0, 0.0]
        self.gyro_offset = [0.0, 0.0, 0.0]

    def set_gyro_range(self, range_setting):
        self.bus.write_byte_data(self.address, 0x1B, range_setting << 3)

    def set_accel_range(self, range_setting):
        self.bus.write_byte_data(self.address, 0x1C, range_setting << 3)

    def set_dlpf_bandwidth(self, bandwidth):
        self.bus.write_byte_data(self.address, 0x1A, bandwidth)

    def read_raw_data(self, addr):
        high = self.bus.read_byte_data(self.address, addr)
        low = self.bus.read_byte_data(self.address, addr + 1)
        value = ((high << 8) | low)
        if value > 32768:
            value = value - 65536
        return value

    def read_imu_data(self):
        accel_x = self.read_raw_data(0x3B)
        accel_y = self.read_raw_data(0x3D)
        accel_z = self.read_raw_data(0x3F)

        gyro_x = self.read_raw_data(0x43)
        gyro_y = self.read_raw_data(0x45)
        gyro_z = self.read_raw_data(0x47)

        accel_x = accel_x / 16384.0 - self.accel_offset[0]
        accel_y = accel_y / 16384.0 - self.accel_offset[1]
        accel_z = accel_z / 16384.0 - self.accel_offset[2]

        gyro_x = gyro_x / 131.0 - self.gyro_offset[0]
        gyro_y = gyro_y / 131.0 - self.gyro_offset[1]
        gyro_z = gyro_z / 131.0 - self.gyro_offset[2]

        return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z

    def calibrate_sensor(self, num_samples=100):
        accel_sum = [0.0, 0.0, 0.0]
        gyro_sum = [0.0, 0.0, 0.0]

        self.bus.write_byte_data(self.address, 0x6B, 0x00)  # Ensure the sensor is awake
        print("Calibrating IMU... Please keep the sensor stationary.")

        for _ in range(num_samples):
            accel_x = self.read_raw_data(0x3B)
            accel_y = self.read_raw_data(0x3D)
            accel_z = self.read_raw_data(0x3F)

            gyro_x = self.read_raw_data(0x43)
            gyro_y = self.read_raw_data(0x45)
            gyro_z = self.read_raw_data(0x47)

            accel_sum[0] += accel_x / 16384.0
            accel_sum[1] += accel_y / 16384.0
            accel_sum[2] += accel_z / 16384.0

            gyro_sum[0] += gyro_x / 131.0
            gyro_sum[1] += gyro_y / 131.0
            gyro_sum[2] += gyro_z / 131.0

            time.sleep(0.01)  # Small delay between readings

        self.accel_offset = [x / num_samples for x in accel_sum]
        self.gyro_offset = [x / num_samples for x in gyro_sum]

        self.accel_offset[2] -= 1.0  # Subtract gravity from Z-axis

        print("Calibration complete.")
        print(f"Accelerometer offset: {self.accel_offset}")
        print(f"Gyroscope offset: {self.gyro_offset}")

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        # Declare parameters
        self.declare_parameter('calibrate', True)
        self.declare_parameter('gyro_range', 0)
        self.declare_parameter('accel_range', 0)
        self.declare_parameter('dlpf_bandwidth', 2)
        self.declare_parameter('frequency', 100)

        # Retrieve parameters
        calibrate = self.get_parameter('calibrate').value
        gyro_range = self.get_parameter('gyro_range').value
        accel_range = self.get_parameter('accel_range').value
        dlpf_bandwidth = self.get_parameter('dlpf_bandwidth').value
        frequency = self.get_parameter('frequency').value

        self.publisher_ = self.create_publisher(Imu, 'imu/data_raw', 10)
        timer_period = 1.0 / frequency  # seconds
        self.timer = self.create_timer(timer_period, self.publish_imu_data)
        self.imu_sensor = MPU9250(gyro_range=gyro_range, accel_range=accel_range, dlpf_bandwidth=dlpf_bandwidth)

        if calibrate:
            self.imu_sensor.calibrate_sensor()

    def publish_imu_data(self):
        imu_msg = Imu()

        accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = self.imu_sensor.read_imu_data()

        imu_msg.linear_acceleration.x = accel_x * 9.80665
        imu_msg.linear_acceleration.y = accel_y * 9.80665
        imu_msg.linear_acceleration.z = accel_z * 9.80665

        imu_msg.angular_velocity.x = math.radians(gyro_x)
        imu_msg.angular_velocity.y = math.radians(gyro_y)
        imu_msg.angular_velocity.z = math.radians(gyro_z)

        self.publisher_.publish(imu_msg)
        self.get_logger().info('Publishing IMU data...')

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = ImuPublisher()
    rclpy.spin(imu_publisher)
    imu_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

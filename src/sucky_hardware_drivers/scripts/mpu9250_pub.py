#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import serial
import math
import time

class ImuSerialPublisher(Node):
    def __init__(self):
        super().__init__('imu_serial_publisher')
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 10)

        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        time.sleep(2)  # Give Arduino time to reboot

    def read_and_publish(self):
        if self.serial_port.in_waiting:
            line = self.serial_port.readline().decode('utf-8').strip()
            
            if "Gyro:" in line and "Accel:" in line and "Mag:" in line:
                try:
                    parts = line.split(" | ")
                    gyro_str = parts[0].replace("Gyro: ", "")
                    accel_str = parts[1].replace("Accel: ", "")
                    mag_str = parts[2].replace("Mag: ", "")

                    gx, gy, gz = map(float, gyro_str.split(", "))
                    ax, ay, az = map(float, accel_str.split(", "))
                    mx, my, mz = map(float, mag_str.split(", "))
            # if line.startswith("IMU_RAW:"):
            #     try:
            #         data = line.replace("IMU_RAW:", "").split(",")
            #         if len(data) != 9:
            #             return
                    
            #         gx, gy, gz = map(float, data[0:3])  # deg/s
            #         ax, ay, az = map(float, data[3:6])  # g
            #         mx, my, mz = map(float, data[6:9])  # µT

                    now = self.get_clock().now().to_msg()

                    # === Publish IMU ===
                    imu_msg = Imu()
                    imu_msg.header.frame_id = 'imu_link'
                    imu_msg.header.stamp = now

                    imu_msg.linear_acceleration.x = ax * 9.80665
                    imu_msg.linear_acceleration.y = ay * 9.80665
                    imu_msg.linear_acceleration.z = az * 9.80665

                    imu_msg.angular_velocity.x = math.radians(gx)
                    imu_msg.angular_velocity.y = math.radians(gy)
                    imu_msg.angular_velocity.z = math.radians(gz)

                    imu_msg.orientation_covariance[0] = -1.0  # No orientation yet

                    self.imu_pub.publish(imu_msg)

                    # === Publish Magnetic Field ===
                    mag_msg = MagneticField()
                    mag_msg.header.frame_id = 'imu_link'
                    mag_msg.header.stamp = now

                    mag_msg.magnetic_field.x = mx * 1e-6  # convert µT -> Tesla
                    mag_msg.magnetic_field.y = my * 1e-6
                    mag_msg.magnetic_field.z = mz * 1e-6

                    self.mag_pub.publish(mag_msg)

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

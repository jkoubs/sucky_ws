#!/usr/bin/env python3

import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import serial
import time 

class RelayControllerNode(Node):
    def __init__(self):
        super().__init__('relay_controller_node')
        self.subscription = self.create_subscription(
             Joy, 
            'joy',
            self.joy_callback,
            10
        )
        self.subscription 

        # Debounce setup 
        self.last_command_time = time.time()
        self.debounce_duration = 0.5
        
        # setup serial commuication with esp32 
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            time.sleep(2)
            self.get_logger().info("Serial connection established")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            raise 


    def joy_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_command_time < self.debounce_duration:
            return 
        try:
            if msg.buttons[4] == 1: # Button index 6 is pressed  
                self.ser.write(b'RELAY1_ON\n')
                self.ser.write(b'RELAY3_ON\n')
                self.get_logger().info("Sent: RELAY1_ON")
                self.get_logger().info("Sent: RELAY3_ON")
                
                self.last_command_time = current_time
            elif msg.buttons[5] == 1: # condition to turn off the relay 
                self.ser.write(b'RELAY1_OFF\n')
                self.ser.write(b'RELAY3_OFF\n')
                self.get_logger().info("Sent: RELAY1_OFF")
                self.get_logger().info("Sent: RELAY3_OFF")

                self.last_command_time = current_time
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to write to serial port: {e}")


    def read_serial(self):
        try:
            if self.ser.in_waiting > 0:
                raw_data = self.ser.readline()
                try:
                    data = raw_data.decode('utf-8', errors='ignore').strip()
                    self.get_logger().info(f"Recieved from Arduino: {data}")
                except UnicodeDecodeError as e:
                    self.get_logger().warn(f"Failed to decode serial data: {e}")
        except serial.SerialException as e:
                self.get_logger().error(f"Serial read error: {e}")
        
def main(args=None):
    rclpy.init(args=args)
    relay_controller_node = RelayControllerNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(relay_controller_node, timeout_sec=0.1)
            relay_controller_node.read_serial()
    except KeyboardInterrupt:
        pass
    finally:
        relay_controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()






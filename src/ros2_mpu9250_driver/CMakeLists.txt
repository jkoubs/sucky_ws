cmake_minimum_required(VERSION 3.5)
project(mpu9250driver)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)

# Install Python scripts
install(PROGRAMS
  src/mpu9250driver_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and params
install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

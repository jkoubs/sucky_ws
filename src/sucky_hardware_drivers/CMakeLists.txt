cmake_minimum_required(VERSION 3.8)
project(sucky_hardware_drivers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)


# Install directories
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)


# Install scripts
install(PROGRAMS
    scripts/relay_controller_node.py 
    scripts/mpu9250_pub.py
    scripts/mpu9250_no_mag_pub.py
    scripts/yaw_drift_pub_mpu9250.py
    scripts/yaw_mag_pub_mpu9250.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

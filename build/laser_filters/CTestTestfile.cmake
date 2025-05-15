# CMake generated Testfile for 
# Source directory: /home/robot/sweepy_dev_ws/src/laser_filters
# Build directory: /home/robot/sweepy_dev_ws/build/laser_filters
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_scan_filter_chain "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/robot/sweepy_dev_ws/build/laser_filters/test_results/laser_filters/test_scan_filter_chain.gtest.xml" "--package-name" "laser_filters" "--command" "/home/robot/sweepy_dev_ws/build/laser_filters/test_scan_filter_chain" "--ros-args" "--params-file" "/home/robot/sweepy_dev_ws/src/laser_filters/test/test_scan_filter_chain.yaml" "--gtest_output=xml:/home/robot/sweepy_dev_ws/build/laser_filters/test_results/laser_filters/test_scan_filter_chain.gtest.xml")
set_tests_properties(test_scan_filter_chain PROPERTIES  TIMEOUT "60" WORKING_DIRECTORY "/home/robot/sweepy_dev_ws/build/laser_filters" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/home/robot/sweepy_dev_ws/src/laser_filters/CMakeLists.txt;58;ament_add_test;/home/robot/sweepy_dev_ws/src/laser_filters/CMakeLists.txt;0;")
add_test(test_shadow_detector "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/robot/sweepy_dev_ws/build/laser_filters/test_results/laser_filters/test_shadow_detector.gtest.xml" "--package-name" "laser_filters" "--command" "/home/robot/sweepy_dev_ws/build/laser_filters/test_shadow_detector" "--ros-args" "--gtest_output=xml:/home/robot/sweepy_dev_ws/build/laser_filters/test_results/laser_filters/test_shadow_detector.gtest.xml")
set_tests_properties(test_shadow_detector PROPERTIES  TIMEOUT "60" WORKING_DIRECTORY "/home/robot/sweepy_dev_ws/build/laser_filters" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/home/robot/sweepy_dev_ws/src/laser_filters/CMakeLists.txt;72;ament_add_test;/home/robot/sweepy_dev_ws/src/laser_filters/CMakeLists.txt;0;")
subdirs("gtest")

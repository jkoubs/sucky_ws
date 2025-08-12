# sucky_ws


# Instructions

### full_coverage_path_planning

```bash
ros2 launch sucky_bringup sim_fcpp_bringup.launch.py
ros2 launch sucky_nav sim_fcpp_visualizers.launch.py
```

### opennav_coverage

```bash
ros2 launch sucky_nav sim_coverage_server.launch.py 
ros2 launch sucky_bringup sim_opennav_bringup.launch.py 
ros2 run sucky_nav demo_coverage.py
```

### RTAB-Map SIM

```bash
ros2 launch sucky_bringup sim_bringup.launch.py
ros2 launch sucky_rtabmap sim_rtabmap.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diffbot_base_controller/cmd_vel_unstamped
ros2 run nav2_map_server map_saver_cli -f my_2d_map
rtabmap-databaseViewer ~/.ros/rtabmap.db
```

### Collect RTAB-Map INput Topics [REAL]

```bash
ssh_sucky
ros2 launch sucky_bringup bringup.launch.py
ros2 bag record \
  --compression-mode file \
  --compression-format zstd \
  -o ~/bags/rosbag2_$(date +%Y_%m_%d-%H_%M_%S) \
  /diffbot_base_controller/odom \
  /camera/d455/color/camera_info \
  /camera/d455/color/image_raw/compressed \
  /camera/d455/depth/image_rect_raw/compressedDepth \
  /scan \
  /tf \
  /tf_static


ros2 topic hz /camera/d455/color/camera_info
ros2 topic hz /camera/d455/color/image_raw
ros2 topic hz /camera/d455/depth/image_rect_raw/compressedDepth
```


### IMU

```bash
ros2 run sucky_hardware_drivers mpu9250_pub.py
ros2 run imu_filter_madgwick imu_filter_madgwick_node   --ros-args   -p use_mag:=true   -p world_frame:="ned"
ros2 run sucky_perception yaw_pub_mpu9250.py
```


### Detect Ports

```bash
python3 ~/sucky_robot/src/sucky/tools/detect_serial_ports.py
```

### Edit ros2 control

```bash
nano ~/sucky_robot/src/sucky/urdf/ros2_control.xacro 
```
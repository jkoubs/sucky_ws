# Instructions

### full_coverage_path_planning

```bash
cd ~/sucky_ws/
colcon build
source install/setup.bash
ros2 launch sucky_bringup sim_fcpp_bringup.launch.py
```

```bash
cd ~/sucky_ws/
colcon build --packages-select sucky_nav
source install/setup.bash
ros2 launch sucky_nav sim_fcpp_visualizers.launch.py
```

```bash
gz model --spawn-file=/home/robot/sucky_ws/src/sucky_bringup/models/cylinder_obstacle/model.sdf --model-name="Cylinder Obstacle" -x -1.16 -y -0.49 -z 0.76
```

### opennav_coverage

```bash
cd ~/sucky_ws/
colcon build --packages-select sucky_nav
source install/setup.bash
ros2 launch sucky_nav sim_coverage_server.launch.py 
```

```bash
cd ~/sucky_ws/
colcon build --packages-select sucky_bringup
source install/setup.bash
ros2 launch sucky_bringup sim_opennav_bringup.launch.py 
```

```bash
cd ~/sucky_ws/
colcon build --packages-select sucky_nav
source install/setup.bash
ros2 run sucky_nav demo_coverage_optimized.py
```


### RTAB-Map SIM

```bash
ros2 launch sucky_bringup sim_bringup.launch.py
ros2 launch sucky_rtabmap sim_rtabmap.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diffbot_base_controller/cmd_vel_uncstamped
rtabmap-databaseViewer ~/.ros/rtabmap.db
```

### Collect RTAB-Map Input Topics [REAL]

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
```


### Foxglove

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
foxglove-studio
```


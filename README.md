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

### RTAB-Map

```bash
ros2 launch sucky_bringup sim_bringup.launch.py
ros2 launch sucky_rtabmap sim_rtabmap.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diffbot_base_controller/cmd_vel_unstamped
ros2 run nav2_map_server map_saver_cli -f my_2d_map
rtabmap-databaseViewer ~/.ros/rtabmap.db
```
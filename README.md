# sucky_ws


# Instructions

### full_coverage_path_planning

```bash
ros2 launch sweepy_bringup sim_fcpp_bringup.launch.py
ros2 launch sweepy_nav sim_fcpp_visualizers.launch.py
```

### opennav_coverage

```bash
ros2 launch sweepy_nav sim_coverage_server.launch.py 
ros2 launch sweepy_bringup sim_opennav_bringup.launch.py 
ros2 run sweepy_nav demo_coverage.py
```
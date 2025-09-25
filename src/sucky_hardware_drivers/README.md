# Testing IMU (MPU9250)

## Calibration Steps
Before using the IMU, each sensor must be calibrated to ensure accurate readings.

1. **Gyroscope Calibration** – Zeroes out bias to measure angular velocity accurately.  
2. **Accelerometer Calibration** – Aligns measurements with gravity and corrects offsets.  
3. **Magnetometer Calibration** – Corrects hard and soft iron distortions for accurate heading.


## Instructions

```bash
ros2 run sucky_hardware_drivers mpu9250_pub.py
ros2 run imu_filter_madgwick imu_filter_madgwick_node   --ros-args   -p use_mag:=true   -p world_frame:="ned"
ros2 run sucky_perception yaw_pub_mpu9250.py
```
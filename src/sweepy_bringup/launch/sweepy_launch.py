import os
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = FindPackageShare(package='sweeper_bot').find('sweeper_bot')
    default_model_path = os.path.join(pkg_share, 'src', 'description', 'robot.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'config', 'urdf_config.rviz')
    ekf_params_file = os.path.join(get_package_share_directory('sweeper_bot'), 'config', 'ekf_params.yaml')

    robot_description_content = Command(['xacro ', LaunchConfiguration('model')])
    robot_description = {'robot_description': robot_description_content}



    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file, {'use_sim_time': False}]
    )

    robot_controllers_path = PathJoinSubstitution([
        FindPackageShare("sweeper_bot"),
        "config",
        "roboclaw_controllers.yaml"
    ])

    
    sick_scan_pkg_prefix = get_package_share_directory('sick_scan_xd')
    tim_launch_file_path = os.path.join(sick_scan_pkg_prefix, 'launch/sick_tim_7xx.launch')
    
    sick_node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        output= 'screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            tim_launch_file_path,
            'tf_base_frame_id:=sick_lidar_frame',
            'tf_publish_rate:=30.0',
        ]
    )
    
    scan_config = os.path.join(get_package_share_directory('sweeper_bot'),'config','scan_filter_params.yaml')
    scan_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        parameters=[scan_config, {'use_sim_time': False}],
    )

    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('mpu9250driver'), 'launch', 'mpu9250driver_launch.py'
        )])
    )


    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        #joint_state_publisher_node,
        #imu,

        ekf_node,
        sick_node,
        scan_filter,
    ])

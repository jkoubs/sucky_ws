import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction


def generate_launch_description():
    bringup_pkg = get_package_share_directory('sweepy_bringup')
    controllers_config = os.path.join(bringup_pkg, 'config', 'sim_controllers.yaml')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')

    # Controller Manager (ros2_control_node)
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='both'
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='both'
    )

    # Diff Drive Controller
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diffbot_base_controller', '--controller-manager', '/controller_manager'],
        output='both'
    )

    # Joint State Broadcaster with delay
    delay_joint_broadcaster = TimerAction(
        period=5.0,
        actions=[joint_state_broadcaster_spawner]
    )

    # Diff Drive Controller with delay
    delay_diff_drive_controller = TimerAction(
        period=8.0,
        actions=[diff_drive_controller_spawner]
    )

    return LaunchDescription([
        declare_use_sim_time,
        ros2_control_node,
        delay_joint_broadcaster,
        delay_diff_drive_controller,
    ])

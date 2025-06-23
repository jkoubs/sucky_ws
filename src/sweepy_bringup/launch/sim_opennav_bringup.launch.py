import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_pkg = get_package_share_directory('sweepy_bringup')
    nav_pkg = get_package_share_directory('sweepy_nav')
    rviz_config = os.path.join(nav_pkg, 'rviz', 'opennav_coverage.rviz')  # Make sure this file exists

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'sim_bringup.launch.py')
        )
    )

    amcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, 'launch', 'sim_amcl.launch.py')
        )
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, 'launch', 'sim_opennav_coverage.launch.py')
        )
    )

    delayed_navigation = TimerAction(
        period=7.0,  # Adjust if needed
        actions=[navigation]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    return LaunchDescription([
        bringup,
        amcl,
        # navigation,
        delayed_navigation,
        rviz_node
    ])

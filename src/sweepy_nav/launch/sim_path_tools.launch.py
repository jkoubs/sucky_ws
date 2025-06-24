from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sweepy_nav',
            executable='path_interpolator.py',
            name='path_interpolator',
            output='screen'
        ),
        Node(
            package='sweepy_nav',
            executable='send_interpolated_path.py',
            name='send_interpolated_path',
            output='screen'
        ),
        Node(
            package='sweepy_nav',
            executable='path_debugging.py',
            name='path_debugging',
            output='screen'
        ),
        Node(
            package='sweepy_nav',
            executable='cleaning_visualizer.py',
            name='cleaning_visualizer',
            output='screen'
        )
    ])

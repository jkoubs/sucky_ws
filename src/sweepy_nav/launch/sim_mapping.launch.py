from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    sweepy_nav_dir = get_package_share_directory('sweepy_nav')
    params_file = os.path.join(sweepy_nav_dir, 'config', 'mapper_params_online_async.yaml')
    rviz_config = os.path.join(sweepy_nav_dir, 'rviz', 'mapping.rviz') 

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[params_file]
    )

    # Rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        slam_node,
        rviz_node
    ])

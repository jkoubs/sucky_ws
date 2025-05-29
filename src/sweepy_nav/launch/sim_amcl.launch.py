from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess, TimerAction


from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    sweepy_nav_dir = get_package_share_directory('sweepy_nav')

    map_file = os.path.join(sweepy_nav_dir, 'maps', '1_obstacle.yaml')  # Update to match your map file name
    amcl_params = os.path.join(sweepy_nav_dir, 'config', 'nav2_params.yaml')  # Create or reference your AMCL param file here
    rviz_config = os.path.join(sweepy_nav_dir, 'rviz', 'localization.rviz')  # Make sure this file exists
    
    # Map Server Node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file}]
    )

    # AMCL Node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params]
    )

    # Lifecycle Manager Node
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )

    # Publish initial pose after  delay
    initial_pose_node = Node(
        package='sweepy_nav',
        executable='initial_pose_publisher.py',
        name='initial_pose_publisher',
        output='screen'
    )


    return LaunchDescription([
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
        rviz_node,
        initial_pose_node
    ])
 
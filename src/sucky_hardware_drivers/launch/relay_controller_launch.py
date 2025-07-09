import os 
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node 

def generate_launch_description():

    relay_controller_node = Node(
        package='sucky_hardware_drivers',
        executable='relay_controller_node.py',
        name='relay_controller_node',
        output='screen'
    )

    #Launch!
    return LaunchDescription([
        relay_controller_node
    ])
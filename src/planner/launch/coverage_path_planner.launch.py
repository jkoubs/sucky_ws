import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    boustrophedon_decomposition_arg = DeclareLaunchArgument(
        "boustrophedon_decomposition", default_value="true", description="Whether to execute the boustrophedon decomposition"
    )
    border_drive_arg = DeclareLaunchArgument(
        "border_drive", default_value="false", description="Drive around the cell first"
    )

    # Create the node
    path_coverage_node = Node(
        package="planner",
        executable="path_coverage_ros",
        name="path_coverage",
        output="screen",
        parameters=[
            {"boustrophedon_decomposition": LaunchConfiguration("boustrophedon_decomposition")},
            {"border_drive": LaunchConfiguration("border_drive")},
            {"robot_width": 0.3},
            {"costmap_max_non_lethal": 70},
            {"base_frame": "base_link"},
        ],
    )

    return LaunchDescription([
        boustrophedon_decomposition_arg,
        border_drive_arg,
        path_coverage_node,
    ])

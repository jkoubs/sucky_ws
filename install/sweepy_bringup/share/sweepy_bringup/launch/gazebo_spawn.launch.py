import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_pkg = FindPackageShare(package='sweepy_bringup').find('sweepy_bringup')

    sweepy_model = os.path.join(bringup_pkg, 'urdf', 'gazebo_robot.urdf.xacro')
    rviz_config = os.path.join(bringup_pkg, 'rviz', 'spawn.rviz')
    sweepy_world = os.path.join(bringup_pkg, 'worlds', 'empty.world')  
    controllers_config = os.path.join(bringup_pkg, 'config', 'sim_controllers.yaml')


    # Launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation clock if true'
    )
    declare_world = DeclareLaunchArgument(
        'world', default_value=sweepy_world, description='World file to load in Gazebo'
    )
    declare_model = DeclareLaunchArgument(
        'model', default_value=sweepy_model, description='Absolute path to robot urdf file'
    )
    declare_rvizconfig = DeclareLaunchArgument(
        'rvizconfig', default_value=rviz_config, description='Absolute path to rviz config file'
    )

    # Process the robot description using xacro
    robot_description_content = Command(['xacro ', LaunchConfiguration('model')])
    robot_description = {'robot_description': robot_description_content}

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world'), 'verbose': 'true'}.items()
    )

    # Spawn the robot entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'sweeper_bot',
            '-x', '0',
            '-y', '0',
            '-z', '0.5'  # Slightly above ground to avoid collision issues
        ],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    # Robot state publisher 
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sweepy_bringup'), 'launch', 'rsp.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_ros2_control': 'true'
        }.items()
    )

    # RViz2 visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # Controller manager node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output="both",
    )

    # Spawner for joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="both",
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    # Spawner for differential drive controller
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
        output="both",
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    # Optional: Laser scan filter
    scan_config = os.path.join(get_package_share_directory('sweepy_perception'), 'config', 'scan_filter_params.yaml')
    scan_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        parameters=[scan_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        declare_model,
        declare_rvizconfig,
        gazebo_launch,
        rsp,
        spawn_entity,
        rviz_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        #scan_filter,
    ])

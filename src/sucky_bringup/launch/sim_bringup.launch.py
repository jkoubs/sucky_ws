import os
import xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    bringup_pkg = get_package_share_directory('sucky_bringup')
    rviz_config_path = os.path.join(
        get_package_share_directory('sucky_bringup'),'rviz','camera.rviz')
    

    # Gazebo Launch
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
    #     launch_arguments={'world': os.path.join(bringup_pkg, 'worlds', 'empty.world')}.items()
    # )

    # Gazebo Launch with safe plugin overrides
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': os.path.join(bringup_pkg, 'worlds', 'T_shape_obstacles.world'),
            'gui_plugins': '[]',
            'server_required_plugins': "['libgazebo_ros_init.so','libgazebo_ros_factory.so']"
        }.items()
    )

    # Load and Process Xacro
    xacro_file = os.path.join(bringup_pkg, 'urdf', 'gazebo_robot.urdf.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params, {'use_sim_time': True}]
    )

    # Spawn Robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'sucky',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
        ],
        output='screen'
    )

    # Load Controllers After Spawn
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'diffbot_base_controller'],
        output='screen'
    )

    # Rviz config
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', rviz_config_path],
    # )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        RegisterEventHandler(OnProcessExit(target_action=spawn_entity, on_exit=[load_joint_state_broadcaster])),
        RegisterEventHandler(OnProcessExit(target_action=load_joint_state_broadcaster, on_exit=[load_diff_drive_controller])),
        #rviz_node
    ])

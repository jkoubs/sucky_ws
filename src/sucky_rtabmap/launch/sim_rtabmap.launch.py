from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    localization = LaunchConfiguration('localization')

    parameters = {
        'frame_id': 'base_link',  # Change if your robot base frame is different
        'odom_frame_id': 'odom',
        'use_sim_time': True,
        'subscribe_rgbd': True,
        'subscribe_scan': True,
        'approx_sync': True,
        'sync_queue_size': 10,
        'Reg/Strategy': '1',  # Use ICP
        'Reg/Force3DoF': 'true',  # 2D SLAM
        # 'Grid/FromDepth': 'false',  # Use 2D LiDAR, not depth for grid map
        'Icp/MaxCorrespondenceDistance': '0.2',
        'Icp/PointToPlane': 'false',
        'Icp/PM': 'false',
        'RGBD/ProximityBySpace': 'true',
        'Mem/STMSize': '30',

        # Enable 3D point cloud and OctoMap outputs
        'Rtabmap/Publish3DClouds': True,
        'Rtabmap/PublishOctoMap': True,
        'Grid/FromDepth': True,  # REQUIRED to activate cloud generation from depth camera

    }

    # parameters = {
    #     'frame_id': 'base_footprint',
    #     'odom_frame_id': 'odom',
    #     'odom_tf_linear_variance': 0.001,
    #     'odom_tf_angular_variance': 0.001,
    #     'subscribe_rgbd': True,
    #     'subscribe_scan': True,
    #     'approx_sync': True,
    #     'sync_queue_size': 10,

    #     # RTAB-Map internal params (must be strings!)
    #     'RGBD/NeighborLinkRefining': 'true',
    #     'RGBD/ProximityBySpace': 'true',
    #     'RGBD/ProximityByTime': 'false',
    #     'RGBD/ProximityPathMaxNeighbors': '10',
    #     'Reg/Strategy': '1',  # Use ICP only
    #     'Vis/MinInliers': '12',
    #     'RGBD/OptimizeFromGraphEnd': 'false',
    #     'RGBD/OptimizeMaxError': '4',
    #     'Reg/Force3DoF': 'true',  # Enforce 2D SLAM

    #     # Use LiDAR instead of depth for grid
    #     # 'Grid/FromDepth': 'false',

    #     # Filter obstacles based on height
    #     'Grid/MaxObstacleHeight': '1.0',

    #     # ICP tuning
    #     'Icp/CorrespondenceRatio': '0.2',
    #     'Icp/PM': 'false',
    #     'Icp/PointToPlane': 'false',
    #     'Icp/MaxCorrespondenceDistance': '0.15',
    #     'Icp/VoxelSize': '0.05',

    #     # Graph memory tuning
    #     'Mem/STMSize': '30',
    #     'RGBD/LocalRadius': '5'
    # }

    remappings = [
        ('rgb/image',       '/color/image_raw'),
        ('depth/image',     '/depth/image_raw'),
        ('rgb/camera_info', '/color/camera_info'),
        ('scan',            '/scan_filtered'),
        ('odom',            '/diffbot_base_controller/odom')
    ]

    return LaunchDescription([
        DeclareLaunchArgument('rtabmap_viz',  default_value='true'),
        DeclareLaunchArgument('rviz',         default_value='true'),
        DeclareLaunchArgument('localization', default_value='false'),
        SetParameter(name='use_sim_time', value=True),

        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[parameters,
                        {'rgb_image_transport': 'compressed',
                         'depth_image_transport': 'compressedDepth'}],
            remappings=remappings),

        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']),

        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters,
                        {'Mem/IncrementalMemory': 'False',
                         'Mem/InitWMWithAllNodes': 'True'}],
            remappings=remappings),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            condition=IfCondition(LaunchConfiguration("rtabmap_viz")),
            parameters=[parameters],
            remappings=remappings),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=['-d', os.path.join(
                get_package_share_directory('sucky_rtabmap'),
                'rviz',
                'rtabmap_demo.rviz'
            )],
        ),
    ])

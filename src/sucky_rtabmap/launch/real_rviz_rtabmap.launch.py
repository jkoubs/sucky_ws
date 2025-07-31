from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config_rviz = LaunchConfiguration('rviz_cfg')

    parameters={
          'use_sim_time':'false',
          'frame_id':'base_footprint',
          'odom_frame_id':'odom',
          'odom_tf_linear_variance':0.001, # set to low to trust odometry from wheel encoders
          'odom_tf_angular_variance':0.001,
          'subscribe_rgbd':True,
          'subscribe_scan':True,
          'approx_sync':True,
          'sync_queue_size': 30,
          'RGBD/NeighborLinkRefining': 'true',
          'RGBD/ProximityBySpace':     'true',
          'RGBD/ProximityByTime':      'false',
          'RGBD/ProximityPathMaxNeighbors': '10',
          'Reg/Strategy':              '1',
          'Vis/MinInliers':            '12',
          'RGBD/OptimizeFromGraphEnd': 'false',
          'RGBD/OptimizeMaxError':     '4',
          'Reg/Force3DoF':             'true',
          'Mem/STMSize':               '30',
          'RGBD/LocalRadius':          '5',
          'Icp/CorrespondenceRatio':   '0.2',
          'Icp/PM':                    'false',
          'Icp/PointToPlane':          'false',
          'Icp/MaxCorrespondenceDistance': '0.15',
          'Icp/VoxelSize':             '0.05',

          # 2D Grid Map 
          'Grid/Sensor': '2',                        # Use both scan and depth/point cloud
          'Grid/NormalsSegmentation': 'true',        # Enable surface normal filtering
          'Grid/FlatObstacleDetected': 'true',       # Keep tables, benches, etc.
          'Grid/MaxGroundHeight': '0.1',             # Anything below 15cm = ground
          'Grid/MaxObstacleHeight': '2.00',           # Filter out ceilings and tall objects
          'Grid/RayTracing': 'false',                 # Simulate free space
          'Grid/RangeMin': '0.3',                    # Ignore very close points
          'Grid/RangeMax': '5.0',                    # Sensor max range
          'Grid/MapFrameProjection': 'true',         # Project obstacles in map frame
          'Grid/NormalK': '30',                      # More stable normals
    }

    remappings = [
        ('rgb/image', '/camera/d455/color/image_raw'),
        ('depth/image', '/camera/d455/depth/image_rect_raw'),
        ('rgb/camera_info', '/camera/d455/color/camera_info'),
        ('scan', '/scan')
    ]

    default_rviz_config = os.path.join(
        get_package_share_directory('sucky_rtabmap'), 'rviz', 'sucky_camera.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz_cfg', default_value=default_rviz_config, description='RViz config file'),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[parameters],
            remappings=remappings),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', config_rviz]),
    ])

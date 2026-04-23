import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # --- Package paths ---
    pkg_share = FindPackageShare(package='rtabmap_diff_drive')

    # --- Launch arguments ---
    use_sim_time     = LaunchConfiguration('use_sim_time', default='true')
    qos              = LaunchConfiguration('qos',          default='2')
    rviz_config_path = LaunchConfiguration('rviz_config',
                           default=PathJoinSubstitution([pkg_share, 'config', 'rtabmap_stereo.rviz']))

    # --- RTAB-Map parameters ---
    rtabmap_params = {
        # TODO: enable stereo subscription (set subscribe_stereo, subscribe_depth, subscribe_rgb)
        'subscribe_stereo': True,
        'subscribe_depth':  False,
        'subscribe_rgb':    False,

        # TODO: synchronisation
        'approx_sync':        True,
        'sync_queue_size':    20,

        # Frame IDs — fill in the correct frame names
        'frame_id':      'base_link',   # robot base frame
        'map_frame_id':  'map',         # global map frame
        'odom_frame_id': 'odom',        # odometry frame

        'wait_for_transform': 0.2,
        'use_sim_time':       use_sim_time,

        # QoS
        'qos_image': qos,
        'qos_imu':   qos,
        'qos_odom':  qos,

        # TODO: RTAB-Map strategy parameters
        # Hint: use vision registration (Reg/Strategy='0'), enforce planar motion
        'Reg/Strategy':    '0',
        'Reg/Force3DoF':   'True',
        'Vis/MinInliers':  '10',

        # Occupancy grid
        'Grid/FromDepth':        True,
        'Grid/RangeMax':         '5.0',
        'Grid/MaxObstacleHeight':'2.0',

        'publish_tf_map': True,
    }

    # --- Stereo topic remappings ---
    # Map RTAB-Map's expected topic names → topics published by the Gazebo camera plugins
    stereo_remappings = [
        ('left/image_rect',  '/stereo/left/image_raw'),
        ('right/image_rect', '/stereo/right/image_raw'),
        ('left/camera_info', '/stereo/left/camera_info'),
        ('right/camera_info','/stereo/right/camera_info'),
        ('odom',             '/odom'),
        ('imu',              '/imu/data'),
    ]

    # --- Stereo Odometry node ---
    stereo_odometry_node = Node(
        package='rtabmap_odom',
        executable='stereo_odometry',
        name='stereo_odometry',
        output='screen',
        parameters=[{
            'subscribe_depth': False,
            'approx_sync': True,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'Reg/Force3DoF': 'True',
            'Vis/MinInliers': '10',
            'use_sim_time': use_sim_time,
            'qos_image': qos,
        }],
        remappings=stereo_remappings,
    )

    # --- RTAB-Map core node ---
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_params],
        remappings=stereo_remappings,
        arguments=['-d'],   # delete previous database on start
    )

    # --- RTAB-Map visualisation node ---
    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'stereo': True,
            'subscribe_depth': False,
            'subscribe_stereo': True,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'use_sim_time': use_sim_time,
            'qos_image': qos,
            'qos_odom': qos,
        }],
        remappings=stereo_remappings,
    )

    # --- RViz node ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # --- Launch Description ---
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true',
                                        description='Use simulation clock'))
    ld.add_action(DeclareLaunchArgument('qos', default_value='2',
                                        description='QoS profile (0=default,1=sensor,2=reliable)'))
    ld.add_action(DeclareLaunchArgument('rviz_config',
                                        default_value=PathJoinSubstitution(
                                            [pkg_share, 'config', 'rtabmap_stereo.rviz']),
                                        description='RViz config file'))

    # Add nodes (order matters)
    ld.add_action(stereo_odometry_node)  # Start odometry first
    ld.add_action(rtabmap_node)          # Then RTAB-Map core
    ld.add_action(rtabmap_viz_node)      # Then visualization
    ld.add_action(rviz_node)             # Finally RViz

    return ld

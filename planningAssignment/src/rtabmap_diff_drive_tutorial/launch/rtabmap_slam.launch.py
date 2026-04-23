import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare(package='rtabmap_diff_drive_tutorial')
    default_rviz_config_path = PathJoinSubstitution([pkg_share, 'config', 'rtabmap.rviz'])

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    qos = LaunchConfiguration('qos', default='2')
    rviz_config_path = LaunchConfiguration('rviz_config', default=default_rviz_config_path)
    use_rtabmap_viz = LaunchConfiguration('use_rtabmap_viz', default='false')

    rtabmap_params = {
        'Mem/IncrementalMemory': 'True',
        'Mem/InitWMWithAllNodes': 'False',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_scan': True,
        'subscribe_odom_info': False,
        'frame_id': 'base_link',
        'map_frame_id': 'map',
        'odom_frame_id': 'odom',
        'approx_sync': True,
        'wait_for_transform': 0.2,
        'use_sim_time': use_sim_time,
        'qos_image': qos,
        'qos_imu': qos,
        'qos_odom': qos,
        'qos_scan': qos,
        'Grid/FromDepth': 'False',
        'Grid/FromScan': 'True',
        'Grid/RangeMax': '10.0',
        'RGBD/ProximityBySpace': 'False',
        'RGBD/LoopClosureReextractFeatures': 'False',
        'Reg/Strategy': '0',
        'Reg/Force3DoF': 'True',
        'Vis/MinInliers': '12',
        'queue_size': 30,
        'publish_tf_map': True,
        'database_path': LaunchConfiguration('database_path'),
    }

    rtabmap_remaps = [
        ('odom', '/odom'),
        ('imu', '/imu/data'),
        ('rgb/image', '/camera/image_raw'),
        ('rgb/camera_info', '/camera/camera_info'),
        ('depth/image', '/camera/depth/image_raw'),
        ('depth/camera_info', '/camera/depth/camera_info'),
        ('scan', '/scan'),
    ]

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_params],
        remappings=rtabmap_remaps,
    )

    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        condition=IfCondition(use_rtabmap_viz),
        parameters=[{
            'subscribe_scan': True,
            'qos_scan': qos,
            'subscribe_odom_info': True,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'use_sim_time': use_sim_time,
            'qos_image': qos,
            'qos_imu': qos,
            'qos_odom': qos,
            'queue_size': 30,
        }],
        remappings=rtabmap_remaps,
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock?'))
    ld.add_action(DeclareLaunchArgument('qos', default_value='2', description='QoS profile: 0=system default, 1=sensor data, 2=reliable'))
    ld.add_action(DeclareLaunchArgument('rviz_config', default_value=default_rviz_config_path, description='Full path to the RVIZ config file to use'))
    ld.add_action(DeclareLaunchArgument('use_rtabmap_viz', default_value='false', description='Launch RTAB-Map Qt visualization window'))
    ld.add_action(DeclareLaunchArgument('database_path', default_value='maps/current_world.db', description='Path to the RTAB-Map database file to create/use'))
    ld.add_action(rtabmap_node)
    ld.add_action(rtabmap_viz_node)
    ld.add_action(rviz_node)

    return ld
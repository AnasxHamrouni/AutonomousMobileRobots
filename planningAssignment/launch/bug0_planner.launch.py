import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch file for Bug 0 planner testing"""

    # Get package paths
    rtabmap_pkg = FindPackageShare(package='rtabmap_diff_drive_tutorial')
    rviz_config_path = PathJoinSubstitution([rtabmap_pkg, 'config', 'rtabmap.rviz'])

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    database_path = LaunchConfiguration('database_path', default='~/.ros/rtabmap.db')

    # Bug 0 Planner Node
    bug0_planner = Node(
        package='bug_navigation',
        executable='bug0_planner_node',
        name='bug0_planner',
        output='screen',
        parameters=[{
            'wall_follow_dist': 0.4,
            'obstacle_threshold': 0.3,
            'goal_tolerance': 0.2,
            'linear_speed': 0.2,
            'angular_speed': 0.5,
            'control_loop_freq': 10.0,
            'target_frame': 'odom',
            'use_sim_time': use_sim_time,
        }]
    )

    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock'))
    ld.add_action(DeclareLaunchArgument(
        'database_path', default_value='~/.ros/rtabmap.db',
        description='Path to RTAB-Map database'))

    # Add nodes
    ld.add_action(bug0_planner)

    return ld

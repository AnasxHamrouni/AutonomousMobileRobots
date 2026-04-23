import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():

    # --- Package paths ---
    pkg_share     = FindPackageShare(package='rtabmap_diff_drive')
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros')

    # --- Launch arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    default_world_path = PathJoinSubstitution([pkg_share, 'worlds', 'test.world'])
    world_path = LaunchConfiguration('world', default=default_world_path)

    # --- Robot description (xacro → URDF string) ---
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'stereo_robot.urdf.xacro'])
    robot_description_raw = Command(['xacro ', xacro_file])

    # --- TODO: Robot State Publisher node ---
    # Hint: package='robot_state_publisher', executable='robot_state_publisher'
    # Parameters needed: robot_description, use_sim_time
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': use_sim_time,
        }],
    )

    # --- TODO: Joint State Publisher node ---
    # Hint: package='joint_state_publisher', executable='joint_state_publisher'
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # --- TODO: Gazebo server (loads the world) ---
    # Hint: use IncludeLaunchDescription with gzserver.launch.py from pkg_gazebo_ros
    # Pass world_path as a launch argument
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gzserver.launch.py'])
        ),
        launch_arguments={'world': world_path}.items(),
    )

    # --- TODO: Gazebo client (opens the GUI) ---
    # Hint: use IncludeLaunchDescription with gzclient.launch.py from pkg_gazebo_ros
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gzclient.launch.py'])
        ),
    )

    # --- TODO: Spawn Entity node ---
    # Hint: package='gazebo_ros', executable='spawn_entity.py'
    # Spawn from the /robot_description topic, name the entity 'stereo_diff_drive_robot'
    # Initial position: x=0, y=0, z=0.1
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'stereo_diff_drive_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
        ],
    )

    # --- Launch Description ---
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true',
                                        description='Use simulation (Gazebo) clock?'))
    ld.add_action(DeclareLaunchArgument('world', default_value=default_world_path,
                                        description='Full path to world file to load'))

    # TODO: add all nodes/launch includes to ld in the correct order
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_joint_state_publisher)
    ld.add_action(spawn_entity_node)

    return ld

#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Get the package directories
    my_robot_description = get_package_share_directory('my_robot_description')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    # Define file paths
    urdf_path = os.path.join(my_robot_description, 'urdf', 'my_robot.urdf.xacro')
    rviz_config_path = os.path.join(my_robot_description, 'rviz', 'urdf_config.rviz')
    world_path = os.path.join(my_robot_description, 'worlds', 'test.world')

    # robot_state_publisher node with xacro command substitution
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
        }]
    )

    # Node for Joint State Publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    # Include Gazebo's launch file and pass the world file as an argument
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_path}.items()
    )

    # Launch RViz2
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )

    # Spawn the robot entity in Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo_launch,
        rviz2_node,
        spawn_entity_node,
    ])


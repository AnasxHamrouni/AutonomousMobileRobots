#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, ExecuteProcess,
                            RegisterEventHandler)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get the package directories
    exam_pkg = get_package_share_directory('exam')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    # Define file paths
    urdf_path = os.path.join(exam_pkg, 'urdf', 'ackermann_robot.urdf.xacro')
    world_path = os.path.join(exam_pkg, 'worlds', 'exam_world.world')

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

    # Include Gazebo's launch file and pass the world file as an argument
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_path}.items()
    )

    # Spawn the robot entity in Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-topic', 'robot_description', '-entity', 'ackermann_robot',
                   '-x', '0', '-y', '0', '-z', '0.05'],
        parameters=[{'use_sim_time': True}]
    )

    # ros2_control: load controllers after robot is spawned
    joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'active', 'joint_state_broadcaster'],
        output='screen')

    forward_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'active', 'forward_velocity_controller'],
        output='screen')

    forward_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'active', 'forward_position_controller'],
        output='screen')

    # Ackermann drive node (cmd_vel -> joint commands + odom)
    ackermann_drive_node = Node(
        package='exam',
        executable='ackermann_drive_node',
        name='ackermann_drive_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Obstacle avoidance node (/cmd_vel_nav + /scan -> /cmd_vel)
    obstacle_avoidance_node = Node(
        package='exam',
        executable='obstacle_avoidance',
        name='obstacle_avoidance',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        # First: start Gazebo, robot_state_publisher, spawn robot
        robot_state_publisher_node,
        gazebo_launch,
        spawn_entity_node,

        # After spawn completes: load joint_state_broadcaster
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity_node,
                on_exit=[joint_state_broadcaster])),

        # After joint_state_broadcaster: load velocity and position controllers
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[forward_velocity_controller,
                         forward_position_controller])),

        # Ackermann drive node + obstacle avoidance safety layer
        ackermann_drive_node,
        # obstacle_avoidance_node,
    ])

#!/usr/bin/env python3

import math
from abc import ABC, abstractmethod

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf_transformations import euler_from_quaternion


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class WaypointControllerBase(Node, ABC):
    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.declare_parameter('waypoint_tolerance', 0.20)
        self.declare_parameter('control_period', 0.1)
        self.declare_parameter('max_linear_velocity', 0.5)
        self.declare_parameter('max_angular_velocity', 1.5)
        self.declare_parameter('waypoints_x', [3.0, 6.0, 3.0, 3.0, 0.0])
        self.declare_parameter('waypoints_y', [0.0, 4.0, 4.0, 1.0, 3.0])

        self.waypoint_tolerance = float(self.get_parameter('waypoint_tolerance').value)
        self.control_period = float(self.get_parameter('control_period').value)
        self.max_linear_velocity = float(self.get_parameter('max_linear_velocity').value)
        self.max_angular_velocity = float(self.get_parameter('max_angular_velocity').value)

        waypoints_x = self.get_parameter('waypoints_x').value
        waypoints_y = self.get_parameter('waypoints_y').value

        if len(waypoints_x) != len(waypoints_y) or len(waypoints_x) == 0:
            self.get_logger().warning('Invalid waypoint lists, using defaults.')
            waypoints_x = [3.0, 6.0, 3.0, 3.0, 0.0]
            waypoints_y = [0.0, 4.0, 4.0, 1.0, 3.0]

        self.waypoints = np.column_stack((np.array(waypoints_x, dtype=float), np.array(waypoints_y, dtype=float)))

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.has_odom = False
        self.current_waypoint_idx = 0
        self.goal_reached = False

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.control_timer = self.create_timer(self.control_period, self.control_loop)

        self.get_logger().info(f'{node_name} started with {len(self.waypoints)} waypoints')

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        orientation = msg.pose.pose.orientation
        _, _, self.robot_yaw = euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        ])
        self.has_odom = True

    def current_target(self) -> np.ndarray:
        return self.waypoints[self.current_waypoint_idx]

    def distance_to_target(self, target: np.ndarray) -> float:
        return math.hypot(target[0] - self.robot_x, target[1] - self.robot_y)

    def heading_to_target(self, target: np.ndarray) -> float:
        return math.atan2(target[1] - self.robot_y, target[0] - self.robot_x)

    def advance_waypoint_if_reached(self):
        if self.goal_reached:
            return

        target = self.current_target()
        if self.distance_to_target(target) <= self.waypoint_tolerance:
            self.get_logger().info(
                f'Reached waypoint {self.current_waypoint_idx + 1}: ({target[0]:.2f}, {target[1]:.2f})'
            )
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx >= len(self.waypoints):
                self.current_waypoint_idx = len(self.waypoints) - 1
                self.goal_reached = True
                self.get_logger().info('All waypoints reached. Stopping robot.')

    def publish_twist(self, linear_velocity: float, angular_velocity: float):
        cmd = Twist()
        cmd.linear.x = float(max(-self.max_linear_velocity, min(self.max_linear_velocity, linear_velocity)))
        cmd.angular.z = float(max(-self.max_angular_velocity, min(self.max_angular_velocity, angular_velocity)))
        self.cmd_vel_publisher.publish(cmd)

    def stop_robot(self):
        self.publish_twist(0.0, 0.0)

    def control_loop(self):
        if not self.has_odom:
            return

        if self.goal_reached:
            self.stop_robot()
            return

        self.advance_waypoint_if_reached()
        if self.goal_reached:
            self.stop_robot()
            return

        target = self.current_target()
        distance = self.distance_to_target(target)

        linear_velocity, angular_velocity = self.compute_control(target, distance)
        self.publish_twist(linear_velocity, angular_velocity)

    @abstractmethod
    def compute_control(self, target: np.ndarray, distance_to_target: float) -> tuple[float, float]:
        raise NotImplementedError


def spin_controller(node: Node):
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

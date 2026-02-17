#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np
from tf_transformations import euler_from_quaternion

class WaypointController(Node):
    def __init__(self):
        super().__init__('waypoint_controller')
        
        # Declare and get parameters
        self.declare_parameter('linear_velocity', 0.3)
        self.declare_parameter('lookahead_distance', 0.5)
        self.declare_parameter('waypoint_tolerance', 0.1)
        
        self.linear_velocity = self.get_parameter('linear_velocity').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        
        # Define waypoints
        self.waypoints = np.array([
            [3, 0],
            [6, 4],
            [3, 4],
            [3, 1],
            [0, 3]
        ])
        
        self.current_waypoint_idx = 0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        # Data collection for plotting
        self.time_data = []
        self.x_data = []
        self.y_data = []
        self.yaw_data = []
        self.start_time = None
        
        # Publishers and Subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Control loop timer
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.goal_reached = False
        
        self.get_logger().info(f"Waypoint Controller started with {len(self.waypoints)} waypoints")
        self.get_logger().info(f"Linear velocity: {self.linear_velocity} m/s")
        self.get_logger().info(f"Lookahead distance: {self.lookahead_distance} m")

    def odom_callback(self, msg):
        """Callback to update robot's current position and orientation."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Convert quaternion to Euler angles
        orientation = msg.pose.pose.orientation
        _, _, self.robot_yaw = euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ])
        
        # Collect data for visualization
        if self.start_time is None:
            self.start_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        elapsed_time = current_time - self.start_time
        
        self.time_data.append(elapsed_time)
        self.x_data.append(self.robot_x)
        self.y_data.append(self.robot_y)
        self.yaw_data.append(self.robot_yaw)

    def pure_pursuit_control(self):
        """
        Pure Pursuit Controller Implementation
        
        The controller calculates steering commands based on:
        1. Cross-track error (perpendicular distance to path)
        2. Heading error (difference between robot heading and desired heading)
        """
        target = self.waypoints[self.current_waypoint_idx]
        
        # Calculate distance to target waypoint
        dx = target[0] - self.robot_x
        dy = target[1] - self.robot_y
        distance_to_target = math.sqrt(dx**2 + dy**2)
        
        # Check if waypoint is reached
        if distance_to_target < self.waypoint_tolerance:
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_idx + 1} at ({target[0]}, {target[1]})")
            self.current_waypoint_idx += 1
            
            if self.current_waypoint_idx >= len(self.waypoints):
                self.goal_reached = True
                self.get_logger().info("All waypoints reached! Stopping robot.")
                return 0.0, 0.0  # Stop the robot
        
        # Calculate desired heading (angle to next waypoint)
        desired_heading = math.atan2(dy, dx)
        
        # Calculate heading error
        heading_error = desired_heading - self.robot_yaw
        
        # Normalize heading error to [-pi, pi]
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi
        
        # Pure Pursuit Law: Calculate angular velocity
        # Maximum curvature is limited by the lookahead distance
        # omega = 2 * sin(alpha) / L, where alpha is the heading error and L is lookahead distance
        alpha = heading_error
        
        # Limit the maximum angular velocity
        max_angular_velocity = 1.0  # rad/s
        
        if self.lookahead_distance > 0:
            angular_velocity = (2.0 / self.lookahead_distance) * math.sin(alpha)
            angular_velocity = max(-max_angular_velocity, min(max_angular_velocity, angular_velocity))
        else:
            angular_velocity = 0.0
        
        # Use linear velocity proportional to distance and alignment
        # Reduce speed if heading is misaligned
        linear_velocity = self.linear_velocity * math.cos(heading_error)
        linear_velocity = max(0.0, linear_velocity)  # Don't go backwards
        
        return linear_velocity, angular_velocity

    def control_loop(self):
        """Main control loop."""
        if self.goal_reached:
            # Publish zero velocity to stop robot
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist)
            return
        
        linear_vel, angular_vel = self.pure_pursuit_control()
        
        # Create and publish Twist message
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        
        self.cmd_vel_publisher.publish(twist)
        
        # Log current state
        target = self.waypoints[self.current_waypoint_idx]
        distance = math.sqrt((target[0] - self.robot_x)**2 + (target[1] - self.robot_y)**2)
        
        self.get_logger().debug(
            f"Robot: ({self.robot_x:.2f}, {self.robot_y:.2f}, {math.degrees(self.robot_yaw):.1f}°) | "
            f"Target: ({target[0]:.1f}, {target[1]:.1f}) | "
            f"Distance: {distance:.2f} | "
            f"Vel: ({linear_vel:.2f}, {angular_vel:.2f})"
        )

    def save_data(self):
        """Save collected data to file for visualization."""
        import json
        
        data = {
            'time': self.time_data,
            'x': self.x_data,
            'y': self.y_data,
            'yaw': self.yaw_data,
            'waypoints': self.waypoints.tolist()
        }
        
        with open('/tmp/robot_trajectory.json', 'w') as f:
            json.dump(data, f, indent=2)
        
        self.get_logger().info("Data saved to /tmp/robot_trajectory.json")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_data()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

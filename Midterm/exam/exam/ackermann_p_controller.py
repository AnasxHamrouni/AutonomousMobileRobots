"""
Ackermann P Controller - EXAM STARTER TEMPLATE
===============================================
Proportional controller for waypoint following with Ackermann steering.

  - The Ackermann plugin interprets cmd_vel.angular.z as the steering angle
  - The robot cannot rotate in place, so we reduce speed during sharp turns

Complete the TODOs below to make the robot follow the waypoints.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math

WAYPOINTS = np.array([[3, 0], [6, 4], [3, 4], [0, 0]])
WHEELBASE = 0.5       # Distance between front and rear axles (meters)
MAX_STEER = 0.6       # Max steering angle in radians (~35 degrees)
MAX_SPEED = 1.0       # Max forward speed in m/s
MIN_SPEED = 0.3       # Min speed — Ackermann needs forward motion to steer


def yaw_from_quaternion(q):
    """Extract yaw angle from a quaternion (x, y, z, w)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class AckermannPController(Node):
    def __init__(self):
        super().__init__('ackermann_p_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.kp_speed = 0.8
        self.kp_steer = 2.0
        self.current_pose = [0.0, 0.0, 0.0]  # [x, y, yaw]
        self.current_waypoint_index = 0

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(orientation)
        self.current_pose = [position.x, position.y, yaw]
        self.control_loop()

    def control_loop(self):
        if self.current_waypoint_index >= len(WAYPOINTS):
            self.stop_robot()
            return

        goal = WAYPOINTS[self.current_waypoint_index]
        distance = np.linalg.norm(
            [goal[0] - self.current_pose[0], goal[1] - self.current_pose[1]])

        if distance < 0.3:
            self.current_waypoint_index += 1
            self.get_logger().info(
                f'Reached waypoint {self.current_waypoint_index}')
            return

        # ============================================================
        # TODO 1: Compute the angle from the robot to the goal

        angle_to_goal = math.atan2(
            goal[1] - self.current_pose[1],
            goal[0] - self.current_pose[0]
        )

        # ============================================================
        # TODO 2: Compute the heading error
   
        heading_error = math.atan2(
            math.sin(angle_to_goal - self.current_pose[2]),
            math.cos(angle_to_goal - self.current_pose[2])
        )

        # ============================================================
        # TODO 3: Compute the steering angle using proportional control

        steering_angle = self.kp_steer * heading_error
        steering_angle = float(np.clip(steering_angle, -MAX_STEER, MAX_STEER))

        # ============================================================
        # TODO 4: Compute the forward speed

        turn_factor = 1.0 - 0.5 * (abs(steering_angle) / MAX_STEER)
        speed = self.kp_speed * distance * turn_factor
        speed = float(np.clip(speed, MIN_SPEED, MAX_SPEED))

        # ============================================================
        # TODO 5: Create and publish the Twist message
        
        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = steering_angle
        self.publisher_.publish(cmd)

    def stop_robot(self):
        twist = Twist()
        self.publisher_.publish(twist)
        self.get_logger().info("All waypoints reached!")


def main():
    rclpy.init()
    node = AckermannPController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

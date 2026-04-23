"""
Ackermann Stanley Controller - EXAM STARTER TEMPLATE
=====================================================
Stanley path tracking controller for Ackermann steering.


Where:
  psi_e = heading error          (robot yaw - path heading, normalized to [-pi, pi])
  e     = cross-track error      (signed, left-positive)
  k     = Stanley gain
  k_s   = softening constant     (prevents division by zero at low speed)
  v     = current forward speed

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
K_STANLEY = 1.0       # Stanley gain (cross-track error correction strength)
K_SOFT = 0.5          # Softening constant (prevents division by zero at low speed)
WAYPOINT_REACHED_DIST = 0.45


def normalize_angle(angle):
    """Wrap angle to the range [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q):
    """Extract yaw angle from a quaternion (x, y, z, w)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class AckermannStanley(Node):
    def __init__(self):
        super().__init__('ackermann_stanley')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.current_pose = [0.0, 0.0, 0.0]  # [x, y, yaw]
        self.current_waypoint_index = 0
        # Track the previous waypoint (start of current path segment)
        self.prev_waypoint = np.array([0.0, 0.0])

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
        rx, ry, yaw = self.current_pose
        distance = np.linalg.norm([goal[0] - rx, goal[1] - ry])

        if distance < WAYPOINT_REACHED_DIST:
            self.prev_waypoint = goal.copy()
            self.current_waypoint_index += 1
            self.get_logger().info(
                f'Reached waypoint {self.current_waypoint_index}')
            return

        # ============================================================
        # TODO 1: Compute the path segment heading

        segment = goal - self.prev_waypoint
        segment_norm = np.linalg.norm(segment)
        if segment_norm < 1e-6:
            path_heading = yaw
        else:
            path_heading = math.atan2(segment[1], segment[0])

        # ============================================================
        # TODO 2: Compute the heading error

        heading_error = normalize_angle(path_heading - yaw)

        # ============================================================
        # TODO 3: Compute the cross-track error (CTE)

        if segment_norm < 1e-6:
            cte = 0.0
        else:
            rel = np.array([rx, ry]) - self.prev_waypoint
            cte = (segment[0] * rel[1] - segment[1] * rel[0]) / segment_norm

        # ============================================================
        # TODO 4: Compute the Stanley steering angle

        heading_ratio = min(1.0, abs(heading_error) / (math.pi / 2.0))
        nominal_speed = MAX_SPEED * (1.0 - 0.55 * heading_ratio)
        nominal_speed = float(np.clip(nominal_speed, MIN_SPEED, MAX_SPEED))

        cte_term = math.atan2(K_STANLEY * cte, K_SOFT + nominal_speed)
        steering_angle = heading_error + cte_term
        steering_angle = float(np.clip(steering_angle, -MAX_STEER, MAX_STEER))

        # ============================================================
        # TODO 5: Compute the forward speed

        turn_factor = 1.0 - 0.7 * (abs(steering_angle) / MAX_STEER)
        speed = nominal_speed * turn_factor
        speed = float(np.clip(speed, MIN_SPEED, MAX_SPEED))

        # ============================================================
        # TODO 6: Create and publish the Twist message

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
    node = AckermannStanley()
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

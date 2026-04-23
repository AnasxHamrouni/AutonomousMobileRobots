"""
Ackermann Pure Pursuit Controller - EXAM STARTER TEMPLATE
==========================================================
Pure Pursuit path tracking for Ackermann steering.

Where:
  L     = wheelbase (distance between front and rear axles)
  alpha = angle between robot heading and the lookahead point
  Ld    = lookahead distance

Complete the TODOs below to make the robot follow the waypoints.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math

WAYPOINTS = np.array([[3, 0], [6, 4], [3, 4], [0, 0]])
WHEELBASE = 0.5              # Distance between front and rear axles (meters)
MAX_STEER = 0.6              # Max steering angle in radians
MAX_SPEED = 1.0              # Max forward speed in m/s
LOOKAHEAD_DISTANCE = 1.0     # Lookahead distance for pure pursuit
MIN_SPEED = 0.25             # Keep forward motion for Ackermann steering
WAYPOINT_REACHED_DIST = 0.45 # Slightly larger capture radius for cornering


def yaw_from_quaternion(q):
    """Extract yaw angle from a quaternion (x, y, z, w)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class AckermannPurePursuit(Node):
    def __init__(self):
        super().__init__('ackermann_pure_pursuit')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.current_pose = [0.0, 0.0, 0.0]  # [x, y, yaw]
        self.current_waypoint_index = 0
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
        robot_xy = np.array([rx, ry])

        segment = goal - self.prev_waypoint
        segment_len = np.linalg.norm(segment)
        distance = np.linalg.norm(goal - robot_xy)

        if segment_len < 1e-6:
            passed_waypoint = False
            lookahead_point = goal
        else:
            rel = robot_xy - self.prev_waypoint
            s = np.dot(rel, segment) / segment_len
            passed_waypoint = s >= segment_len
            lookahead_s = float(np.clip(s + LOOKAHEAD_DISTANCE, 0.0, segment_len))
            lookahead_point = self.prev_waypoint + (lookahead_s / segment_len) * segment

        if distance < WAYPOINT_REACHED_DIST or passed_waypoint:
            self.prev_waypoint = goal.copy()
            self.current_waypoint_index += 1
            self.get_logger().info(
                f'Reached waypoint {self.current_waypoint_index}')
            return

        # ============================================================
        # TODO 1: Compute the angle from the robot to the goal
        
        angle_to_goal = math.atan2(
            lookahead_point[1] - ry,
            lookahead_point[0] - rx
        )

        # ============================================================
        # TODO 2: Compute alpha - the angle between the robot's heading
        #         and the line to the goal point.
       
        alpha = math.atan2(
            math.sin(angle_to_goal - yaw),
            math.cos(angle_to_goal - yaw)
        )

        # ============================================================
        # TODO 3: Compute the effective lookahead distance
      
        ld = max(0.35, np.linalg.norm(lookahead_point - robot_xy))

        # ============================================================
        # TODO 4: Compute the steering angle using the Pure Pursuit
        
        steering_angle = math.atan2(2.0 * WHEELBASE * math.sin(alpha), ld)
        steering_angle = float(np.clip(steering_angle, -MAX_STEER, MAX_STEER))

        # ============================================================
        # TODO 5: Compute the speed
        
        heading_ratio = min(1.0, abs(alpha) / (math.pi / 2.0))
        speed = MAX_SPEED * (1.0 - 0.7 * heading_ratio)
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
    node = AckermannPurePursuit()
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

"""
Obstacle Avoidance Node - EXAM STARTER TEMPLATE
================================================
A reactive safety layer that sits between the path controller and the drive node.

Data flow:
  Path controller  →  /cmd_vel_nav  →  [this node]  →  /cmd_vel  →  ackermann_drive_node

Behavior:
  - When no obstacle is detected ahead: pass /cmd_vel_nav through unchanged.
  - When an obstacle is detected within OBSTACLE_THRESHOLD: override the
    steering command to steer away from the obstacle and reduce speed.

Complete the TODOs below to implement reactive obstacle avoidance.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

OBSTACLE_THRESHOLD = 1.0   # Distance (m) at which avoidance activates
AVOIDANCE_SPEED = 0.3      # Forward speed during avoidance (m/s)
MAX_STEER = 0.6            # Max steering angle in radians


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        # Publisher: final velocity commands to the drive node
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber: desired commands from the path controller
        self.nav_sub = self.create_subscription(
            Twist, '/cmd_vel_nav', self.nav_callback, 10)

        # Subscriber: LiDAR scan
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Store the latest navigation command and scan data
        self.latest_nav_cmd = Twist()
        self.latest_scan = None

        self.get_logger().info('Obstacle avoidance node started')

    def scan_callback(self, msg):
        self.latest_scan = msg

    def nav_callback(self, msg):
        """Called each time the path controller sends a new command."""
        self.latest_nav_cmd = msg

        # If we haven't received a scan yet, pass through unchanged
        if self.latest_scan is None:
            self.cmd_pub.publish(msg)
            return

        self.process_and_publish()

    def process_and_publish(self):
        ranges = self.latest_scan.ranges
        n = len(ranges)

        # ============================================================
        # TODO 1: Parse the scan into two front sectors.

        #   For the ±45° front cone (assuming n == 360):
        #   Verified Gazebo mapping:
        #     front_right : indices 136 to 179  (right of forward)
        #     front_left  : indices 180 to 224  (left of forward)

        front_right_ranges = ranges[136:180]
        front_left_ranges = ranges[180:225]

        # ============================================================
        # TODO 2: Compute the minimum valid distance in each sector.
        #   Filter out values that are inf, nan, or <= 0 before taking min.
        #   If a sector has no valid readings, treat its minimum as infinity.

        valid_left = [r for r in front_left_ranges if math.isfinite(r) and r > 0.0]
        valid_right = [r for r in front_right_ranges if math.isfinite(r) and r > 0.0]

        min_left = min(valid_left) if valid_left else math.inf
        min_right = min(valid_right) if valid_right else math.inf
        min_front = min(min_left, min_right)

        # ============================================================
        # TODO 3: Decide whether an obstacle requires avoidance.
        #   An obstacle is present if min_front < OBSTACLE_THRESHOLD.

        obstacle_detected = min_front < OBSTACLE_THRESHOLD

        # ============================================================
        # TODO 4: Compute the avoidance command.


        avoidance_cmd = Twist()
        avoidance_cmd.linear.x = AVOIDANCE_SPEED
        if min_left < min_right:
            avoidance_cmd.angular.z = -MAX_STEER
        else:
            avoidance_cmd.angular.z = MAX_STEER

        # ============================================================
        # TODO 5: Publish the appropriate command.

        if obstacle_detected:
            self.cmd_pub.publish(avoidance_cmd)
        else:
            self.cmd_pub.publish(self.latest_nav_cmd)


def main():
    rclpy.init()
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

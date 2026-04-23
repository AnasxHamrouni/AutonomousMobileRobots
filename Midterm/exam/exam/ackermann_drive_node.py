"""
Ackermann Drive Node
====================
Converts cmd_vel (Twist) to ros2_control joint commands and publishes odometry.

- angular.z is interpreted as the desired steering angle (NOT angular velocity)
- linear.x is the desired forward speed (m/s)

Uses IMU for accurate heading and joint states for wheel velocity feedback.

Publishes:
  /forward_position_controller/commands  (steering joint positions)
  /forward_velocity_controller/commands  (rear wheel angular velocities)
  /odom  (odometry with IMU-corrected heading)
  /tf    (odom -> base_footprint transform)
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster


class AckermannDriveNode(Node):
    def __init__(self):
        super().__init__('ackermann_drive_node')

        # Vehicle parameters
        self.wheelbase = 0.5        # Distance between front and rear axles
        self.track_width = 0.45     # Center-to-center distance between left and right wheels
        self.wheel_radius = 0.1     # Wheel radius
        self.max_steer = 0.6        # Max steering angle (rad)
        self.max_speed = 2.0        # Max linear speed (m/s)

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0            # From IMU
        self.last_time = self.get_clock().now()

        # Current commands
        self.steering_angle = 0.0
        self.velocity = 0.0

        # Actual feedback from sensors
        self.imu_yaw = 0.0
        self.imu_received = False
        self.actual_left_wheel_vel = 0.0
        self.actual_right_wheel_vel = 0.0

        # Publishers
        self.position_pub = self.create_publisher(
            Float64MultiArray, '/forward_position_controller/commands', 10)
        self.velocity_pub = self.create_publisher(
            Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

        # Timer for publishing at fixed rate
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz

        self.get_logger().info('Ackermann drive node started')

    def cmd_vel_callback(self, msg):
        # angular.z = desired steering angle
        self.steering_angle = max(-self.max_steer, min(self.max_steer, msg.angular.z))
        # linear.x = desired forward speed
        self.velocity = max(-self.max_speed, min(self.max_speed, msg.linear.x))

    def imu_callback(self, msg):
        # Extract yaw from IMU quaternion
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.imu_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.imu_received = True

    def joint_state_callback(self, msg):
        # Extract actual rear wheel velocities from joint states
        for i, name in enumerate(msg.name):
            if i < len(msg.velocity):
                vel = msg.velocity[i]
                if not math.isnan(vel):
                    if name == 'rear_left_wheel_joint':
                        self.actual_left_wheel_vel = vel
                    elif name == 'rear_right_wheel_joint':
                        self.actual_right_wheel_vel = vel

    def ackermann_steering_angles(self):
        """Compute inner/outer wheel steering angles using Ackermann geometry."""
        left_angle = 0.0
        right_angle = 0.0

        if abs(self.steering_angle) > 1e-3:
            sin_a = math.sin(abs(self.steering_angle))
            cos_a = math.cos(abs(self.steering_angle))

            if self.steering_angle > 0.0:
                # Turning left
                left_angle = math.atan(
                    (2 * self.wheelbase * sin_a) /
                    (2 * self.wheelbase * cos_a - self.track_width * sin_a))
                right_angle = math.atan(
                    (2 * self.wheelbase * sin_a) /
                    (2 * self.wheelbase * cos_a + self.track_width * sin_a))
            else:
                # Turning right
                left_angle = -math.atan(
                    (2 * self.wheelbase * sin_a) /
                    (2 * self.wheelbase * cos_a + self.track_width * sin_a))
                right_angle = -math.atan(
                    (2 * self.wheelbase * sin_a) /
                    (2 * self.wheelbase * cos_a - self.track_width * sin_a))

        return left_angle, right_angle

    def rear_differential_velocity(self):
        """Compute differential rear wheel velocities during turning."""
        left_vel = self.velocity
        right_vel = self.velocity

        if abs(self.steering_angle) > 1e-3:
            turning_radius = self.wheelbase / math.tan(abs(self.steering_angle))
            angular_velocity = self.velocity / turning_radius

            inner_radius = turning_radius - (self.track_width / 2.0)
            outer_radius = turning_radius + (self.track_width / 2.0)

            if self.steering_angle > 0.0:
                # Turning left
                left_vel = angular_velocity * inner_radius
                right_vel = angular_velocity * outer_radius
            else:
                # Turning right
                left_vel = angular_velocity * outer_radius
                right_vel = angular_velocity * inner_radius

            # Scale if exceeding max
            max_wheel_vel = max(abs(left_vel), abs(right_vel))
            if max_wheel_vel > self.max_speed:
                scale = self.max_speed / max_wheel_vel
                left_vel *= scale
                right_vel *= scale

        return left_vel, right_vel

    def update_odometry(self):
        """Compute odometry using IMU heading and joint state wheel velocities."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0.0 or dt > 1.0:
            return

        # Use IMU for heading (ground truth, no drift)
        if self.imu_received:
            self.theta = self.imu_yaw

        # Use actual wheel velocities for linear speed
        avg_wheel_vel = (self.actual_left_wheel_vel + self.actual_right_wheel_vel) / 2.0
        actual_speed = avg_wheel_vel * self.wheel_radius

        # Integrate position using actual speed and IMU heading
        d_x = actual_speed * math.cos(self.theta) * dt
        d_y = actual_speed * math.sin(self.theta) * dt

        self.x += d_x
        self.y += d_y

    def publish_odometry(self):
        """Publish odom message and tf."""
        now = self.get_clock().now().to_msg()

        # Quaternion from yaw
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        # Compute actual linear velocity from wheel feedback
        avg_wheel_vel = (self.actual_left_wheel_vel + self.actual_right_wheel_vel) / 2.0
        actual_speed = avg_wheel_vel * self.wheel_radius

        # Odom message
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = actual_speed
        if abs(self.steering_angle) > 1e-3:
            odom.twist.twist.angular.z = actual_speed * math.tan(self.steering_angle) / self.wheelbase
        self.odom_pub.publish(odom)

        # TF: odom -> base_footprint
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

    def timer_callback(self):
        # Compute Ackermann steering angles
        left_steer, right_steer = self.ackermann_steering_angles()

        # Compute rear wheel velocities
        left_vel, right_vel = self.rear_differential_velocity()

        # Publish steering positions
        pos_msg = Float64MultiArray()
        pos_msg.data = [left_steer, right_steer]
        self.position_pub.publish(pos_msg)

        # Publish wheel angular velocities
        vel_msg = Float64MultiArray()
        vel_msg.data = [left_vel / self.wheel_radius, right_vel / self.wheel_radius]
        self.velocity_pub.publish(vel_msg)

        # Update and publish odometry
        self.update_odometry()
        self.publish_odometry()


def main():
    rclpy.init()
    node = AckermannDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

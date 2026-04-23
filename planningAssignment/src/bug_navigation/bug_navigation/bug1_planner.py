#!/usr/bin/env python3
"""
Bug 1 Path Planning Algorithm

Task 2 implementation:
- GO_TO_GOAL: Move toward original goal until obstacle hit
- WALL_FOLLOW: Circumnavigate full obstacle boundary and track closest boundary point to goal
- GO_TO_CLOSEST_POINT: After full loop, follow boundary from hit point to closest point
- Resume GO_TO_GOAL from closest boundary point
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from enum import Enum
import math
import os
import csv
from datetime import datetime
from copy import copy

import tf2_ros
import tf2_geometry_msgs  # noqa: F401 Registers PoseStamped transform for tf2
from tf2_ros import TransformException


def euler_from_quaternion(quaternion):
    """Convert a geometry_msgs Quaternion to yaw."""
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z


class Bug1State(Enum):
    GO_TO_GOAL = 1
    WALL_FOLLOW = 2
    GO_TO_CLOSEST_POINT = 3
    GOAL_REACHED = 4


class Bug1PlannerNode(Node):
    """
    Bug 1 Algorithm Node (Task 2)
    Implements the Bug 1 path planning algorithm for a differential drive robot in ROS 2.

    Main states:
    - GO_TO_GOAL: Move directly toward the goal until an obstacle is detected.
    - WALL_FOLLOW: Circumnavigate the obstacle boundary, tracking the closest point to the goal.
    - GO_TO_CLOSEST_POINT: After a full loop, follow the boundary to the closest point found.
    - GOAL_REACHED: Stop when the goal is reached within tolerance.

    Key logic:
    - On obstacle hit, start wall following and record the hit point.
    - During wall following, track the closest point on the boundary to the goal.
    - Detect when a full circumnavigation is complete (return to hit point after leaving it).
    - After a full loop, transition to GO_TO_CLOSEST_POINT and follow the boundary to the closest point.
    - Resume GO_TO_GOAL from the closest point.

    Metrics logging:
    - Logs run statistics (success, time, path length, wall follow entries, circumnavigations, recovery events) to CSV for quantitative comparison.

    Parameters:
    - wall_follow_dist: Desired distance to keep from the wall during wall following.
    - obstacle_threshold: Distance threshold to consider an obstacle detected.
    - goal_tolerance: Distance to goal for success.
    - loop_hit_tolerance: Tolerance for detecting return to hit point.
    - linear_speed, angular_speed: Robot speeds.
    - control_loop_freq: Main control loop frequency.
    - scenario_name: Tag for metrics logging.
    """
    def __init__(self):
        super().__init__('bug1_planner_node')

        # Parameters
        self.declare_parameter('wall_follow_dist', 0.4)
        self.declare_parameter('max_scan_range', 10.0)
        self.declare_parameter('obstacle_threshold', 0.32)
        self.declare_parameter('clearance_margin', 0.03)
        self.declare_parameter('goal_tolerance', 0.25)
        self.declare_parameter('loop_hit_tolerance', 0.30)
        self.declare_parameter('linear_speed', 0.20)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('control_loop_freq', 10.0)
        self.declare_parameter('target_frame', 'odom')
        self.declare_parameter('enable_metrics_logging', True)
        self.declare_parameter('metrics_output_dir', 'metrics')
        self.declare_parameter('scenario_name', 'default')

        # Goal and pose state
        self.goal_pose_stamped = None
        self.goal_internal = None
        self.current_pose = None
        self.current_yaw = 0.0
        self.odom_frame = None
        self.scan_data = None

        # Bug1 state
        self.state = Bug1State.GO_TO_GOAL
        self.obstacle_hit_point = None
        self.left_hit_area_once = False
        self.closest_point_on_boundary = None
        self.min_dist_to_goal = float('inf')
        self.stuck_counter = 0
        self.recovery_steps = 0
        self.corner_cooldown_steps = 0
        self.last_pose_for_stuck = None

        # Visualization
        self.path_msg = Path()

        # Metrics state
        self.metrics_goal_active = False
        self.goal_start_time_ns = 0
        self.goal_start_pose = None
        self.prev_pose_for_path = None
        self.path_length_m = 0.0
        self.wall_follow_entries = 0
        self.recovery_event_count = 0
        self.circumnavigation_count = 0
        self.goal_counter = 0
        self.metrics_path = None

        self._init_metrics_logging()

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ROS I/O
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.path_publisher = self.create_publisher(Path, 'robot_path', 10)

        self.goal_subscriber = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.odom_subscriber = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.scan_subscriber = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, rclpy.qos.qos_profile_sensor_data)

        freq = self.get_parameter('control_loop_freq').get_parameter_value().double_value
        self.timer = self.create_timer(1.0 / freq, self.control_loop)

        self.get_logger().info('Bug 1 Planner Node Initialized.')

    def _init_metrics_logging(self):
        """
        Initialize CSV logging for metrics if enabled.
        Creates the output directory and CSV header if needed.
        """
        if not self.get_parameter('enable_metrics_logging').get_parameter_value().bool_value:
            return

        output_dir = self.get_parameter('metrics_output_dir').get_parameter_value().string_value.strip()
        if not output_dir:
            output_dir = 'metrics'
        if not os.path.isabs(output_dir):
            output_dir = os.path.join(os.getcwd(), output_dir)

        os.makedirs(output_dir, exist_ok=True)
        self.metrics_path = os.path.join(output_dir, 'bug1_metrics.csv')

        if not os.path.exists(self.metrics_path):
            with open(self.metrics_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'timestamp', 'scenario', 'planner', 'goal_index', 'success',
                    'duration_s', 'path_length_m', 'start_to_goal_dist_m',
                    'wall_follow_entries', 'circumnavigations', 'recovery_events',
                    'final_state', 'notes'
                ])

    def _start_goal_metrics(self):
        """
        Start timing and path metrics for a new goal.
        Called when a new goal is received.
        """
        self.metrics_goal_active = True
        self.goal_start_time_ns = self.get_clock().now().nanoseconds
        self.goal_start_pose = None
        if self.current_pose is not None:
            self.goal_start_pose = Point(x=self.current_pose.x, y=self.current_pose.y, z=self.current_pose.z)
        self.prev_pose_for_path = None
        self.path_length_m = 0.0
        self.wall_follow_entries = 0
        self.recovery_event_count = 0
        self.circumnavigation_count = 0
        self.goal_counter += 1

    def _finish_goal_metrics(self, success: bool, final_state: str, notes: str = ''):
        """
        Finish and log metrics for the current goal attempt.
        Called on goal reached, interruption, or failure.
        """
        if not self.metrics_goal_active:
            return
        self.metrics_goal_active = False

        if self.metrics_path is None:
            return

        now_ns = self.get_clock().now().nanoseconds
        duration_s = max(0.0, (now_ns - self.goal_start_time_ns) / 1e9)

        start_to_goal_dist = ''
        if self.goal_internal is not None and self.goal_start_pose is not None:
            start_to_goal_dist = f"{self.distance(self.goal_start_pose, self.goal_internal):.3f}"

        scenario_name = self.get_parameter('scenario_name').get_parameter_value().string_value
        row = [
            datetime.now().isoformat(timespec='seconds'),
            scenario_name,
            'bug1',
            self.goal_counter,
            int(success),
            f"{duration_s:.3f}",
            f"{self.path_length_m:.3f}",
            start_to_goal_dist,
            self.wall_follow_entries,
            self.circumnavigation_count,
            self.recovery_event_count,
            final_state,
            notes,
        ]

        with open(self.metrics_path, 'a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(row)

    def goal_callback(self, msg):
        """
        Callback for new goal pose.
        Resets state and starts metrics.
        """
        if self.metrics_goal_active:
            self._finish_goal_metrics(False, 'INTERRUPTED', 'New goal received before previous goal completion')

        self.goal_pose_stamped = msg
        self.goal_internal = None
        self.state = Bug1State.GO_TO_GOAL
        self.obstacle_hit_point = None
        self.left_hit_area_once = False
        self.closest_point_on_boundary = None
        self.min_dist_to_goal = float('inf')
        self.stuck_counter = 0
        self.recovery_steps = 0
        self.corner_cooldown_steps = 0
        self.last_pose_for_stuck = None
        self._start_goal_metrics()
        self.get_logger().info('New goal received. Reset state to GO_TO_GOAL.')

    def odom_callback(self, msg):
        """
        Odometry callback: updates robot pose and path for visualization and metrics.
        """
        self.current_pose = msg.pose.pose.position
        self.odom_frame = msg.header.frame_id
        self.current_yaw = euler_from_quaternion(msg.pose.pose.orientation)

        if self.metrics_goal_active:
            curr = Point(x=self.current_pose.x, y=self.current_pose.y, z=self.current_pose.z)
            if self.prev_pose_for_path is not None:
                self.path_length_m += self.distance(curr, self.prev_pose_for_path)
            self.prev_pose_for_path = curr

        # Publish trail path
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = self.odom_frame
        pose_stamped.pose = msg.pose.pose
        self.path_msg.poses.append(pose_stamped)
        if len(self.path_msg.poses) > 700:
            self.path_msg.poses = self.path_msg.poses[-700:]
        self.path_msg.header = pose_stamped.header
        self.path_publisher.publish(self.path_msg)

    def scan_callback(self, msg):
        """
        Laser scan callback: stores latest scan for obstacle checks.
        """
        self.scan_data = msg

    def normalize_angle(self, angle):
        """
        Normalize angle to [-pi, pi].
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def distance(self, p1, p2):
        """
        Euclidean distance between two geometry_msgs Points.
        """
        return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)

    def sanitize_range(self, r, max_scan_range):
        """
        Clamp invalid scan values to a safe max range.
        """
        if math.isinf(r) or math.isnan(r):
            return max_scan_range
        if self.scan_data and r < self.scan_data.range_min:
            return max_scan_range
        if r > max_scan_range:
            return max_scan_range
        return r

    def front_blocked(self, threshold):
        """
        Check if the front arc is blocked within a threshold.
        Returns (blocked: bool, min_front: float).
        """
        if self.scan_data is None:
            return True, 0.0

        ranges = self.scan_data.ranges
        if not ranges or self.scan_data.angle_increment == 0.0:
            return True, 0.0

        max_scan_range = self.get_parameter('max_scan_range').get_parameter_value().double_value
        center_index = int((-self.scan_data.angle_min) / self.scan_data.angle_increment)
        check_angle = math.radians(18.0)
        n = max(1, int(check_angle / self.scan_data.angle_increment))

        min_front = max_scan_range
        blocked = False
        for i in range(n + 1):
            for s in (1, -1):
                idx = (center_index + s * i) % len(ranges)
                r = self.sanitize_range(ranges[idx], max_scan_range)
                min_front = min(min_front, r)
                if r <= threshold:
                    blocked = True

        return blocked, min_front

    def sector_min_range(self, start_deg, end_deg, max_scan_range):
        """
        Minimum range in [start_deg, end_deg] sector (degrees in robot frame).
        Used for body/edge safety checks.
        """
        if self.scan_data is None or not self.scan_data.ranges or self.scan_data.angle_increment == 0.0:
            return max_scan_range

        angle_min = self.scan_data.angle_min
        angle_inc = self.scan_data.angle_increment
        ranges = self.scan_data.ranges

        start_idx = int((math.radians(start_deg) - angle_min) / angle_inc)
        end_idx = int((math.radians(end_deg) - angle_min) / angle_inc)

        rmin = max_scan_range
        for idx in range(min(start_idx, end_idx), max(start_idx, end_idx) + 1):
            r = self.sanitize_range(ranges[idx % len(ranges)], max_scan_range)
            rmin = min(rmin, r)
        return rmin

    def compute_wall_follow_command(self, lin_speed, ang_speed, wall_dist):
        """
        Right-hand wall-follow command with corner protection and recovery.
        Returns a Twist command.
        """
        max_scan_range = self.get_parameter('max_scan_range').get_parameter_value().double_value
        obs_threshold = self.get_parameter('obstacle_threshold').get_parameter_value().double_value
        clearance_margin = self.get_parameter('clearance_margin').get_parameter_value().double_value
        effective_obs = obs_threshold + clearance_margin

        ranges = self.scan_data.ranges
        angle_min = self.scan_data.angle_min
        angle_inc = self.scan_data.angle_increment

        # Right distance around -90 deg
        right_idx = int((math.radians(-90.0) - angle_min) / angle_inc)
        right_dist = max_scan_range
        for off in range(-3, 4):
            r = self.sanitize_range(ranges[(right_idx + off) % len(ranges)], max_scan_range)
            right_dist = min(right_dist, r)

        blocked, front_dist = self.front_blocked(effective_obs)
        front_left = self.sector_min_range(20.0, 60.0, max_scan_range)
        front_right = self.sector_min_range(-60.0, -20.0, max_scan_range)

        cmd = Twist()

        # Stuck detection near front/corners.
        if self.last_pose_for_stuck is None:
            self.last_pose_for_stuck = Point(x=self.current_pose.x, y=self.current_pose.y, z=self.current_pose.z)
        else:
            moved = self.distance(self.current_pose, self.last_pose_for_stuck)
            near_corner = min(front_dist, front_left, front_right) < (effective_obs + 0.03)
            if near_corner and moved < 0.01:
                self.stuck_counter += 1
            else:
                self.stuck_counter = max(0, self.stuck_counter - 1)
            self.last_pose_for_stuck = Point(x=self.current_pose.x, y=self.current_pose.y, z=self.current_pose.z)

        if self.stuck_counter > 10 and self.recovery_steps == 0:
            self.recovery_steps = 10
            self.corner_cooldown_steps = 10
            self.recovery_event_count += 1
            self.get_logger().warn('[BUG1] Stuck near edge. Starting recovery maneuver.')

        if self.recovery_steps > 0:
            self.recovery_steps -= 1
            cmd.linear.x = -0.06
            cmd.angular.z = ang_speed if front_left > front_right else -ang_speed
            return cmd

        # Corner guards to protect wheel edges
        corner_guard = effective_obs + 0.02
        if front_right < corner_guard:
            cmd.linear.x = 0.0
            cmd.angular.z = ang_speed * 0.45
            self.corner_cooldown_steps = 8
            return cmd

        if front_left < corner_guard:
            cmd.linear.x = 0.0
            cmd.angular.z = -ang_speed * 0.45
            self.corner_cooldown_steps = 8
            return cmd

        if blocked:
            # Turn in place left to escape direct collision
            cmd.linear.x = 0.0
            cmd.angular.z = ang_speed if front_left > front_right else -ang_speed
            self.corner_cooldown_steps = 8
            return cmd

        wall_lost_threshold = wall_dist * 1.8
        if right_dist > wall_lost_threshold:
            # Search right wall
            cmd.linear.x = lin_speed * 0.50
            cmd.angular.z = -ang_speed * 0.55
            return cmd

        # Standard proportional wall-follow
        wall_err = wall_dist - right_dist
        kp = 1.15
        cmd.angular.z = kp * wall_err
        cmd.angular.z = max(-ang_speed, min(ang_speed, cmd.angular.z))

        if front_dist < wall_dist * 1.6 or abs(cmd.angular.z) > ang_speed * 0.65:
            cmd.linear.x = lin_speed * 0.40
        else:
            cmd.linear.x = lin_speed * 0.70

        if self.corner_cooldown_steps > 0:
            self.corner_cooldown_steps -= 1
            cmd.linear.x = min(cmd.linear.x, lin_speed * 0.30)
            cmd.angular.z = max(-ang_speed * 0.60, min(ang_speed * 0.60, cmd.angular.z))

        return cmd

    def track_closest_boundary_point(self):
        """
        Update closest point on boundary to goal during circumnavigation.
        """
        if self.current_pose is None or self.goal_internal is None:
            return

        d = self.distance(self.current_pose, self.goal_internal)
        if d < self.min_dist_to_goal:
            self.min_dist_to_goal = d
            self.closest_point_on_boundary = Point(
                x=self.current_pose.x,
                y=self.current_pose.y,
                z=self.current_pose.z,
            )

    def check_full_circumnavigation(self):
        """
        Return True if robot left hit area and came back to it (full loop).
        """
        if self.obstacle_hit_point is None or self.current_pose is None:
            return False

        hit_tol = self.get_parameter('loop_hit_tolerance').get_parameter_value().double_value
        d_hit = self.distance(self.current_pose, self.obstacle_hit_point)

        if d_hit > hit_tol * 1.8:
            self.left_hit_area_once = True

        return self.left_hit_area_once and d_hit <= hit_tol

    def control_loop(self):
        """
        Main control loop for Bug 1 state machine.
        Handles GO_TO_GOAL, WALL_FOLLOW, GO_TO_CLOSEST_POINT, and GOAL_REACHED states.
        Publishes Twist commands and manages transitions.
        """
        if self.scan_data is None or self.current_pose is None or self.odom_frame is None:
            return

        if self.goal_pose_stamped is None:
            self.stop_robot()
            return

        # Transform goal once for current target
        if self.goal_internal is None:
            target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
            try:
                if self.odom_frame != target_frame:
                    self.get_logger().error(
                        f"Odometry frame '{self.odom_frame}' does not match target frame '{target_frame}'",
                        once=True)
                    self.stop_robot()
                    return

                goal_copy = copy(self.goal_pose_stamped)
                goal_copy.header.stamp.sec = 0
                goal_copy.header.stamp.nanosec = 0

                transformed = self.tf_buffer.transform(
                    goal_copy, target_frame,
                    timeout=rclpy.duration.Duration(seconds=0.5))
                self.goal_internal = transformed.pose.position
                self.get_logger().info(f"Goal transformed to '{target_frame}' frame")
            except TransformException as e:
                self.get_logger().error(f'Could not transform goal: {e}')
                self.stop_robot()
                return

        # Global goal reached check
        goal_tol = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        if self.distance(self.current_pose, self.goal_internal) < goal_tol:
            if self.state != Bug1State.GOAL_REACHED:
                self.state = Bug1State.GOAL_REACHED
                self.get_logger().info('Goal reached!')
                self._finish_goal_metrics(True, 'GOAL_REACHED')
            self.stop_robot()
            return

        lin_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        ang_speed = self.get_parameter('angular_speed').get_parameter_value().double_value
        wall_dist = self.get_parameter('wall_follow_dist').get_parameter_value().double_value
        obs_threshold = self.get_parameter('obstacle_threshold').get_parameter_value().double_value
        clearance_margin = self.get_parameter('clearance_margin').get_parameter_value().double_value
        effective_obs = obs_threshold + clearance_margin

        cmd = Twist()

        if self.state == Bug1State.GO_TO_GOAL:
            blocked, min_front = self.front_blocked(effective_obs)
            max_scan_range = self.get_parameter('max_scan_range').get_parameter_value().double_value
            front_left = self.sector_min_range(20.0, 60.0, max_scan_range)
            front_right = self.sector_min_range(-60.0, -20.0, max_scan_range)
            if front_left < (effective_obs + 0.03) and front_right < (effective_obs + 0.03):
                blocked = True

            if blocked:
                # Hit obstacle: start Bug1 circumnavigation
                if self.state != Bug1State.WALL_FOLLOW:
                    self.wall_follow_entries += 1
                self.state = Bug1State.WALL_FOLLOW
                self.obstacle_hit_point = Point(
                    x=self.current_pose.x,
                    y=self.current_pose.y,
                    z=self.current_pose.z,
                )
                self.left_hit_area_once = False
                self.closest_point_on_boundary = Point(
                    x=self.current_pose.x,
                    y=self.current_pose.y,
                    z=self.current_pose.z,
                )
                self.min_dist_to_goal = self.distance(self.current_pose, self.goal_internal)
                self.get_logger().info('Transition to WALL_FOLLOW (start circumnavigation).')
                self.stop_robot()
                return

            # Move toward original goal
            angle_to_goal = math.atan2(
                self.goal_internal.y - self.current_pose.y,
                self.goal_internal.x - self.current_pose.x)
            angle_err = self.normalize_angle(angle_to_goal - self.current_yaw)
            cmd.angular.z = max(-ang_speed, min(ang_speed, ang_speed * math.tanh(angle_err)))
            if abs(angle_err) < math.radians(28.0):
                proximity = min(min_front, front_left, front_right)
                if proximity < (effective_obs + 0.12):
                    cmd.linear.x = lin_speed * 0.65
                else:
                    cmd.linear.x = lin_speed
            else:
                cmd.linear.x = 0.0

        elif self.state == Bug1State.WALL_FOLLOW:
            # Track best boundary point while making full loop
            self.track_closest_boundary_point()

            if self.check_full_circumnavigation():
                self.circumnavigation_count += 1
                self.state = Bug1State.GO_TO_CLOSEST_POINT
                self.get_logger().info('Full circumnavigation completed. Transition to GO_TO_CLOSEST_POINT.')
                self.stop_robot()
                return

            cmd = self.compute_wall_follow_command(lin_speed, ang_speed, wall_dist)

        elif self.state == Bug1State.GO_TO_CLOSEST_POINT:
            # Continue along boundary until reaching the stored closest boundary point
            if self.closest_point_on_boundary is None:
                # Safety fallback
                self.state = Bug1State.GO_TO_GOAL
                self.get_logger().warn('Closest point missing, fallback to GO_TO_GOAL.')
                self.stop_robot()
                return

            d_closest = self.distance(self.current_pose, self.closest_point_on_boundary)
            if d_closest < goal_tol:
                self.state = Bug1State.GO_TO_GOAL
                self.get_logger().info('Reached closest boundary point. Resume GO_TO_GOAL.')
                self.stop_robot()
                return

            cmd = self.compute_wall_follow_command(lin_speed, ang_speed, wall_dist)

        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        if self.state != Bug1State.GOAL_REACHED:
            self.cmd_vel_publisher.publish(cmd)

    def stop_robot(self):
        """
        Publish zero Twist to stop the robot.
        """
        cmd = Twist()
        self.cmd_vel_publisher.publish(cmd)


def main(args=None):
    """
    Main entry point for Bug 1 planner node.
    """
    rclpy.init(args=args)
    node = Bug1PlannerNode()
    try:
        rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        node.stop_robot()
        node.get_logger().info('Bug 1 node stopped')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

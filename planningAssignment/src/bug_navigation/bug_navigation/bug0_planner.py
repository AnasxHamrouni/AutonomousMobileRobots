#!/usr/bin/env python3
"""
Bug 0 Path Planning Algorithm - Fixed Version

Task 1 Implementation following homework instructions:
- GO_TO_GOAL: Move straight towards goal until obstacle detected
- WALL_FOLLOW: Follow the obstacle boundary, continuously check if goal direction is clear
- Leave Condition: When direct path to goal becomes clear, immediately return to GO_TO_GOAL
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Quaternion, PoseStamped 
from sensor_msgs.msg import LaserScan
import math
import os
import csv
from datetime import datetime
from enum import Enum
from copy import copy
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException
from nav_msgs.msg import Odometry, Path 


def euler_from_quaternion(quaternion):
    """Convert quaternion to euler angles"""
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z


class BugState(Enum):
    GO_TO_GOAL = 1
    WALL_FOLLOW = 2
    GOAL_REACHED = 3


class Bug0PlannerNode(Node):
    """
    Bug 0 Algorithm Node (Task 1)
    Implements the Bug 0 path planning algorithm for a differential drive robot in ROS 2.

    Main states:
    - GO_TO_GOAL: Move directly toward the goal until an obstacle is detected.
    - WALL_FOLLOW: Follow the obstacle boundary, keeping the wall to the right.
    - GOAL_REACHED: Stop when the goal is reached within tolerance.

    Key logic:
    - When an obstacle is detected in front, transition to WALL_FOLLOW.
    - While wall following, continuously check if the direct path to the goal is clear (using laser scan in the goal direction).
    - If the path to the goal is clear, immediately return to GO_TO_GOAL.
    - No M-line or minimum distance checks (unlike Bug1/Tangent Bug).

    Metrics logging:
    - Logs run statistics (success, time, path length, wall follow entries, recovery events) to CSV for quantitative comparison.

    Parameters:
    - wall_follow_dist: Desired distance to keep from the wall during wall following.
    - obstacle_threshold: Distance threshold to consider an obstacle detected.
    - body_safety_dist: Minimum safe distance to any obstacle (for edge/wheel protection).
    - goal_tolerance: Distance to goal for success.
    - linear_speed, angular_speed: Robot speeds.
    - control_loop_freq: Main control loop frequency.
    - scenario_name: Tag for metrics logging.
    """
    def __init__(self):
        super().__init__('bug0_planner_node')

        # Declare parameters for tuning and reproducibility
        self.declare_parameter('wall_follow_dist', 0.4)  # Desired wall distance
        self.declare_parameter('max_scan_range', 10.0)   # Max valid scan range
        self.declare_parameter('obstacle_threshold', 0.38)  # Obstacle detection threshold
        self.declare_parameter('clearance_margin', 0.05)    # Extra margin for safety
        self.declare_parameter('body_safety_dist', 0.24)    # Side/wheel safety margin
        self.declare_parameter('goal_tolerance', 0.25)      # Goal reached tolerance
        self.declare_parameter('linear_speed', 0.20)        # Forward speed
        self.declare_parameter('angular_speed', 0.5)        # Turn speed
        self.declare_parameter('control_loop_freq', 10.0)   # Control loop frequency (Hz)
        self.declare_parameter('target_frame', 'odom')      # Frame for goal transformation
        self.declare_parameter('enable_metrics_logging', True)  # Enable CSV logging
        self.declare_parameter('metrics_output_dir', 'metrics') # Output dir for metrics
        self.declare_parameter('scenario_name', 'default')      # Scenario tag for metrics

        # Goal state
        self.goal_pose_stamped = None
        self.goal_internal = None

        # Robot state
        self.current_pose = None
        self.current_yaw = 0.0
        self.odom_frame = None
        self.scan_data = None

        # Bug algorithm state
        self.obstacle_hit_point = None
        self.state = BugState.GO_TO_GOAL
        self.stuck_counter = 0
        self.recovery_steps = 0
        self.last_pose_for_stuck = None
        self.corner_cooldown_steps = 0

        # Path visualization
        self.path_msg = Path()

        # Metrics state
        self.metrics_goal_active = False
        self.goal_start_time_ns = 0
        self.goal_start_pose = None
        self.prev_pose_for_path = None
        self.path_length_m = 0.0
        self.wall_follow_entries = 0
        self.recovery_event_count = 0
        self.goal_counter = 0
        self.metrics_path = None

        self._init_metrics_logging()

        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.path_publisher = self.create_publisher(Path, 'robot_path', 10)

        # Subscribers
        self.goal_subscriber = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.odom_subscriber = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.scan_subscriber = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 
            rclpy.qos.qos_profile_sensor_data)

        # Control loop
        freq = self.get_parameter('control_loop_freq').get_parameter_value().double_value
        self.timer = self.create_timer(1.0 / freq, self.control_loop)

        self.get_logger().info("Bug 0 Planner Node Initialized.")

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
        self.metrics_path = os.path.join(output_dir, 'bug0_metrics.csv')

        if not os.path.exists(self.metrics_path):
            with open(self.metrics_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'timestamp', 'scenario', 'planner', 'goal_index', 'success',
                    'duration_s', 'path_length_m', 'start_to_goal_dist_m',
                    'wall_follow_entries', 'recovery_events', 'final_state', 'notes'
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
            'bug0',
            self.goal_counter,
            int(success),
            f"{duration_s:.3f}",
            f"{self.path_length_m:.3f}",
            start_to_goal_dist,
            self.wall_follow_entries,
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
        self.get_logger().info(f"New goal received")
        if self.metrics_goal_active:
            self._finish_goal_metrics(False, 'INTERRUPTED', 'New goal received before previous goal completion')

        self.goal_pose_stamped = msg
        self.goal_internal = None
        self.state = BugState.GO_TO_GOAL
        self.obstacle_hit_point = None
        self.stuck_counter = 0
        self.recovery_steps = 0
        self.last_pose_for_stuck = None
        self.corner_cooldown_steps = 0
        self._start_goal_metrics()

    def odom_callback(self, msg):
        """
        Odometry callback: updates robot pose and path for visualization and metrics.
        """
        self.current_pose = msg.pose.pose.position
        self.odom_frame = msg.header.frame_id
        _, _, self.current_yaw = euler_from_quaternion(msg.pose.pose.orientation)

        if self.metrics_goal_active:
            curr = Point(x=self.current_pose.x, y=self.current_pose.y, z=self.current_pose.z)
            if self.prev_pose_for_path is not None:
                self.path_length_m += self.distance(curr, self.prev_pose_for_path)
            self.prev_pose_for_path = curr
        
        # Publish path
        if self.current_pose:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = self.odom_frame
            pose_stamped.pose = msg.pose.pose
            self.path_msg.poses.append(pose_stamped)
            if len(self.path_msg.poses) > 500:
                self.path_msg.poses = self.path_msg.poses[-500:]
            self.path_msg.header = pose_stamped.header
            self.path_publisher.publish(self.path_msg)

    def scan_callback(self, msg):
        """
        Laser scan callback: stores latest scan for obstacle checks.
        """
        self.scan_data = msg

    def distance(self, p1, p2):
        """
        Euclidean distance between two geometry_msgs Points.
        """
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

    def normalize_angle(self, angle):
        """
        Normalize angle to [-pi, pi].
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def sanitize_range(self, r, max_range):
        """
        Clamp invalid scan values to a safe max range.
        """
        if self.scan_data is None:
            return max_range
        if math.isinf(r) or math.isnan(r) or r < self.scan_data.range_min:
            return max_range
        if r > max_range:
            return max_range
        return r

    def sector_min_range(self, start_deg, end_deg, max_range):
        """
        Minimum range in [start_deg, end_deg] sector (robot frame, degrees).
        Used for body/edge safety checks.
        """
        if self.scan_data is None or not self.scan_data.ranges or self.scan_data.angle_increment == 0:
            return max_range

        angle_min = self.scan_data.angle_min
        angle_inc = self.scan_data.angle_increment
        ranges = self.scan_data.ranges

        start_idx = int((math.radians(start_deg) - angle_min) / angle_inc)
        end_idx = int((math.radians(end_deg) - angle_min) / angle_inc)
        rmin = max_range
        for idx in range(min(start_idx, end_idx), max(start_idx, end_idx) + 1):
            r = self.sanitize_range(ranges[idx % len(ranges)], max_range)
            rmin = min(rmin, r)
        return rmin

    def is_goal_direction_clear(self):
        """
        Check if direct path to goal is clear using a sector of laser scan.
        This is the ONLY leave condition for Bug 0 (per assignment).
        """
        if self.goal_internal is None or self.scan_data is None or self.current_pose is None:
            return False

        # Calculate goal direction
        angle_to_goal = math.atan2(
            self.goal_internal.y - self.current_pose.y,
            self.goal_internal.x - self.current_pose.x)
        rel_angle = self.normalize_angle(angle_to_goal - self.current_yaw)

        # Get scan parameters
        obstacle_threshold = self.get_parameter('obstacle_threshold').get_parameter_value().double_value
        max_scan_range = self.get_parameter('max_scan_range').get_parameter_value().double_value
        angle_min = self.scan_data.angle_min
        angle_inc = self.scan_data.angle_increment
        ranges = self.scan_data.ranges

        if not ranges or angle_inc == 0:
            return False

        # Find indices pointing towards goal
        # Check a 20° arc centered on goal direction
        check_span = math.radians(20.0)
        num_samples = max(1, int(check_span / angle_inc))

        goal_idx = int((rel_angle - angle_min) / angle_inc)
        
        # Check samples in both directions
        for offset in range(num_samples + 1):
            for sign in [1, -1]:
                idx = (goal_idx + sign * offset) % len(ranges)
                r = ranges[idx]
                
                # Handle invalid readings
                if math.isinf(r) or math.isnan(r) or r < self.scan_data.range_min:
                    r = max_scan_range
                if r > max_scan_range:
                    r = max_scan_range
                
                # If ANY sample is blocked, path not clear
                if r <= obstacle_threshold:
                    return False

        self.get_logger().info("[BUG0] Goal direction CLEAR - leaving wall")
        return True

    def control_loop(self):
        """
        Main control loop for Bug 0 state machine.
        Handles GO_TO_GOAL, WALL_FOLLOW, and GOAL_REACHED states.
        Publishes Twist commands and manages transitions.
        """
        # Wait for data
        if self.scan_data is None or self.current_pose is None or self.odom_frame is None:
            return

        # Wait for goal
        if self.goal_pose_stamped is None:
            self.stop_robot()
            return

        # Transform goal to robot frame if needed
        if self.goal_internal is None:
            try:
                target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
                
                goal_copy = copy(self.goal_pose_stamped)
                # Set timestamp to 0 = use latest available transform (avoids time mismatch)
                goal_copy.header.stamp.sec = 0
                goal_copy.header.stamp.nanosec = 0
                
                transformed = self.tf_buffer.transform(
                    goal_copy, target_frame,
                    timeout=rclpy.duration.Duration(seconds=0.5))
                self.goal_internal = transformed.pose.position
                self.get_logger().info(f"[BUG0] Goal transformed to {target_frame}")
            except TransformException as e:
                self.get_logger().error(f"Transform error: {e}")
                self.stop_robot()
                return

        # Check goal reached
        dist_to_goal = self.distance(self.current_pose, self.goal_internal)
        goal_tol = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        
        if dist_to_goal < goal_tol:
            if self.state != BugState.GOAL_REACHED:
                self.get_logger().info(f"[BUG0] GOAL REACHED! Distance: {dist_to_goal:.3f}m")
                self.state = BugState.GOAL_REACHED
                self._finish_goal_metrics(True, 'GOAL_REACHED')
            self.stop_robot()
            return

        # Get parameters
        lin_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        ang_speed = self.get_parameter('angular_speed').get_parameter_value().double_value
        obs_threshold = self.get_parameter('obstacle_threshold').get_parameter_value().double_value
        clearance_margin = self.get_parameter('clearance_margin').get_parameter_value().double_value
        effective_obs_threshold = obs_threshold + clearance_margin
        max_range = self.get_parameter('max_scan_range').get_parameter_value().double_value

        cmd = Twist()

        # GO_TO_GOAL STATE
        if self.state == BugState.GO_TO_GOAL:
            # Task 1: "move towards the goal until an obstacle is detected within the threshold"
            
            # Check if obstacle in front
            angle_min = self.scan_data.angle_min
            angle_inc = self.scan_data.angle_increment
            ranges = self.scan_data.ranges
            
            if not ranges or angle_inc == 0:
                self.stop_robot()
                return

            center_idx = int(-angle_min / angle_inc)
            front_span = math.radians(22.0)
            num_samples = max(1, int(front_span / angle_inc))
            
            obstacle_detected = False
            min_front = max_range
            
            for offset in range(num_samples + 1):
                for sign in [1, -1]:
                    idx = (center_idx + sign * offset) % len(ranges)
                    r = self.sanitize_range(ranges[idx], max_range)
                    min_front = min(min_front, r)
                    
                    if r <= effective_obs_threshold:
                        obstacle_detected = True
            
            # Extra corner checks (wheel-edge protection near doors)
            front_left = self.sector_min_range(20.0, 60.0, max_range)
            front_right = self.sector_min_range(-60.0, -20.0, max_range)
            left_side = self.sector_min_range(70.0, 110.0, max_range)
            right_side = self.sector_min_range(-110.0, -70.0, max_range)
            body_safety = self.get_parameter('body_safety_dist').get_parameter_value().double_value

            # Only declare corner blockage when BOTH corners are tight.
            corner_threshold = effective_obs_threshold + 0.04
            if front_left < corner_threshold and front_right < corner_threshold:
                obstacle_detected = True

            # Body side clearance: avoid wheel/edge scraping on oblique obstacle faces.
            if min(front_left, front_right, left_side, right_side) < body_safety:
                obstacle_detected = True

            if obstacle_detected:
                # Obstacle found - enter wall following
                self.get_logger().info(f"[BUG0] Obstacle detected! Dist={min_front:.2f}m, entering WALL_FOLLOW")
                if self.state != BugState.WALL_FOLLOW:
                    self.wall_follow_entries += 1
                self.state = BugState.WALL_FOLLOW
                self.obstacle_hit_point = Point(
                    x=self.current_pose.x,
                    y=self.current_pose.y,
                    z=self.current_pose.z)
                self.stop_robot()
                return
            
            # No obstacle - move towards goal
            angle_to_goal = math.atan2(
                self.goal_internal.y - self.current_pose.y,
                self.goal_internal.x - self.current_pose.x)
            angle_err = self.normalize_angle(angle_to_goal - self.current_yaw)
            
            # Simple proportional turn control
            cmd.angular.z = ang_speed * math.tanh(angle_err)  # tanh keeps turn reasonable
            
            # Move forward if roughly oriented to goal
            if abs(angle_err) < math.radians(30.0):
                # Slow down near obstacles/corners to avoid wheel clipping.
                proximity = min(min_front, front_left, front_right, left_side, right_side)
                if proximity < body_safety + 0.04:
                    cmd.linear.x = lin_speed * 0.30
                elif proximity < effective_obs_threshold + 0.12:
                    cmd.linear.x = lin_speed * 0.55
                else:
                    cmd.linear.x = lin_speed

                # Add small steering bias away from the closer corner.
                if front_left < front_right:
                    cmd.angular.z -= 0.08
                elif front_right < front_left:
                    cmd.angular.z += 0.08
            else:
                cmd.linear.x = 0.0

        # WALL_FOLLOW STATE 
        elif self.state == BugState.WALL_FOLLOW:
            # Task 1: "continuously check if the direct path towards the original goal becomes clear again"
            # "If the direct path to the goal is clear, transition immediately back to GO_TO_GOAL"
            
            if self.is_goal_direction_clear():
                self.state = BugState.GO_TO_GOAL
                self.get_logger().info("[BUG0] Transitioning back to GO_TO_GOAL")
                self.stuck_counter = 0
                self.recovery_steps = 0
                self.last_pose_for_stuck = None
                self.corner_cooldown_steps = 0
                cmd.angular.z = 0.0
                cmd.linear.x = 0.0
                self.cmd_vel_publisher.publish(cmd)
                return
            
            # Wall following: Move around the obstacle
            # Simple strategy: keep left/right wall at target distance using proportional control
            
            angle_min = self.scan_data.angle_min
            angle_inc = self.scan_data.angle_increment
            max_range = self.get_parameter('max_scan_range').get_parameter_value().double_value
            wall_dist = self.get_parameter('wall_follow_dist').get_parameter_value().double_value
            ranges = self.scan_data.ranges
            
            if not ranges or angle_inc == 0:
                self.stop_robot()
                return

            # Find wall on the right (-90°)
            right_angle = math.radians(-90.0)
            right_idx = int((right_angle - angle_min) / angle_inc)
            right_range = max_range
            
            # Sample a few indices around right side
            for offset in range(-3, 4):
                idx = (right_idx + offset) % len(ranges)
                r = self.sanitize_range(ranges[idx], max_range)
                right_range = min(right_range, r)
            
            # Find front distance
            center_idx = int(-angle_min / angle_inc)
            front_range = max_range
            for offset in range(-2, 3):
                idx = (center_idx + offset) % len(ranges)
                r = self.sanitize_range(ranges[idx], max_range)
                front_range = min(front_range, r)

            # Front-left / front-right for corner avoidance near doors
            front_left_range = self.sector_min_range(20.0, 60.0, max_range)
            front_right_range = self.sector_min_range(-60.0, -20.0, max_range)
            left_side_range = self.sector_min_range(70.0, 110.0, max_range)
            right_side_range = self.sector_min_range(-110.0, -70.0, max_range)
            body_safety = self.get_parameter('body_safety_dist').get_parameter_value().double_value
            
            # Robust right-hand wall following:
            # 1) If a wall is directly ahead, rotate left in place to escape corners.
            # 2) If wall on the right is lost, steer right to reacquire it.
            # 3) Otherwise follow right wall using P control.
            front_escape_threshold = max(obs_threshold + 0.03, body_safety + 0.03)
            wall_lost_threshold = wall_dist * 1.8

            # Stuck detection: commanded motion but no meaningful position change.
            if self.last_pose_for_stuck is None:
                self.last_pose_for_stuck = Point(x=self.current_pose.x, y=self.current_pose.y, z=self.current_pose.z)
            else:
                move_dist = self.distance(self.current_pose, self.last_pose_for_stuck)
                if min(front_range, front_left_range, front_right_range) < front_escape_threshold and move_dist < 0.01:
                    self.stuck_counter += 1
                else:
                    self.stuck_counter = max(0, self.stuck_counter - 1)
                self.last_pose_for_stuck = Point(x=self.current_pose.x, y=self.current_pose.y, z=self.current_pose.z)

            if self.stuck_counter > 10 and self.recovery_steps == 0:
                self.recovery_steps = 10
                self.corner_cooldown_steps = 10
                self.recovery_event_count += 1
                self.get_logger().warn("[BUG0][WF] Stuck near obstacle edge. Starting recovery maneuver.")

            if self.recovery_steps > 0:
                self.recovery_steps -= 1
                # Reverse slightly and turn toward more open side.
                cmd.linear.x = -0.09
                cmd.angular.z = ang_speed if front_left_range > front_right_range else -ang_speed
                self.cmd_vel_publisher.publish(cmd)
                return

            # Wheel-edge guard near door frames/corners.
            corner_guard = max(effective_obs_threshold + 0.03, body_safety + 0.02)
            if front_right_range < corner_guard:
                cmd.linear.x = 0.0
                cmd.angular.z = ang_speed * 0.65
                self.corner_cooldown_steps = 8
                self.get_logger().debug(
                    f"[BUG0][WF] Right corner too close ({front_right_range:.2f}m). Bias left.",
                    throttle_duration_sec=0.4)
                self.cmd_vel_publisher.publish(cmd)
                return
            if front_left_range < corner_guard:
                cmd.linear.x = 0.0
                cmd.angular.z = -ang_speed * 0.65
                self.corner_cooldown_steps = 8
                self.get_logger().debug(
                    f"[BUG0][WF] Left corner too close ({front_left_range:.2f}m). Bias right.",
                    throttle_duration_sec=0.4)
                self.cmd_vel_publisher.publish(cmd)
                return

            # Side body protection: if robot side gets too close to obstacle edges, back off and rotate.
            if min(left_side_range, right_side_range) < (body_safety - 0.02):
                cmd.linear.x = -0.05
                cmd.angular.z = ang_speed if left_side_range > right_side_range else -ang_speed
                self.corner_cooldown_steps = 10
                self.cmd_vel_publisher.publish(cmd)
                return

            if front_range < front_escape_threshold:
                # Head-on obstacle: prioritize turning in place over forward motion.
                cmd.linear.x = 0.0
                cmd.angular.z = ang_speed if front_left_range > front_right_range else -ang_speed
                self.corner_cooldown_steps = 8
                self.get_logger().debug(
                    f"[BUG0][WF] Front blocked ({front_range:.2f}m). Escape turn left.",
                    throttle_duration_sec=0.5)
            elif right_range > wall_lost_threshold:
                # Lost right wall: arc right to find boundary again.
                cmd.linear.x = lin_speed * 0.35
                cmd.angular.z = -ang_speed * 0.55
                self.get_logger().debug(
                    f"[BUG0][WF] Right wall lost ({right_range:.2f}m). Searching right.",
                    throttle_duration_sec=0.5)
            else:
                # Normal proportional wall following.
                wall_err = wall_dist - right_range
                kp = 1.2
                cmd.angular.z = kp * wall_err
                cmd.angular.z = max(-ang_speed, min(ang_speed, cmd.angular.z))

                if abs(cmd.angular.z) > ang_speed * 0.6:
                    cmd.linear.x = lin_speed * 0.30
                else:
                    cmd.linear.x = lin_speed * 0.55

                self.get_logger().debug(
                    f"[BUG0][WF] front={front_range:.2f} right={right_range:.2f} "
                    f"err={wall_err:.2f} cmd=({cmd.linear.x:.2f},{cmd.angular.z:.2f})",
                    throttle_duration_sec=0.5)

            # After sharp corner turns, creep forward briefly to avoid immediate wheel clipping.
            if self.corner_cooldown_steps > 0:
                self.corner_cooldown_steps -= 1
                cmd.linear.x = min(cmd.linear.x, lin_speed * 0.30)
                cmd.angular.z = max(-ang_speed * 0.60, min(ang_speed * 0.60, cmd.angular.z))

        else:  # GOAL_REACHED
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        # Publish command
        if self.state != BugState.GOAL_REACHED:
            self.cmd_vel_publisher.publish(cmd)

    def stop_robot(self):
        """Stop the robot"""
        cmd = Twist()
        self.cmd_vel_publisher.publish(cmd)


def main(args=None):
    """
    Main entry point for Bug 0 planner node.
    """
    rclpy.init(args=args)
    node = Bug0PlannerNode()
    try:
        rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        node.stop_robot()
        node.get_logger().info("Bug 0 node stopped")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

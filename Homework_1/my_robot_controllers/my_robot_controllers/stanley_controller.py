#!/usr/bin/env python3

import math

import numpy as np
import rclpy

from my_robot_controllers.controller_base import WaypointControllerBase, normalize_angle, spin_controller


class StanleyController(WaypointControllerBase):
    def __init__(self):
        super().__init__('stanley_controller')
        self.declare_parameter('k_heading', 1.2)
        self.declare_parameter('k_cross_track', 1.0)
        self.declare_parameter('kv', 0.6)
        self.declare_parameter('min_speed', 0.08)

        self.k_heading = float(self.get_parameter('k_heading').value)
        self.k_cross_track = float(self.get_parameter('k_cross_track').value)
        self.kv = float(self.get_parameter('kv').value)
        self.min_speed = float(self.get_parameter('min_speed').value)

    def _segment_points(self) -> tuple[np.ndarray, np.ndarray]:
        target_idx = self.current_waypoint_idx
        prev_idx = max(0, target_idx - 1)

        if target_idx == 0:
            # Use current position as a virtual segment start before first waypoint.
            return np.array([self.robot_x, self.robot_y]), self.waypoints[target_idx]

        return self.waypoints[prev_idx], self.waypoints[target_idx]

    def _cross_track_error(self) -> float:
        start, end = self._segment_points()
        seg = end - start
        seg_norm = np.linalg.norm(seg)
        if seg_norm < 1e-6:
            return 0.0

        pos = np.array([self.robot_x, self.robot_y])
        rel = pos - start

        # Signed lateral distance from robot to path segment.
        cross_z = seg[0] * rel[1] - seg[1] * rel[0]
        return float(cross_z / seg_norm)

    def _path_heading(self) -> float:
        start, end = self._segment_points()
        return math.atan2(end[1] - start[1], end[0] - start[0])

    def compute_control(self, target, distance_to_target):
        del target

        # Heading term aligns robot orientation with path tangent.
        path_heading = self._path_heading()
        heading_error = normalize_angle(path_heading - self.robot_yaw)
        # Cross-track term pushes robot back to the path centerline.
        cross_track_error = self._cross_track_error()

        linear_velocity = max(self.min_speed, min(self.max_linear_velocity, self.kv * distance_to_target))
        cte_term = math.atan2(self.k_cross_track * cross_track_error, linear_velocity + 0.1)

        angular_velocity = self.k_heading * heading_error + cte_term

        # Slow down near waypoints to reduce overshoot on tight transitions.
        if distance_to_target < 0.5:
            linear_velocity *= max(0.2, distance_to_target / 0.5)

        return linear_velocity, angular_velocity


def main(args=None):
    rclpy.init(args=args)
    node = StanleyController()
    spin_controller(node)


if __name__ == '__main__':
    main()

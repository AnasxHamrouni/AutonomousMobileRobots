#!/usr/bin/env python3

import math

import rclpy

from my_robot_controllers.controller_base import WaypointControllerBase, normalize_angle, spin_controller


class PurePursuitController(WaypointControllerBase):
    def __init__(self):
        super().__init__('pure_pursuit_controller')
        self.declare_parameter('lookahead_distance', 0.7)
        self.declare_parameter('nominal_linear_velocity', 0.3)

        self.lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        self.nominal_linear_velocity = float(self.get_parameter('nominal_linear_velocity').value)

    def compute_control(self, target, distance_to_target):
        # Heading from current pose to active waypoint.
        desired_heading = self.heading_to_target(target)
        heading_error = normalize_angle(desired_heading - self.robot_yaw)

        # Pure pursuit steering law: curvature is driven by heading error.
        if self.lookahead_distance <= 1e-3:
            angular_velocity = 0.0
        else:
            angular_velocity = (2.0 / self.lookahead_distance) * math.sin(heading_error)

        # Keep linear speed high when aligned, slow down if the robot is pointed away.
        linear_velocity = self.nominal_linear_velocity * max(0.0, math.cos(heading_error))
        # Final approach speed reduction to avoid crossing the waypoint too hard.
        if distance_to_target < 0.5:
            linear_velocity *= distance_to_target / 0.5

        return linear_velocity, angular_velocity


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()
    spin_controller(node)


if __name__ == '__main__':
    main()

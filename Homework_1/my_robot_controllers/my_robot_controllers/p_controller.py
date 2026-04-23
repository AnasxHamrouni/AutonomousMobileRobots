#!/usr/bin/env python3

import rclpy

from my_robot_controllers.controller_base import WaypointControllerBase, normalize_angle, spin_controller


class PController(WaypointControllerBase):
    def __init__(self):
        super().__init__('p_controller')
        self.declare_parameter('kp_linear', 0.7)
        self.declare_parameter('kp_angular', 1.8)
        self.declare_parameter('min_linear_velocity', 0.05)

        self.kp_linear = float(self.get_parameter('kp_linear').value)
        self.kp_angular = float(self.get_parameter('kp_angular').value)
        self.min_linear_velocity = float(self.get_parameter('min_linear_velocity').value)

    def compute_control(self, target, distance_to_target):
        desired_heading = self.heading_to_target(target)
        heading_error = normalize_angle(desired_heading - self.robot_yaw)

        # Basic proportional position control.
        linear_velocity = self.kp_linear * distance_to_target
        linear_velocity = max(self.min_linear_velocity, linear_velocity)

        # If heading error is large, prioritize turning before moving forward fast.
        if abs(heading_error) > 1.0:
            linear_velocity *= 0.3

        # Angular P-control around heading error.
        angular_velocity = self.kp_angular * heading_error
        return linear_velocity, angular_velocity


def main(args=None):
    rclpy.init(args=args)
    node = PController()
    spin_controller(node)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import math

import numpy as np
import rclpy

from my_robot_controllers.controller_base import WaypointControllerBase, normalize_angle, spin_controller


class MPCController(WaypointControllerBase):
    def __init__(self):
        super().__init__('mpc_controller')

        self.declare_parameter('horizon_steps', 15)
        self.declare_parameter('prediction_dt', 0.1)
        self.declare_parameter('num_rollouts', 100)
        self.declare_parameter('w_position', 3.0)
        self.declare_parameter('w_heading', 2.5)
        self.declare_parameter('w_control', 0.05)
        self.declare_parameter('lookahead_ratio', 1.5)

        self.horizon_steps = int(self.get_parameter('horizon_steps').value)
        self.prediction_dt = float(self.get_parameter('prediction_dt').value)
        self.num_rollouts = int(self.get_parameter('num_rollouts').value)

        self.w_position = float(self.get_parameter('w_position').value)
        self.w_heading = float(self.get_parameter('w_heading').value)
        self.w_control = float(self.get_parameter('w_control').value)
        self.lookahead_ratio = float(self.get_parameter('lookahead_ratio').value)

        self.prev_control = np.array([0.0, 0.0], dtype=float)

    @staticmethod
    def _step(state: np.ndarray, control: np.ndarray, dt: float) -> np.ndarray:
        x, y, yaw = state
        v, w = control

        x_next = x + v * math.cos(yaw) * dt
        y_next = y + v * math.sin(yaw) * dt
        yaw_next = normalize_angle(yaw + w * dt)
        return np.array([x_next, y_next, yaw_next], dtype=float)

    def _sample_rollout(self) -> np.ndarray:
        v_center, w_center = self.prev_control

        # Sample around the previous best control to keep behavior smooth.
        v_min = max(0.0, v_center - 0.2)
        v_max = min(self.max_linear_velocity, v_center + 0.2)
        w_min = max(-self.max_angular_velocity, w_center - 0.8)
        w_max = min(self.max_angular_velocity, w_center + 0.8)

        rollout = np.empty((self.horizon_steps, 2), dtype=float)
        rollout[:, 0] = np.random.uniform(v_min, v_max, self.horizon_steps)
        rollout[:, 1] = np.random.uniform(w_min, w_max, self.horizon_steps)
        return rollout

    def _get_lookahead_target(self, current_idx: int, target: np.ndarray) -> np.ndarray:
        """
        Look ahead to next waypoint if current one is close, encouraging sequential progression.
        Prevents the controller from oscillating around the current waypoint.
        """
        distance_to_current = np.linalg.norm(np.array([self.robot_x, self.robot_y]) - target)
        
        # If within lookahead threshold of current waypoint, blend toward next
        if distance_to_current < 0.8 and current_idx + 1 < len(self.waypoints):
            next_target = self.waypoints[current_idx + 1]
            blend = min(1.0, distance_to_current / 0.8)
            # Blend: favor next waypoint as we approach current one
            lookahead = (1.0 - blend) * next_target + blend * target
            return lookahead
        return target

    def _trajectory_cost(self, trajectory: np.ndarray, controls: np.ndarray, target: np.ndarray) -> float:
        final_state = trajectory[-1]

        # Use lookahead target for cost evaluation
        eval_target = self._get_lookahead_target(self.current_waypoint_idx, target)
        
        pos_err = np.linalg.norm(final_state[:2] - eval_target)
        desired_heading = math.atan2(eval_target[1] - final_state[1], eval_target[0] - final_state[0])
        heading_err = abs(normalize_angle(desired_heading - final_state[2]))

        # Penalize aggressive controls to reduce oscillations and zig-zags.
        control_effort = np.sum(np.square(controls[:, 0])) + np.sum(np.square(controls[:, 1]))

        return self.w_position * pos_err + self.w_heading * heading_err + self.w_control * control_effort

    def compute_control(self, target, distance_to_target):
        start_state = np.array([self.robot_x, self.robot_y, self.robot_yaw], dtype=float)

        best_cost = float('inf')
        best_control = np.array([0.0, 0.0], dtype=float)

        for _ in range(self.num_rollouts):
            candidate_controls = self._sample_rollout()
            trajectory = np.empty((self.horizon_steps, 3), dtype=float)

            state = start_state.copy()
            for step in range(self.horizon_steps):
                state = self._step(state, candidate_controls[step], self.prediction_dt)
                trajectory[step] = state

            cost = self._trajectory_cost(trajectory, candidate_controls, target)
            if cost < best_cost:
                best_cost = cost
                # Execute only the first action of the best predicted sequence.
                best_control = candidate_controls[0]

        # Smooth deceleration as we approach waypoint
        if distance_to_target < 1.0:
            slowdown_factor = max(0.1, distance_to_target / 1.0)
            best_control[0] *= slowdown_factor
            # Reduce angular velocity near target to stabilize
            best_control[1] *= slowdown_factor

        self.prev_control = best_control
        return float(best_control[0]), float(best_control[1])


def main(args=None):
    rclpy.init(args=args)
    node = MPCController()
    spin_controller(node)


if __name__ == '__main__':
    main()

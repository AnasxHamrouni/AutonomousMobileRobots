#!/usr/bin/env python3
"""
A* Path Planning Algorithm

Task 3 implementation:
- Subscribes to /map (nav_msgs/OccupancyGrid)
- Subscribes to /goal_pose
- Gets robot start pose from TF (map -> base_link)
- Runs A* search on occupancy grid
- Publishes nav_msgs/Path on /planned_path
"""
import math
import heapq
import os
import csv
import time
from datetime import datetime
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path

import tf2_ros
import tf2_geometry_msgs  # noqa: F401 Registers PoseStamped transform for tf2
from tf2_ros import TransformException


GridCell = Tuple[int, int]


class AStarPlannerNode(Node):
    """
    A* Global Planner Node (Task 3)
    Implements the A* path planning algorithm on a 2D occupancy grid for a differential drive robot in ROS 2.

    Main features:
    - Subscribes to /map (nav_msgs/OccupancyGrid) and /goal_pose (PoseStamped)
    - Gets robot start pose from TF (map -> base_link)
    - Runs A* search on the occupancy grid (with inflation and diagonal options)
    - Publishes nav_msgs/Path on /planned_path

    Key logic:
    - On receiving a goal, transforms start and goal to grid coordinates.
    - Runs A* search, reconstructs the path, and publishes it as a Path message.
    - Handles blocked/unknown cells, map inflation, and diagonal movement.

    Metrics logging:
    - Logs planning statistics (success, time, path length, expanded nodes, waypoints) to CSV for quantitative comparison.

    Parameters:
    - occupancy_threshold: Occupancy value above which a cell is considered blocked.
    - unknown_is_obstacle: Treat unknown cells as obstacles.
    - inflate_radius_m: Inflate obstacles by this radius (meters).
    - allow_diagonal: Allow diagonal movement in A*.
    - scenario_name: Tag for metrics logging.
    """
    def __init__(self):
        super().__init__('a_star_planner_node')

        # Parameters
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('occupancy_threshold', 50)
        self.declare_parameter('unknown_is_obstacle', True)
        self.declare_parameter('inflate_radius_m', 0.12)
        self.declare_parameter('allow_diagonal', True)
        self.declare_parameter('enable_metrics_logging', True)
        self.declare_parameter('metrics_output_dir', 'metrics')
        self.declare_parameter('scenario_name', 'default')

        # Map state
        self.map_msg: Optional[OccupancyGrid] = None
        self.grid_width = 0
        self.grid_height = 0
        self.grid_resolution = 0.0
        self.grid_origin_x = 0.0
        self.grid_origin_y = 0.0
        self.occupancy: List[int] = []
        self.blocked: List[bool] = []
        self.goal_counter = 0
        self.metrics_path: Optional[str] = None

        self._init_metrics_logging()

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ROS I/O
        map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value

        self.map_sub = self.create_subscription(OccupancyGrid, map_topic, self.map_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, goal_topic, self.goal_callback, 10)
        self.path_pub = self.create_publisher(Path, path_topic, 10)

        self.get_logger().info('A* Planner Node Initialized.')

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
        self.metrics_path = os.path.join(output_dir, 'astar_metrics.csv')

        if not os.path.exists(self.metrics_path):
            with open(self.metrics_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'timestamp', 'scenario', 'planner', 'goal_index', 'success', 'reason',
                    'planning_time_ms', 'expanded_nodes', 'path_waypoints', 'path_length_m',
                    'start_x', 'start_y', 'goal_x', 'goal_y',
                    'start_cell_x', 'start_cell_y', 'goal_cell_x', 'goal_cell_y'
                ])

    def _append_metrics(self,
        success: bool,
        reason: str,
        planning_time_ms: float,
        expanded_nodes: int,
        path_waypoints: int,
        path_length_m: float,
        start_world: Optional[Tuple[float, float]],
        goal_world: Optional[Tuple[float, float]],
        start_cell: Optional[GridCell],
        goal_cell: Optional[GridCell],
    ):
        """
        Append a row of planning metrics to the CSV file.
        """
        if self.metrics_path is None:
            return

        scenario_name = self.get_parameter('scenario_name').get_parameter_value().string_value
        row = [
            datetime.now().isoformat(timespec='seconds'),
            scenario_name,
            'astar',
            self.goal_counter,
            int(success),
            reason,
            f"{planning_time_ms:.3f}",
            expanded_nodes,
            path_waypoints,
            f"{path_length_m:.3f}",
            '' if start_world is None else f"{start_world[0]:.3f}",
            '' if start_world is None else f"{start_world[1]:.3f}",
            '' if goal_world is None else f"{goal_world[0]:.3f}",
            '' if goal_world is None else f"{goal_world[1]:.3f}",
            '' if start_cell is None else start_cell[0],
            '' if start_cell is None else start_cell[1],
            '' if goal_cell is None else goal_cell[0],
            '' if goal_cell is None else goal_cell[1],
        ]

        with open(self.metrics_path, 'a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(row)

    def _path_length_world(self, cells: List[GridCell]) -> float:
        """
        Compute the total path length in world coordinates.
        """
        if len(cells) < 2:
            return 0.0

        length = 0.0
        prev_w = self.grid_to_world(cells[0][0], cells[0][1])
        for gx, gy in cells[1:]:
            curr_w = self.grid_to_world(gx, gy)
            dx = curr_w[0] - prev_w[0]
            dy = curr_w[1] - prev_w[1]
            length += math.sqrt(dx * dx + dy * dy)
            prev_w = curr_w
        return length

    def map_callback(self, msg: OccupancyGrid):
        """
        Callback for new occupancy grid map.
        Updates internal grid representation and blocked cells.
        """
        self.map_msg = msg
        self.grid_width = msg.info.width
        self.grid_height = msg.info.height
        self.grid_resolution = msg.info.resolution
        self.grid_origin_x = msg.info.origin.position.x
        self.grid_origin_y = msg.info.origin.position.y
        self.occupancy = list(msg.data)
        self._build_blocked_grid()

    def goal_callback(self, goal_msg: PoseStamped):
        """
        Callback for new goal pose.
        Runs A* search and publishes the planned path.
        """
        self.goal_counter += 1
        t0 = time.perf_counter()

        if self.map_msg is None:
            self.get_logger().warn('No map received yet. Cannot plan.')
            self._append_metrics(False, 'NO_MAP', (time.perf_counter() - t0) * 1000.0, 0, 0, 0.0, None, None, None, None)
            return

        start_world = self._get_start_world_pose()
        if start_world is None:
            self._append_metrics(False, 'NO_START_TF', (time.perf_counter() - t0) * 1000.0, 0, 0, 0.0, None, None, None, None)
            return

        goal_world = self._transform_goal_to_map(goal_msg)
        if goal_world is None:
            self._append_metrics(False, 'GOAL_TF_FAIL', (time.perf_counter() - t0) * 1000.0, 0, 0, 0.0, start_world, None, None, None)
            return

        start_cell = self.world_to_grid(start_world[0], start_world[1])
        goal_cell = self.world_to_grid(goal_world[0], goal_world[1])

        if start_cell is None or goal_cell is None:
            self.get_logger().warn('Start or goal is outside map bounds.')
            self._append_metrics(False, 'OUT_OF_BOUNDS', (time.perf_counter() - t0) * 1000.0, 0, 0, 0.0, start_world, goal_world, start_cell, goal_cell)
            return

        if self.is_blocked(goal_cell[0], goal_cell[1]):
            self.get_logger().warn('Goal cell is blocked. Cannot plan path.')
            self._append_metrics(False, 'GOAL_BLOCKED', (time.perf_counter() - t0) * 1000.0, 0, 0, 0.0, start_world, goal_world, start_cell, goal_cell)
            return

        path_cells, expanded_nodes = self.a_star(start_cell, goal_cell)
        if not path_cells:
            self.get_logger().warn('A* failed: no path found.')
            self._append_metrics(False, 'NO_PATH', (time.perf_counter() - t0) * 1000.0, expanded_nodes, 0, 0.0, start_world, goal_world, start_cell, goal_cell)
            return

        path_msg = self.cells_to_path(path_cells)
        self.path_pub.publish(path_msg)
        planning_time_ms = (time.perf_counter() - t0) * 1000.0
        path_length_m = self._path_length_world(path_cells)
        self._append_metrics(True, 'OK', planning_time_ms, expanded_nodes, len(path_cells), path_length_m, start_world, goal_world, start_cell, goal_cell)
        self.get_logger().info(f'A* path published with {len(path_cells)} waypoints.')

    def _build_blocked_grid(self):
        """
        Build the blocked cell grid from occupancy values and inflate obstacles.
        """
        occ_thr = self.get_parameter('occupancy_threshold').get_parameter_value().integer_value
        unknown_is_obstacle = self.get_parameter('unknown_is_obstacle').get_parameter_value().bool_value
        inflate_radius_m = self.get_parameter('inflate_radius_m').get_parameter_value().double_value

        n = self.grid_width * self.grid_height
        self.blocked = [False] * n

        for i, v in enumerate(self.occupancy):
            if v < 0:
                self.blocked[i] = unknown_is_obstacle
            else:
                self.blocked[i] = v >= occ_thr

        if self.grid_resolution <= 0.0:
            return

        inflate_cells = int(math.ceil(inflate_radius_m / self.grid_resolution))
        if inflate_cells <= 0:
            return

        original = self.blocked[:]
        for y in range(self.grid_height):
            for x in range(self.grid_width):
                idx = self.to_index(x, y)
                if not original[idx]:
                    continue
                for dy in range(-inflate_cells, inflate_cells + 1):
                    for dx in range(-inflate_cells, inflate_cells + 1):
                        nx = x + dx
                        ny = y + dy
                        if not self.in_bounds(nx, ny):
                            continue
                        if dx * dx + dy * dy <= inflate_cells * inflate_cells:
                            self.blocked[self.to_index(nx, ny)] = True

    def _get_start_world_pose(self) -> Optional[Tuple[float, float]]:
        """
        Get the robot's current pose in the map frame using TF.
        """
        map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        try:
            transform = self.tf_buffer.lookup_transform(
                map_frame,
                base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
            t = transform.transform.translation
            return t.x, t.y
        except TransformException as e:
            self.get_logger().warn(f'Could not get start pose from TF: {e}')
            return None

    def _transform_goal_to_map(self, goal_msg: PoseStamped) -> Optional[Tuple[float, float]]:
        """
        Transform the goal pose to the map frame using TF.
        """
        map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        try:
            goal_copy = PoseStamped()
            goal_copy.header = goal_msg.header
            goal_copy.pose = goal_msg.pose
            goal_copy.header.stamp.sec = 0
            goal_copy.header.stamp.nanosec = 0

            transformed = self.tf_buffer.transform(
                goal_copy,
                map_frame,
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
            p = transformed.pose.position
            return p.x, p.y
        except TransformException as e:
            self.get_logger().warn(f'Could not transform goal to map frame: {e}')
            return None

    def in_bounds(self, x: int, y: int) -> bool:
        """
        Check if grid cell (x, y) is within map bounds.
        """
        return 0 <= x < self.grid_width and 0 <= y < self.grid_height

    def to_index(self, x: int, y: int) -> int:
        """
        Convert grid cell (x, y) to flat index.
        """
        return y * self.grid_width + x

    def is_blocked(self, x: int, y: int) -> bool:
        """
        Return True if cell (x, y) is blocked.
        """
        if not self.in_bounds(x, y):
            return True
        return self.blocked[self.to_index(x, y)]

    def world_to_grid(self, wx: float, wy: float) -> Optional[GridCell]:
        """
        Convert world coordinates (wx, wy) to grid cell indices.
        """
        gx = int((wx - self.grid_origin_x) / self.grid_resolution)
        gy = int((wy - self.grid_origin_y) / self.grid_resolution)
        if not self.in_bounds(gx, gy):
            return None
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        """
        Convert grid cell indices to world coordinates (center of cell).
        """
        wx = self.grid_origin_x + (gx + 0.5) * self.grid_resolution
        wy = self.grid_origin_y + (gy + 0.5) * self.grid_resolution
        return wx, wy

    def heuristic(self, a: GridCell, b: GridCell) -> float:
        """
        Heuristic function for A* (Euclidean distance).
        """
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return math.sqrt(dx * dx + dy * dy)

    def neighbors(self, x: int, y: int) -> List[Tuple[GridCell, float]]:
        """
        Get valid neighbor cells for A* search (with/without diagonal moves).
        """
        allow_diagonal = self.get_parameter('allow_diagonal').get_parameter_value().bool_value
        nbrs: List[Tuple[GridCell, float]] = []

        steps = [
            (1, 0, 1.0),
            (-1, 0, 1.0),
            (0, 1, 1.0),
            (0, -1, 1.0),
        ]
        if allow_diagonal:
            diag = math.sqrt(2.0)
            steps.extend([
                (1, 1, diag),
                (1, -1, diag),
                (-1, 1, diag),
                (-1, -1, diag),
            ])

        for dx, dy, cost in steps:
            nx = x + dx
            ny = y + dy
            if not self.in_bounds(nx, ny):
                continue
            if self.is_blocked(nx, ny):
                continue

            # Avoid diagonal corner cutting
            if dx != 0 and dy != 0:
                if self.is_blocked(x + dx, y) or self.is_blocked(x, y + dy):
                    continue

            nbrs.append(((nx, ny), cost))

        return nbrs

    def reconstruct_path(self, came_from: Dict[GridCell, GridCell], current: GridCell) -> List[GridCell]:
        """
        Reconstruct the path from start to goal using the parent map.
        """
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def a_star(self, start: GridCell, goal: GridCell) -> Tuple[List[GridCell], int]:
        """
        Run the A* search algorithm from start to goal.
        Returns the path and number of expanded nodes.
        """
        open_heap: List[Tuple[float, GridCell]] = []
        heapq.heappush(open_heap, (0.0, start))

        came_from: Dict[GridCell, GridCell] = {}
        g_score: Dict[GridCell, float] = {start: 0.0}
        closed = set()
        expanded_nodes = 0

        while open_heap:
            _, current = heapq.heappop(open_heap)
            if current in closed:
                continue

            if current == goal:
                return self.reconstruct_path(came_from, current), expanded_nodes

            closed.add(current)
            expanded_nodes += 1
            cx, cy = current

            for (nx, ny), step_cost in self.neighbors(cx, cy):
                neighbor = (nx, ny)
                if neighbor in closed:
                    continue

                tentative_g = g_score[current] + step_cost
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_heap, (f, neighbor))

        return [], expanded_nodes

    def cells_to_path(self, cells: List[GridCell]) -> Path:
        """
        Convert a list of grid cells to a nav_msgs/Path message.
        """
        path = Path()
        map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        path.header.frame_id = map_frame
        path.header.stamp = self.get_clock().now().to_msg()

        for i, (gx, gy) in enumerate(cells):
            wx, wy = self.grid_to_world(gx, gy)
            pose = PoseStamped()
            pose.header.frame_id = map_frame
            pose.header.stamp = path.header.stamp
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = 0.0

            if i < len(cells) - 1:
                nx, ny = cells[i + 1]
                nwx, nwy = self.grid_to_world(nx, ny)
                yaw = math.atan2(nwy - wy, nwx - wx)
            elif i > 0:
                px, py = cells[i - 1]
                pwx, pwy = self.grid_to_world(px, py)
                yaw = math.atan2(wy - pwy, wx - pwx)
            else:
                yaw = 0.0

            pose.pose.orientation.z = math.sin(yaw * 0.5)
            pose.pose.orientation.w = math.cos(yaw * 0.5)
            path.poses.append(pose)

        return path


def main(args=None):
    """
    Main entry point for A* planner node.
    """
    rclpy.init(args=args)
    node = AStarPlannerNode()
    try:
        rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        node.get_logger().info('A* planner stopped')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

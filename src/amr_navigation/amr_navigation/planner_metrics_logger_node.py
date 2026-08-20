#!/usr/bin/env python3
"""
Planner Metrics & Clearance Logger Node (amr_navigation)
Captures live telemetry from Nav2 SmacPlanner2D path planning across the fleet.

Computes:
- Path Length (m)
- Waypoint Count
- Minimum Obstacle Clearance (m)
- Average Obstacle Clearance (m)
- Bottleneck Segment Ratio (< 0.75m clearance)
- Costmap Sampled Cost Profile (mean, max)
- Straightness Index (Euclidean displacement / Path length)

Appends records to a timestamped CSV file in the workspace log directory
and prints a real-time console telemetry card on every newly computed plan.
"""

import os
import csv
import math
import numpy as np
from typing import Optional, Dict
from scipy.ndimage import distance_transform_edt

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Path, OccupancyGrid


class PlannerMetricsLoggerNode(Node):
    def __init__(self):
        super().__init__('planner_metrics_logger')

        # Parameters
        self.declare_parameter('log_dir', os.path.join(os.getcwd(), 'log'))
        self.declare_parameter('csv_filename', 'planner_metrics.csv')
        self.declare_parameter('bottleneck_threshold_m', 0.75)
        self.declare_parameter('robot_names', ['bcr_bot_amr1', 'bcr_bot_amr2'])

        self.log_dir = str(self.get_parameter('log_dir').value)
        self.csv_filename = str(self.get_parameter('csv_filename').value)
        self.bottleneck_thresh = float(self.get_parameter('bottleneck_threshold_m').value)
        self.robot_names = list(self.get_parameter('robot_names').value)

        os.makedirs(self.log_dir, exist_ok=True)
        self.csv_path = os.path.join(self.log_dir, self.csv_filename)

        # Initialize CSV header if file does not exist
        if not os.path.exists(self.csv_path) or os.path.getsize(self.csv_path) == 0:
            with open(self.csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'timestamp_sim',
                    'robot',
                    'path_length_m',
                    'waypoint_count',
                    'min_clearance_m',
                    'avg_clearance_m',
                    'bottleneck_pct',
                    'mean_cost',
                    'peak_cost',
                    'straightness_index'
                ])

        # State storage for distance maps and costmaps
        self.dist_map: Optional[np.ndarray] = None
        self.map_info: Optional[OccupancyGrid] = None
        self.last_path_lengths: Dict[str, float] = {}

        # QoS for static/transient local map
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # Subscribe to fused world map
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/fleet/merged_map',
            self.map_callback,
            map_qos
        )

        # Subscribe to path plans for each robot
        self.path_subs = []
        for r_name in self.robot_names:
            sub = self.create_subscription(
                Path,
                f'/{r_name}/plan',
                lambda msg, name=r_name: self.plan_callback(msg, name),
                10
            )
            self.path_subs.append(sub)

        self.get_logger().info(
            f'[PLANNER_LOGGER] Initialized. Monitoring {self.robot_names}. '
            f'Writing telemetry to: {self.csv_path}'
        )

    def map_callback(self, msg: OccupancyGrid):
        """Updates distance field to obstacles whenever map updates."""
        w = msg.info.width
        h = msg.info.height
        res = msg.info.resolution
        if w == 0 or h == 0 or res <= 0.0:
            return

        raw_data = np.array(msg.data, dtype=np.int8).reshape((h, w))
        # Obstacles are cells with cost >= 65
        obstacle_mask = (raw_data >= 65)
        # Distance transform computes distance from non-obstacle to nearest obstacle in pixels
        non_obstacle_mask = ~obstacle_mask
        dist_pixels = distance_transform_edt(non_obstacle_mask)
        self.dist_map = dist_pixels * res  # in meters
        self.map_info = msg

    def plan_callback(self, msg: Path, robot_name: str):
        """Analyzes path clearance, length, and costs, then logs to CSV."""
        if not msg.poses or len(msg.poses) < 2:
            return

        # Calculate path length and waypoints
        poses = msg.poses
        w_count = len(poses)
        coords = []
        path_len = 0.0

        for i in range(w_count):
            p = poses[i].pose.position
            coords.append((p.x, p.y))
            if i > 0:
                dx = p.x - poses[i - 1].pose.position.x
                dy = p.y - poses[i - 1].pose.position.y
                path_len += math.hypot(dx, dy)

        # Ignore tiny duplicates / unchanged replans
        prev_len = self.last_path_lengths.get(robot_name, 0.0)
        if abs(path_len - prev_len) < 0.05 and path_len > 0.0:
            return
        self.last_path_lengths[robot_name] = path_len

        # Straight-line distance
        dx_tot = coords[-1][0] - coords[0][0]
        dy_tot = coords[-1][1] - coords[0][1]
        straight_dist = math.hypot(dx_tot, dy_tot)
        straightness = straight_dist / path_len if path_len > 0 else 1.0

        # Obstacle clearance extraction
        clearances = []
        sampled_costs = []
        bottleneck_count = 0

        if self.dist_map is not None and self.map_info is not None:
            res = self.map_info.info.resolution
            ox = self.map_info.info.origin.position.x
            oy = self.map_info.info.origin.position.y
            mw = self.map_info.info.width
            mh = self.map_info.info.height
            raw_data = np.array(self.map_info.data, dtype=np.int8).reshape((mh, mw))

            for wx, wy in coords:
                gx = int((wx - ox) / res)
                gy = int((wy - oy) / res)
                if 0 <= gx < mw and 0 <= gy < mh:
                    c_dist = float(self.dist_map[gy, gx])
                    clearances.append(c_dist)
                    c_val = int(raw_data[gy, gx])
                    if c_val >= 0:
                        sampled_costs.append(c_val)
                    if c_dist < self.bottleneck_thresh:
                        bottleneck_count += 1
                else:
                    clearances.append(2.0)  # Open space outside grid

        min_c = float(np.min(clearances)) if clearances else 1.5
        avg_c = float(np.mean(clearances)) if clearances else 1.5
        bottleneck_pct = (bottleneck_count / float(w_count)) * 100.0 if w_count > 0 else 0.0
        mean_cost = float(np.mean(sampled_costs)) if sampled_costs else 0.0
        peak_cost = float(np.max(sampled_costs)) if sampled_costs else 0.0

        sim_time = self.get_clock().now().nanoseconds / 1e9

        # Append to CSV
        try:
            with open(self.csv_path, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    f'{sim_time:.2f}',
                    robot_name,
                    f'{path_len:.2f}',
                    w_count,
                    f'{min_c:.2f}',
                    f'{avg_c:.2f}',
                    f'{bottleneck_pct:.1f}',
                    f'{mean_cost:.1f}',
                    f'{peak_cost:.1f}',
                    f'{straightness:.2f}'
                ])
        except Exception as e:
            self.get_logger().error(f'Failed to write metrics CSV: {e}')

        # Formatted console telemetry log
        self.get_logger().info(
            f'[{robot_name.upper()} PLANNER_TELEMETRY] '
            f'Length={path_len:.2f}m | '
            f'MinClearance={min_c:.2f}m | '
            f'AvgClearance={avg_c:.2f}m | '
            f'Bottlenecks={bottleneck_pct:.1f}% | '
            f'MeanCost={mean_cost:.1f} | '
            f'Straightness={straightness:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PlannerMetricsLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

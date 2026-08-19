#!/usr/bin/env python3

"""map_fusion_node

A custom ROS2 node that fuses the OccupancyGrid maps from two independent
robots (AMR-1 and AMR-2) into a single global map while applying a selective
"frontier‑aware" throttling policy to the contribution from AMR‑1.

Key features:
- Uses static transforms (world → robot map) defined at launch time.
- Maintains a per‑cell visit‑density counter for AMR‑1.
- Detects frontier cells (cells adjacent to unknown space) and always
  propagates those updates regardless of visit count.
- Publishes the fused map on ``/fleet/merged_map`` (frame ``world``).
- Publishes a diagnostic ``/fleet/amr1_selective_stats`` message containing
  counts of total, updated, skipped, and frontier cells per merge cycle.

The node is deliberately lightweight (Python + NumPy) and does not interfere
with the robots' local SLAM or Nav2 stacks – it operates entirely downstream
of the per‑robot ``/map`` topics.
"""

import math
from collections import namedtuple
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header, String

CellStats = namedtuple("CellStats", ["total", "updated", "skipped", "frontier"])


class MapFusionNode(Node):
    def __init__(self):
        super().__init__("map_fusion_node")

        # Parameters (override via launch if needed)
        self.declare_parameter("amr1_spawn_x", 0.0)
        self.declare_parameter("amr1_spawn_y", 0.0)
        self.declare_parameter("amr1_spawn_yaw", 0.0)
        self.declare_parameter("amr2_spawn_x", 4.0)
        self.declare_parameter("amr2_spawn_y", 0.0)
        self.declare_parameter("amr2_spawn_yaw", 0.0)
        self.declare_parameter("visit_threshold", 3)
        self.declare_parameter("merge_rate_hz", 1.0)
        self.declare_parameter("world_frame_id", "world")
        self.declare_parameter("debug", False)

        # Custom Ramp / Slope traversability parameters
        self.declare_parameter("ramp_min_x", -4.15)
        self.declare_parameter("ramp_max_x", -2.65)
        self.declare_parameter("ramp_min_y", -4.05)
        self.declare_parameter("ramp_max_y", 4.05)
        self.declare_parameter("ramp_slope_cost", 30)  # Traversable cost penalty (0-100)

        self.ramp_min_x = self.get_parameter("ramp_min_x").get_parameter_value().double_value
        self.ramp_max_x = self.get_parameter("ramp_max_x").get_parameter_value().double_value
        self.ramp_min_y = self.get_parameter("ramp_min_y").get_parameter_value().double_value
        self.ramp_max_y = self.get_parameter("ramp_max_y").get_parameter_value().double_value
        self.ramp_slope_cost = int(self.get_parameter("ramp_slope_cost").get_parameter_value().integer_value)

        self.amr1_spawn = (
            self.get_parameter("amr1_spawn_x").get_parameter_value().double_value,
            self.get_parameter("amr1_spawn_y").get_parameter_value().double_value,
            self.get_parameter("amr1_spawn_yaw").get_parameter_value().double_value,
        )
        self.amr2_spawn = (
            self.get_parameter("amr2_spawn_x").get_parameter_value().double_value,
            self.get_parameter("amr2_spawn_y").get_parameter_value().double_value,
            self.get_parameter("amr2_spawn_yaw").get_parameter_value().double_value,
        )
        self.visit_threshold = self.get_parameter("visit_threshold").get_parameter_value().integer_value
        self.merge_rate_hz = self.get_parameter("merge_rate_hz").get_parameter_value().double_value
        self.world_frame = self.get_parameter("world_frame_id").get_parameter_value().string_value
        self.debug = self.get_parameter("debug").get_parameter_value().bool_value

        # QoS: transient local so late subscribers still receive the latest map
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.amr1_map: Optional[OccupancyGrid] = None
        self.amr2_map: Optional[OccupancyGrid] = None
        self.create_subscription(OccupancyGrid, "/bcr_bot_amr1/map", self.amr1_cb, qos)
        self.create_subscription(OccupancyGrid, "/bcr_bot_amr2/map", self.amr2_cb, qos)

        self.merged_pub = self.create_publisher(OccupancyGrid, "/fleet/merged_map", qos)
        self.stats_pub = self.create_publisher(String, "/fleet/amr1_selective_stats", 10)

        # Internal state for selective filtering
        self.world_resolution: Optional[float] = None
        self.world_origin: Optional[np.ndarray] = None
        self.world_width: Optional[int] = None
        self.world_height: Optional[int] = None
        self.visit_density: Optional[np.ndarray] = None
        self.prev_amr1_cells: Optional[np.ndarray] = None

        self.timer = self.create_timer(1.0 / self.merge_rate_hz, self.merge_maps)
        self.get_logger().info("MapFusionNode ready – awaiting map topics")

    # ---------------------------------------------------------------
    def amr1_cb(self, msg: OccupancyGrid):
        self.amr1_map = msg
        if self.debug:
            self.get_logger().debug(f"AMR1 map received (stamp {msg.header.stamp.sec})")

    def amr2_cb(self, msg: OccupancyGrid):
        self.amr2_map = msg
        if self.debug:
            self.get_logger().debug(f"AMR2 map received (stamp {msg.header.stamp.sec})")

    # ---------------------------------------------------------------
    def _get_map_bounds(self, m: OccupancyGrid, spawn: tuple):
        ox = spawn[0] + m.info.origin.position.x
        oy = spawn[1] + m.info.origin.position.y
        w = m.info.width * m.info.resolution
        h = m.info.height * m.info.resolution
        return ox, oy, ox + w, oy + h

    def _init_world_grid(self) -> None:
        maps = []
        if self.amr1_map is not None:
            maps.append((self.amr1_map, self.amr1_spawn))
        if self.amr2_map is not None:
            maps.append((self.amr2_map, self.amr2_spawn))
        if not maps:
            return

        self.world_resolution = maps[0][0].info.resolution

        b_list = [self._get_map_bounds(m, sp) for m, sp in maps]
        min_x = min(b[0] for b in b_list)
        min_y = min(b[1] for b in b_list)
        max_x = max(b[2] for b in b_list)
        max_y = max(b[3] for b in b_list)

        self.world_origin = np.array([min_x, min_y], dtype=float)
        self.world_width = max(1, int(math.ceil((max_x - min_x) / self.world_resolution)))
        self.world_height = max(1, int(math.ceil((max_y - min_y) / self.world_resolution)))

        if self.visit_density is None or self.visit_density.shape != (self.world_height, self.world_width):
            self.visit_density = np.zeros((self.world_height, self.world_width), dtype=np.uint16)

    def merge_maps(self) -> None:
        if self.amr1_map is None and self.amr2_map is None:
            return

        self._init_world_grid()
        if self.world_origin is None or self.world_resolution is None:
            return

        merged = np.full((self.world_height, self.world_width), -1, dtype=np.int8)
        updated = 0
        skipped = 0
        frontier = 0

        min_x, min_y = self.world_origin[0], self.world_origin[1]
        res = self.world_resolution

        # 1. Merge AMR-1 map if available
        if self.amr1_map is not None:
            m1 = self.amr1_map
            h1, w1 = m1.info.height, m1.info.width
            if h1 > 0 and w1 > 0 and len(m1.data) == h1 * w1:
                ox1 = self.amr1_spawn[0] + m1.info.origin.position.x
                oy1 = self.amr1_spawn[1] + m1.info.origin.position.y
                c1 = int(round((ox1 - min_x) / res))
                r1 = int(round((oy1 - min_y) / res))

                m1_arr = np.array(m1.data, dtype=np.int8).reshape(h1, w1)
                known1 = (m1_arr >= 0)
                is_unknown = (m1_arr == -1)

                # Vectorized 4-neighbor frontier detection
                frontier_amr1 = known1 & (
                    np.pad(is_unknown[1:, :], ((0, 1), (0, 0)), constant_values=False) |
                    np.pad(is_unknown[:-1, :], ((1, 0), (0, 0)), constant_values=False) |
                    np.pad(is_unknown[:, 1:], ((0, 0), (0, 1)), constant_values=False) |
                    np.pad(is_unknown[:, :-1], ((0, 0), (1, 0)), constant_values=False)
                )

                # Clamp indices to world grid bounds
                r_end = min(self.world_height, r1 + h1)
                c_end = min(self.world_width, c1 + w1)
                r_start = max(0, r1)
                c_start = max(0, c1)

                src_r_start = r_start - r1
                src_r_end = src_r_start + (r_end - r_start)
                src_c_start = c_start - c1
                src_c_end = src_c_start + (c_end - c_start)

                if r_end > r_start and c_end > c_start:
                    m1_sub = m1_arr[src_r_start:src_r_end, src_c_start:src_c_end]
                    known_sub = known1[src_r_start:src_r_end, src_c_start:src_c_end]
                    front_sub = frontier_amr1[src_r_start:src_r_end, src_c_start:src_c_end]

                    self.visit_density[r_start:r_end, c_start:c_end][known_sub] += 1
                    dens_sub = self.visit_density[r_start:r_end, c_start:c_end]

                    use_sub = known_sub & (front_sub | (dens_sub <= self.visit_threshold))
                    skipped += int(np.count_nonzero(known_sub & (~use_sub)))
                    frontier += int(np.count_nonzero(front_sub))

                    merged_slice = merged[r_start:r_end, c_start:c_end]
                    merged_slice[use_sub] = m1_sub[use_sub]

        # 2. Merge AMR-2 map if available
        if self.amr2_map is not None:
            m2 = self.amr2_map
            h2, w2 = m2.info.height, m2.info.width
            if h2 > 0 and w2 > 0 and len(m2.data) == h2 * w2:
                ox2 = self.amr2_spawn[0] + m2.info.origin.position.x
                oy2 = self.amr2_spawn[1] + m2.info.origin.position.y
                c2 = int(round((ox2 - min_x) / res))
                r2 = int(round((oy2 - min_y) / res))

                m2_arr = np.array(m2.data, dtype=np.int8).reshape(h2, w2)
                known2 = (m2_arr >= 0)

                r_end = min(self.world_height, r2 + h2)
                c_end = min(self.world_width, c2 + w2)
                r_start = max(0, r2)
                c_start = max(0, c2)

                src_r_start = r_start - r2
                src_r_end = src_r_start + (r_end - r_start)
                src_c_start = c_start - c2
                src_c_end = src_c_start + (c_end - c_start)

                if r_end > r_start and c_end > c_start:
                    m2_sub = m2_arr[src_r_start:src_r_end, src_c_start:src_c_end]
                    known_sub = known2[src_r_start:src_r_end, src_c_start:src_c_end]

                    merged_slice = merged[r_start:r_end, c_start:c_end]
                    merged_slice[known_sub] = np.maximum(merged_slice[known_sub], m2_sub[known_sub])

        # 3. Ramp / Slope Traversability Check
        rx_min_col = max(0, int(math.floor((self.ramp_min_x - min_x) / res)))
        rx_max_col = min(self.world_width, int(math.ceil((self.ramp_max_x - min_x) / res)))
        ry_min_row = max(0, int(math.floor((self.ramp_min_y - min_y) / res)))
        ry_max_row = min(self.world_height, int(math.ceil((self.ramp_max_y - min_y) / res)))

        if rx_max_col > rx_min_col and ry_max_row > ry_min_row:
            ramp_slice = merged[ry_min_row:ry_max_row, rx_min_col:rx_max_col]
            ramp_mask = (ramp_slice != -1)
            ramp_slice[ramp_mask] = self.ramp_slope_cost

        updated = int(np.count_nonzero(merged != -1))

        # Publish merged map
        out = OccupancyGrid()
        out.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.world_frame)
        out.info.resolution = float(self.world_resolution)
        out.info.width = int(self.world_width)
        out.info.height = int(self.world_height)
        out.info.origin.position.x = float(self.world_origin[0])
        out.info.origin.position.y = float(self.world_origin[1])
        out.info.origin.position.z = 0.0
        out.info.origin.orientation.w = 1.0
        out.data = merged.flatten().tolist()
        self.merged_pub.publish(out)

        # Publish stats
        stats_msg = String()
        stats_msg.data = f"total:{self.world_width*self.world_height} updated:{updated} " \
                         f"skipped:{skipped} frontier:{frontier}"
        self.stats_pub.publish(stats_msg)

        if self.debug:
            self.get_logger().info(f"Fusion cycle – upd:{updated} skip:{skipped} frontier:{frontier}")


def main(args=None):
    rclpy.init(args=args)
    node = MapFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

if __name__ == "__main__":
    main()

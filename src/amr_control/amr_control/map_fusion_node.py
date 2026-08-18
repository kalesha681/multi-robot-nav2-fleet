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
    def _init_world_grid(self, m1: OccupancyGrid, m2: OccupancyGrid) -> None:
        res1, res2 = m1.info.resolution, m2.info.resolution
        if not math.isclose(res1, res2, rel_tol=1e-6):
            raise RuntimeError("Map resolutions differ – fusion requires identical resolution.")
        self.world_resolution = res1

        def bounds(m: OccupancyGrid):
            ox = m.info.origin.position.x
            oy = m.info.origin.position.y
            w = m.info.width * m.info.resolution
            h = m.info.height * m.info.resolution
            return ox, oy, ox + w, oy + h

        min_x = min(bounds(m1)[0], bounds(m2)[0])
        min_y = min(bounds(m1)[1], bounds(m2)[1])
        max_x = max(bounds(m1)[2], bounds(m2)[2])
        max_y = max(bounds(m1)[3], bounds(m2)[3])
        self.world_origin = np.array([min_x, min_y], dtype=float)
        self.world_width = int(math.ceil((max_x - min_x) / self.world_resolution))
        self.world_height = int(math.ceil((max_y - min_y) / self.world_resolution))
        self.get_logger().info(
            f"World grid: origin ({min_x:.2f},{min_y:.2f}), size {self.world_width}x{self.world_height}")
        self.visit_density = np.zeros((self.world_height, self.world_width), dtype=np.uint16)
        self.prev_amr1_cells = np.full((self.world_height, self.world_width), -2, dtype=np.int8)

    def _world_to_index(self, world_pt: np.ndarray, map_msg: OccupancyGrid) -> Optional[np.ndarray]:
        ox = map_msg.info.origin.position.x
        oy = map_msg.info.origin.position.y
        res = map_msg.info.resolution
        col = int(math.floor((world_pt[0] - ox) / res))
        row = int(math.floor((world_pt[1] - oy) / res))
        if 0 <= col < map_msg.info.width and 0 <= row < map_msg.info.height:
            return np.array([row, col], dtype=int)
        return None

    def _sample(self, map_msg: OccupancyGrid, world_pt: np.ndarray) -> int:
        idx = self._world_to_index(world_pt, map_msg)
        if idx is None:
            return -1
        row, col = idx
        flat = row * map_msg.info.width + col
        return int(map_msg.data[flat])

    # ---------------------------------------------------------------
    def merge_maps(self) -> None:
        if self.amr1_map is None or self.amr2_map is None:
            return
        if self.world_resolution is None:
            self._init_world_grid(self.amr1_map, self.amr2_map)

        merged = np.full((self.world_height, self.world_width), -1, dtype=np.int8)
        updated = 0
        skipped = 0
        frontier = 0

        # Pre‑compute world coordinates for each cell centre (2‑D grids)
        grid_x = self.world_origin[0] + (np.arange(self.world_width) + 0.5) * self.world_resolution
        grid_y = self.world_origin[1] + (np.arange(self.world_height) + 0.5) * self.world_resolution
        wx, wy = np.meshgrid(grid_x, grid_y, indexing='xy')
        # Iterate – readability over vectorisation for now
        for r in range(self.world_height):
            for c in range(self.world_width):
                pt = np.array([wx[r, c], wy[r, c]])
                v1 = self._sample(self.amr1_map, pt)
                v2 = self._sample(self.amr2_map, pt)

                # ---------- selective filter for AMR‑1 ----------
                use_amr1 = True
                if v1 != -1:
                    # frontier check (4‑neighbour unknown in AMR‑1)
                    is_frontier = False
                    for off in [(self.world_resolution, 0), (-self.world_resolution, 0),
                                (0, self.world_resolution), (0, -self.world_resolution)]:
                        npt = pt + np.array(off)
                        if self._sample(self.amr1_map, npt) == -1:
                            is_frontier = True
                            break
                    # update visit density on every valid reading
                    self.visit_density[r, c] += 1
                    self.prev_amr1_cells[r, c] = v1
                    # apply rule
                    if not is_frontier and self.visit_density[r, c] >= self.visit_threshold:
                        use_amr1 = False
                        skipped += 1
                    else:
                        if is_frontier:
                            frontier += 1
                # ---------- merging ----------
                merged_val = -1
                if v1 != -1 and use_amr1:
                    merged_val = v1
                if v2 != -1:
                    merged_val = v2 if merged_val == -1 else max(merged_val, v2)
                merged[r, c] = merged_val
                if merged_val != -1:
                    updated += 1

        # Publish merged map
        out = OccupancyGrid()
        out.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.world_frame)
        out.info.resolution = self.world_resolution
        out.info.width = self.world_width
        out.info.height = self.world_height
        out.info.origin.position.x = self.world_origin[0]
        out.info.origin.position.y = self.world_origin[1]
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
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

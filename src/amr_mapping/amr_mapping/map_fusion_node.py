#!/usr/bin/env python3

"""map_fusion_node

A custom ROS2 node that fuses the OccupancyGrid maps from two independent
robots (AMR-1 and AMR-2) into a single global map while applying a selective
"frontier‑aware" throttling policy to the contribution from AMR‑1.

Key features:
- Uses continuous sub-pixel affine transformations (cv2.warpAffine) for pixel-perfect alignment.
- Dynamically resolves TF transforms (world → robot map) with fallback to launch parameters.
- Clears dynamic inter-robot footprints so static maps never contain lethal start points.
- Maintains a per‑cell visit‑density counter for AMR‑1.
- Detects frontier cells (cells adjacent to unknown space) and always
  propagates those updates regardless of visit count.
- Publishes the fused map on ``/fleet/merged_map`` (frame ``world``).
- Publishes a diagnostic ``/fleet/amr1_selective_stats`` message containing
  counts of total, updated, skipped, and frontier cells per merge cycle.
"""

import math
from collections import namedtuple
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header, String
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

CellStats = namedtuple("CellStats", ["total", "updated", "skipped", "frontier"])


class MapFusionNode(Node):
    def __init__(self):
        super().__init__("map_fusion_node")

        # Parameters (override via launch if needed)
        self.declare_parameter("amr1_spawn_x", 0.0)
        self.declare_parameter("amr1_spawn_y", 0.0)
        self.declare_parameter("amr1_spawn_yaw", 0.0)
        self.declare_parameter("amr2_spawn_x", 2.0)
        self.declare_parameter("amr2_spawn_y", 0.0)
        self.declare_parameter("amr2_spawn_yaw", 0.0)
        self.declare_parameter("visit_threshold", 3)
        self.declare_parameter("merge_rate_hz", 1.0)
        self.declare_parameter("world_frame_id", "world")
        self.declare_parameter("robot_clear_radius", 0.65)
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
        self.robot_clear_radius = self.get_parameter("robot_clear_radius").get_parameter_value().double_value

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

        # TF2 listener for dynamic frame resolution
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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
    def _get_transform(self, frame_name: str, fallback_spawn: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """Looks up world -> frame_name transform from TF, falls back to default parameters."""
        try:
            if self.tf_buffer.can_transform(self.world_frame, frame_name, rclpy.time.Time()):
                tf_msg = self.tf_buffer.lookup_transform(self.world_frame, frame_name, rclpy.time.Time())
                tx = tf_msg.transform.translation.x
                ty = tf_msg.transform.translation.y
                q = tf_msg.transform.rotation
                siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                yaw = math.atan2(siny_cosp, cosy_cosp)
                return tx, ty, yaw
        except Exception:
            pass
        return fallback_spawn[0], fallback_spawn[1], fallback_spawn[2]

    def _get_robot_pose_world(self, robot_ns: str, fallback_spawn: Tuple[float, float, float]) -> Tuple[float, float]:
        """Looks up world -> <robot_ns>/base_footprint position from TF, falls back to spawn coords."""
        try:
            if self.tf_buffer.can_transform(self.world_frame, f"{robot_ns}/base_footprint", rclpy.time.Time()):
                tf_msg = self.tf_buffer.lookup_transform(self.world_frame, f"{robot_ns}/base_footprint", rclpy.time.Time())
                return tf_msg.transform.translation.x, tf_msg.transform.translation.y
        except Exception:
            pass
        return fallback_spawn[0], fallback_spawn[1]

    def _get_map_corners_world(self, m: OccupancyGrid, tx: float, ty: float, yaw: float) -> np.ndarray:
        w = m.info.width * m.info.resolution
        h = m.info.height * m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y

        corners_local = np.array([
            [ox, oy],
            [ox + w, oy],
            [ox, oy + h],
            [ox + w, oy + h]
        ], dtype=np.float64)

        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        rot_mat = np.array([[cos_y, -sin_y], [sin_y, cos_y]], dtype=np.float64)
        corners_world = (rot_mat @ corners_local.T).T + np.array([tx, ty], dtype=np.float64)
        return corners_world

    def _make_affine_matrix(self, ox: float, oy: float, res_m: float,
                            tx: float, ty: float, yaw: float,
                            min_x: float, min_y: float, res_w: float) -> np.ndarray:
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        T_map_pixel = np.array([
            [res_m, 0.0, ox],
            [0.0, res_m, oy],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

        T_world_map = np.array([
            [cos_y, -sin_y, tx],
            [sin_y,  cos_y, ty],
            [0.0,    0.0,   1.0]
        ], dtype=np.float64)

        T_world_pixel = np.array([
            [1.0 / res_w, 0.0,         -min_x / res_w],
            [0.0,         1.0 / res_w, -min_y / res_w],
            [0.0,         0.0,         1.0]
        ], dtype=np.float64)

        M_full = T_world_pixel @ T_world_map @ T_map_pixel
        return M_full[:2, :].astype(np.float32)

    def merge_maps(self) -> None:
        if self.amr1_map is None and self.amr2_map is None:
            return

        # 1. Determine transforms for both maps
        tx1, ty1, yaw1 = self._get_transform("bcr_bot_amr1/map", self.amr1_spawn)
        tx2, ty2, yaw2 = self._get_transform("bcr_bot_amr2/map", self.amr2_spawn)

        # 2. Compute unified bounding envelope in world coordinates
        corners_list = []
        res = 0.05
        if self.amr1_map is not None:
            corners_list.append(self._get_map_corners_world(self.amr1_map, tx1, ty1, yaw1))
            res = self.amr1_map.info.resolution
        if self.amr2_map is not None:
            corners_list.append(self._get_map_corners_world(self.amr2_map, tx2, ty2, yaw2))
            res = self.amr2_map.info.resolution

        all_corners = np.vstack(corners_list)
        min_x = float(all_corners[:, 0].min())
        min_y = float(all_corners[:, 1].min())
        max_x = float(all_corners[:, 0].max())
        max_y = float(all_corners[:, 1].max())

        self.world_resolution = res
        self.world_origin = np.array([min_x, min_y], dtype=float)
        self.world_width = max(1, int(math.ceil((max_x - min_x) / res)))
        self.world_height = max(1, int(math.ceil((max_y - min_y) / res)))

        if self.visit_density is None or self.visit_density.shape != (self.world_height, self.world_width):
            self.visit_density = np.zeros((self.world_height, self.world_width), dtype=np.uint16)

        merged = np.full((self.world_height, self.world_width), -1, dtype=np.int8)
        skipped = 0
        frontier = 0

        # 3. Warp and fuse AMR-1 map (with selective throttling)
        if self.amr1_map is not None:
            m1 = self.amr1_map
            h1, w1 = m1.info.height, m1.info.width
            if h1 > 0 and w1 > 0 and len(m1.data) == h1 * w1:
                arr1 = np.array(m1.data, dtype=np.int8).reshape(h1, w1)
                u1 = np.where(arr1 == -1, 255, arr1).astype(np.uint8)

                M1 = self._make_affine_matrix(
                    m1.info.origin.position.x, m1.info.origin.position.y, m1.info.resolution,
                    tx1, ty1, yaw1, min_x, min_y, res
                )
                w1_warped = cv2.warpAffine(
                    u1, M1, (self.world_width, self.world_height),
                    flags=cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=255
                )
                m1_w = np.where(w1_warped == 255, -1, w1_warped).astype(np.int8)

                known1 = (m1_w >= 0)
                is_unknown = (m1_w == -1)

                # Vectorized 4-neighbor frontier detection
                frontier_amr1 = known1 & (
                    np.pad(is_unknown[1:, :], ((0, 1), (0, 0)), constant_values=False) |
                    np.pad(is_unknown[:-1, :], ((1, 0), (0, 0)), constant_values=False) |
                    np.pad(is_unknown[:, 1:], ((0, 0), (0, 1)), constant_values=False) |
                    np.pad(is_unknown[:, :-1], ((0, 0), (1, 0)), constant_values=False)
                )

                self.visit_density[known1] += 1
                use_amr1 = known1 & (frontier_amr1 | (self.visit_density <= self.visit_threshold))
                skipped += int(np.count_nonzero(known1 & (~use_amr1)))
                frontier += int(np.count_nonzero(frontier_amr1))

                merged[use_amr1] = m1_w[use_amr1]

        # 4. Warp and fuse AMR-2 map (direct priority overlay)
        if self.amr2_map is not None:
            m2 = self.amr2_map
            h2, w2 = m2.info.height, m2.info.width
            if h2 > 0 and w2 > 0 and len(m2.data) == h2 * w2:
                arr2 = np.array(m2.data, dtype=np.int8).reshape(h2, w2)
                u2 = np.where(arr2 == -1, 255, arr2).astype(np.uint8)

                M2 = self._make_affine_matrix(
                    m2.info.origin.position.x, m2.info.origin.position.y, m2.info.resolution,
                    tx2, ty2, yaw2, min_x, min_y, res
                )
                w2_warped = cv2.warpAffine(
                    u2, M2, (self.world_width, self.world_height),
                    flags=cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=255
                )
                m2_w = np.where(w2_warped == 255, -1, w2_warped).astype(np.int8)

                known2 = (m2_w >= 0)
                # Where merged is unknown, take AMR-2 values
                unassigned_mask = (merged == -1) & known2
                merged[unassigned_mask] = m2_w[unassigned_mask]
                # Where both are known, take the maximum occupancy (ensuring obstacle safety)
                both_known = (merged >= 0) & known2
                merged[both_known] = np.maximum(merged[both_known], m2_w[both_known])

        # 5. Clear inter-robot dynamic footprints from fused static map
        # When robots scan each other at spawn or in operation, they map each other as static obstacles.
        # We clear a circular footprint around each robot's world position to free space (0).
        robot_poses = [
            self._get_robot_pose_world("bcr_bot_amr1", self.amr1_spawn),
            self._get_robot_pose_world("bcr_bot_amr2", self.amr2_spawn),
        ]
        rad_px = int(math.ceil(self.robot_clear_radius / res))
        for rx, ry in robot_poses:
            rc = int(round((rx - min_x) / res))
            rr = int(round((ry - min_y) / res))

            r_min = max(0, rr - rad_px)
            r_max = min(self.world_height, rr + rad_px + 1)
            c_min = max(0, rc - rad_px)
            c_max = min(self.world_width, rc + rad_px + 1)

            if r_max > r_min and c_max > c_min:
                grid_y, grid_x = np.ogrid[r_min:r_max, c_min:c_max]
                dist_sq = (grid_x - rc) ** 2 + (grid_y - rr) ** 2
                circle_mask = dist_sq <= (rad_px ** 2)

                footprint_slice = merged[r_min:r_max, c_min:c_max]
                # Set occupied / mapped cells within robot footprint to clean free space (0)
                clear_target = circle_mask & (footprint_slice > 0)
                footprint_slice[clear_target] = 0

        # 6. Ramp / Slope Traversability Region Marking
        rx_min_col = max(0, int(math.floor((self.ramp_min_x - min_x) / res)))
        rx_max_col = min(self.world_width, int(math.ceil((self.ramp_max_x - min_x) / res)))
        ry_min_row = max(0, int(math.floor((self.ramp_min_y - min_y) / res)))
        ry_max_row = min(self.world_height, int(math.ceil((self.ramp_max_y - min_y) / res)))

        if rx_max_col > rx_min_col and ry_max_row > ry_min_row:
            ramp_slice = merged[ry_min_row:ry_max_row, rx_min_col:rx_max_col]
            ramp_mask = (ramp_slice != -1)
            ramp_slice[ramp_mask] = self.ramp_slope_cost

        updated = int(np.count_nonzero(merged != -1))

        # 7. Publish fused OccupancyGrid
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

        # 8. Publish diagnostics
        stats_msg = String()
        stats_msg.data = (
            f"total:{self.world_width * self.world_height} updated:{updated} "
            f"skipped:{skipped} frontier:{frontier}"
        )
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

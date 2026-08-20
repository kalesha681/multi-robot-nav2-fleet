#!/usr/bin/env python3
"""
SensorValidatorNode (amr_bsp)
Hardware Abstraction & Validation Layer for Multi-Robot Fleet.

Functions:
1. Subscribes to raw sensor streams (/<robot>/scan, /<robot>/imu).
2. Validates IMU angular velocity plausibility (|omega_z| <= max_limit).
3. Validates IMU linear acceleration limits and checks for NaN/Inf anomalies.
4. Validates LaserScan beam sanity (range validity, minimum beam density).
5. Ground/Ramp Filter: Prevents 2D planar LiDAR slope reflections from turning into lethal walls on custom ramps.
6. Publishes validated topics: /<robot>/validated/scan, /<robot>/validated/imu.
7. Publishes diagnostic health telemetry: /<robot>/sensor_health.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan, Imu
from amr_msgs.msg import SensorHealth
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class SensorValidatorNode(Node):
    def __init__(self):
        super().__init__('sensor_validator_node')

        # Declare robot parameter & validation thresholds
        self.declare_parameter('robot_name', 'bcr_bot_amr1')
        self.declare_parameter('max_angular_velocity_rad_s', 5.0)
        self.declare_parameter('max_linear_accel_m_s2', 20.0)
        self.declare_parameter('min_valid_beam_ratio', 0.10)
        self.declare_parameter('enable_ramp_ground_filter', True)

        # Ramp zone geometry in world frame
        self.declare_parameter('ramp_min_x', -4.2)
        self.declare_parameter('ramp_max_x', -2.6)
        self.declare_parameter('ramp_min_y', -4.5)
        self.declare_parameter('ramp_max_y', 4.5)

        self.robot_name = self.get_parameter('robot_name').value
        self.max_omega = float(self.get_parameter('max_angular_velocity_rad_s').value)
        self.max_accel = float(self.get_parameter('max_linear_accel_m_s2').value)
        self.min_beam_ratio = float(self.get_parameter('min_valid_beam_ratio').value)
        self.filter_ramp = bool(self.get_parameter('enable_ramp_ground_filter').value)

        self.ramp_x0 = float(self.get_parameter('ramp_min_x').value)
        self.ramp_x1 = float(self.get_parameter('ramp_max_x').value)
        self.ramp_y0 = float(self.get_parameter('ramp_min_y').value)
        self.ramp_y1 = float(self.get_parameter('ramp_max_y').value)

        self.rejected_imu_count = 0
        self.rejected_scan_count = 0
        self.last_imu_healthy = True
        self.last_scan_healthy = True

        # TF2 listener for world-frame ground ray projection
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Input raw topic subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            f'/{self.robot_name}/scan',
            self.scan_callback,
            10
        )
        self.imu_sub = self.create_subscription(
            Imu,
            f'/{self.robot_name}/imu',
            self.imu_callback,
            10
        )

        # Output validated topic publishers
        self.scan_pub = self.create_publisher(
            LaserScan,
            f'/{self.robot_name}/validated/scan',
            10
        )
        self.imu_pub = self.create_publisher(
            Imu,
            f'/{self.robot_name}/validated/imu',
            10
        )

        # 1 Hz Diagnostic Telemetry publisher
        self.health_pub = self.create_publisher(
            SensorHealth,
            f'/{self.robot_name}/sensor_health',
            10
        )
        self.create_timer(1.0, self.publish_health_status)

        # TF listener for ramp ground-filter world-frame projection
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info(
            f'[{self.robot_name.upper()}_BSP] SensorValidator initialized. '
            f'Limits: omega_max={self.max_omega} rad/s, accel_max={self.max_accel} m/s^2, ramp_filter={self.filter_ramp}'
        )

    def imu_callback(self, msg: Imu):
        """Enforces angular velocity bounds, linear acceleration caps, and checks for NaN/Inf."""
        wz = msg.angular_velocity.z
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        # Check for NaN / Inf
        if any(math.isnan(v) or math.isinf(v) for v in [wz, ax, ay, az]):
            self.rejected_imu_count += 1
            self.last_imu_healthy = False
            self.get_logger().warn(
                f'[{self.robot_name}_BSP] IMU NaN/Inf anomaly detected. Dropping sample.',
                throttle_duration_sec=2.0
            )
            return

        # Plausibility check: angular velocity bounds
        if abs(wz) > self.max_omega:
            self.rejected_imu_count += 1
            self.last_imu_healthy = False
            self.get_logger().warn(
                f'[{self.robot_name}_BSP] IMU wz ({wz:.2f} rad/s) exceeded limit ({self.max_omega} rad/s). Dropping.',
                throttle_duration_sec=2.0
            )
            return

        # Plausibility check: linear acceleration magnitude
        accel_mag = math.sqrt(ax**2 + ay**2 + az**2)
        if accel_mag > self.max_accel:
            self.rejected_imu_count += 1
            self.last_imu_healthy = False
            self.get_logger().warn(
                f'[{self.robot_name}_BSP] IMU accel ({accel_mag:.2f} m/s^2) exceeded limit ({self.max_accel} m/s^2). Dropping.',
                throttle_duration_sec=2.0
            )
            return

        self.last_imu_healthy = True
        self.imu_pub.publish(msg)

    def scan_callback(self, msg: LaserScan):
        """Validates LaserScan bounds, filters NaN/Inf, and clears ramp ground strike returns."""
        if not msg.ranges:
            self.rejected_scan_count += 1
            self.last_scan_healthy = False
            return

        ranges = list(msg.ranges)
        total_beams = len(ranges)
        valid_beams = 0

        # Validate range values and sanitize
        min_r = msg.range_min if msg.range_min > 0.0 else 0.05
        max_r = msg.range_max if msg.range_max > 0.0 else 30.0

        for i in range(total_beams):
            r = ranges[i]
            if math.isnan(r) or math.isinf(r):
                ranges[i] = float('nan')
            elif r < min_r or r > max_r:
                ranges[i] = float('nan')
            else:
                valid_beams += 1

        beam_ratio = valid_beams / float(total_beams) if total_beams > 0 else 0.0
        if beam_ratio < self.min_beam_ratio and total_beams > 0:
            self.rejected_scan_count += 1
            self.last_scan_healthy = False
            return

        self.last_scan_healthy = True
        msg.ranges = ranges

        if self.filter_ramp:
            self._apply_ramp_ground_filter(msg)

        self.scan_pub.publish(msg)

    def _get_ramp_height(self, y_world: float) -> float:
        """Expected ramp surface height at a given world y coordinate."""
        if -1.0 <= y_world <= 1.0:
            return 0.529
        if 1.0 < y_world <= 4.023:
            return 0.529 * max(0.0, (4.023 - y_world) / 3.023)
        if -4.023 <= y_world < -1.0:
            return 0.529 * max(0.0, (y_world + 4.023) / 3.023)
        return 0.0

    def _apply_ramp_ground_filter(self, msg: LaserScan):
        """Project scan points into 3D world frame, level valid returns horizontally, and clear ground/ceiling strikes."""
        try:
            trans = self.tf_buffer.lookup_transform(
                'world',
                f'{self.robot_name}/two_d_lidar',
                rclpy.time.Time()
            )
        except Exception:
            return

        t = trans.transform.translation
        q = trans.transform.rotation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        xx, xy, xz = qx * qx, qx * qy, qx * qz
        xw_val = qx * qw
        yy, yz, yw_val = qy * qy, qy * qz, qy * qw
        zz, zw = qz * qz, qz * qw

        R00 = 1 - 2 * (yy + zz)
        R01 = 2 * (xy - zw)
        R02 = 2 * (xz + yw_val)
        R10 = 2 * (xy + zw)
        R11 = 1 - 2 * (xx + zz)
        R12 = 2 * (yz - xw_val)
        R20 = 2 * (xz - yw_val)
        R21 = 2 * (yz + xw_val)
        R22 = 1 - 2 * (xx + yy)

        tx, ty, tz = t.x, t.y, t.z
        angle = msg.angle_min
        filtered = 0

        for i, r in enumerate(msg.ranges):
            if math.isnan(r) or math.isinf(r):
                angle += msg.angle_increment
                continue
            x_l = r * math.cos(angle)
            y_l = r * math.sin(angle)
            z_l = 0.0
            x_w = R00 * x_l + R01 * y_l + R02 * z_l + tx
            y_w = R10 * x_l + R11 * y_l + R12 * z_l + ty
            z_w = R20 * x_l + R21 * y_l + R22 * z_l + tz

            # Calculate expected ground elevation
            if self.ramp_x0 <= x_w <= self.ramp_x1 and self.ramp_y0 <= y_w <= self.ramp_y1:
                z_ground = self._get_ramp_height(y_w)
            else:
                z_ground = 0.0

            # 1. Ground strike filter (within 0.10m of surface)
            if abs(z_w - z_ground) < 0.10 or z_w < 0.06:
                msg.ranges[i] = float('nan')
                filtered += 1
            # 2. Ceiling / high overhead ray filter
            elif z_w > 2.0:
                msg.ranges[i] = float('nan')
                filtered += 1
            else:
                # 3. Virtual Leveling: replace slant range with true horizontal distance
                r_horiz = math.hypot(x_w - tx, y_w - ty)
                msg.ranges[i] = r_horiz

            angle += msg.angle_increment

        if filtered > 0:
            self.get_logger().info(
                f'[{self.robot_name.upper()}_BSP] Ramp & Leveling filter cleared {filtered} beams',
                throttle_duration_sec=3.0
            )

    def publish_health_status(self):
        """Publishes 1 Hz SensorHealth diagnostic telemetry."""
        health = SensorHealth()
        health.header.stamp = self.get_clock().now().to_msg()
        health.header.frame_id = f'{self.robot_name}/base_link'
        health.robot_id = self.robot_name
        health.imu_healthy = self.last_imu_healthy
        health.scan_healthy = self.last_scan_healthy
        health.imu_ang_vel_limit = self.max_omega
        health.valid_scan_beam_ratio = 1.0 if self.last_scan_healthy else 0.0
        health.rejected_samples_count = self.rejected_imu_count + self.rejected_scan_count

        if self.last_imu_healthy and self.last_scan_healthy:
            health.status_message = "HEALTHY_ALL_SENSORS_OPERATIONAL"
        elif not self.last_imu_healthy:
            health.status_message = "IMU_PLAUSIBILITY_FAULT"
        else:
            health.status_message = "LIDAR_DEGRADED_SCAN"

        self.health_pub.publish(health)


def main(args=None):
    rclpy.init(args=args)
    node = SensorValidatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

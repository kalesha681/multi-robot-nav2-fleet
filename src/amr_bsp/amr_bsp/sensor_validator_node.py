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

        # Look up robot laser position in world frame for ramp geometric filtering
        tx, ty, yaw = None, None, None
        if self.filter_ramp:
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    'world',
                    msg.header.frame_id,
                    rclpy.time.Time()
                )
                tx = tf_msg.transform.translation.x
                ty = tf_msg.transform.translation.y
                qz = tf_msg.transform.rotation.z
                qw = tf_msg.transform.rotation.w
                yaw = 2.0 * math.atan2(qz, qw)
            except Exception:
                pass

        for i in range(total_beams):
            r = ranges[i]
            if math.isnan(r) or math.isinf(r):
                ranges[i] = float('nan')
            elif r < min_r or r > max_r:
                ranges[i] = float('nan')
            else:
                # If point falls within the known traversable ramp footprint, filter out ground reflection
                if yaw is not None:
                    angle = msg.angle_min + i * msg.angle_increment
                    pw_x = tx + r * math.cos(yaw + angle)
                    pw_y = ty + r * math.sin(yaw + angle)
                    if self.ramp_x0 <= pw_x <= self.ramp_x1 and self.ramp_y0 <= pw_y <= self.ramp_y1:
                        ranges[i] = float('nan')
                        continue
                valid_beams += 1

        beam_ratio = valid_beams / float(total_beams) if total_beams > 0 else 0.0
        if beam_ratio < self.min_beam_ratio and total_beams > 0:
            self.rejected_scan_count += 1
            self.last_scan_healthy = False
            return

        self.last_scan_healthy = True
        msg.ranges = ranges
        self.scan_pub.publish(msg)

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

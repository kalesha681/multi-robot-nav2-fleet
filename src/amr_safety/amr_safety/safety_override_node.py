#!/usr/bin/env python3
"""
SafetyOverrideNode (amr_safety)
Deterministic, Independent Safety Supervisory & Veto Layer.

Architecture:
1. Subscribes directly to raw LiDAR (/<robot>/scan) to eliminate dependencies on upstream processing.
2. Subscribes to <ns>/sensor_health (from amr_bsp) for hardware status monitoring.
3. Subscribes to <ns>/cmd_vel_nav from Nav2 MPPI controller.
4. Evaluates physics-based dynamic stopping distance, directional cones, and hysteresis state machine.
5. Absolute Veto Authority: Publishes the final gated command to <ns>/cmd_vel.
6. Publishes SafetyStatus telemetry at 10Hz to <ns>/safety_status.
"""

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from amr_msgs.msg import SensorHealth, SafetyStatus

from amr_safety.dynamic_safety_zone import (
    SafetyState,
    calculate_stopping_distance,
    filter_scan_points,
    check_command_watchdog,
    transition_safety_state,
)


class SafetyOverrideNode(Node):
    def __init__(self):
        super().__init__('safety_override_node')

        # Parameters
        self.declare_parameter('robot_name', 'bcr_bot_amr1')
        self.declare_parameter('a_decel_max', 1.0)           # Max deceleration (m/s^2)
        self.declare_parameter('d_margin', 0.35)             # Static bumper margin (m)
        self.declare_parameter('d_warning', 0.85)            # Slowdown warning threshold (m)
        self.declare_parameter('release_margin', 0.15)       # Hysteresis clearance buffer (m)
        self.declare_parameter('forward_cone_deg', 35.0)     # Forward scanning half-angle (deg)
        self.declare_parameter('reverse_cone_deg', 35.0)     # Reverse scanning half-angle (deg)
        self.declare_parameter('rotation_safety_radius', 0.32) # Radial bumper clearance for pivot turns (m)
        self.declare_parameter('sensor_timeout_sec', 0.3)    # Max allowable sensor silence before e-stop (s)
        self.declare_parameter('expected_cmd_hz', 10.0)      # Expected Nav2 command rate (Hz)
        self.declare_parameter('loop_rate_hz', 30.0)         # Safety evaluation loop rate (Hz)
        self.declare_parameter('telemetry_rate_hz', 10.0)    # Telemetry publishing rate (Hz)

        self.robot_name = self.get_parameter('robot_name').value
        self.a_decel_max = float(self.get_parameter('a_decel_max').value)
        self.d_margin = float(self.get_parameter('d_margin').value)
        self.d_warning = float(self.get_parameter('d_warning').value)
        self.release_margin = float(self.get_parameter('release_margin').value)
        self.forward_cone_deg = float(self.get_parameter('forward_cone_deg').value)
        self.reverse_cone_deg = float(self.get_parameter('reverse_cone_deg').value)
        self.rotation_safety_radius = float(self.get_parameter('rotation_safety_radius').value)
        self.sensor_timeout_sec = float(self.get_parameter('sensor_timeout_sec').value)
        self.expected_cmd_hz = float(self.get_parameter('expected_cmd_hz').value)
        self.loop_rate_hz = float(self.get_parameter('loop_rate_hz').value)
        self.telemetry_rate_hz = float(self.get_parameter('telemetry_rate_hz').value)

        # Internal State
        self.current_state = SafetyState.CLEAR
        self.current_measured_speed = 0.0
        self.last_cmd_nav = Twist()
        self.last_cmd_nav_time = 0.0

        self.latest_scan_pairs = []
        self.last_scan_time = 0.0

        self.sensor_health_ok = True
        self.last_health_time = 0.0

        self.current_d_safe = self.d_margin
        self.closest_obstacle_dist = 999.0
        self.dynamic_speed_limit = 1.0
        self.safety_reason = 'CLEAR'

        # Subscriptions
        self.cmd_nav_sub = self.create_subscription(
            Twist,
            f'/{self.robot_name}/cmd_vel_nav',
            self.cmd_nav_callback,
            10
        )
        # Direct raw LiDAR subscription (no dependency on other nodes)
        self.scan_sub = self.create_subscription(
            LaserScan,
            f'/{self.robot_name}/scan',
            self.scan_callback,
            10
        )
        self.health_sub = self.create_subscription(
            SensorHealth,
            f'/{self.robot_name}/sensor_health',
            self.sensor_health_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.robot_name}/odom',
            self.odom_callback,
            10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            f'/{self.robot_name}/cmd_vel',
            10
        )
        self.status_pub = self.create_publisher(
            SafetyStatus,
            f'/{self.robot_name}/safety_status',
            10
        )

        # Periodic Timers
        self.safety_timer = self.create_timer(1.0 / self.loop_rate_hz, self.safety_loop)
        self.telemetry_timer = self.create_timer(1.0 / self.telemetry_rate_hz, self.publish_telemetry)

        self.get_logger().info(
            f'[{self.robot_name.upper()}_SAFETY] SafetyOverrideNode active. '
            f'a_decel_max={self.a_decel_max} m/s^2, d_margin={self.d_margin}m, d_warning={self.d_warning}m, '
            f'hysteresis_margin={self.release_margin}m, loop={self.loop_rate_hz}Hz'
        )

    def _get_now_sec(self) -> float:
        t = self.get_clock().now()
        nanos = t.nanoseconds
        if nanos == 0:
            return time.time()
        return nanos * 1e-9

    def cmd_nav_callback(self, msg: Twist):
        self.last_cmd_nav = msg
        self.last_cmd_nav_time = self._get_now_sec()

    def scan_callback(self, msg: LaserScan):
        pairs = []
        angle = msg.angle_min
        for r in msg.ranges:
            if not math.isnan(r) and not math.isinf(r) and msg.range_min <= r <= msg.range_max:
                pairs.append((angle, r))
            angle += msg.angle_increment
        self.latest_scan_pairs = pairs
        self.last_scan_time = self._get_now_sec()

    def sensor_health_callback(self, msg: SensorHealth):
        self.sensor_health_ok = msg.scan_healthy and msg.imu_healthy
        self.last_health_time = self._get_now_sec()

    def odom_callback(self, msg: Odometry):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_measured_speed = math.sqrt(vx * vx + vy * vy)

    def safety_loop(self):
        """Authoritative safety evaluation executed at high frequency."""
        now = self._get_now_sec()
        out_cmd = Twist()

        # 1. Command Watchdog Check
        if self.last_cmd_nav_time == 0.0:
            time_since_cmd = 999.0
        else:
            time_since_cmd = now - self.last_cmd_nav_time

        is_cmd_fresh = check_command_watchdog(time_since_cmd, self.expected_cmd_hz)
        if not is_cmd_fresh and self.last_cmd_nav_time > 0.0:
            self.get_logger().warn(
                f'[{self.robot_name}] Nav command stale ({time_since_cmd:.2f}s > {2.0/self.expected_cmd_hz:.2f}s). Zeroing cmd_vel.',
                throttle_duration_sec=2.0
            )

        # 2. Sensor Watchdog Check
        scan_age = now - self.last_scan_time if self.last_scan_time > 0.0 else 999.0
        is_sensor_ok = (scan_age <= self.sensor_timeout_sec) and self.sensor_health_ok

        # 3. Commanded Velocities
        cmd_vx = self.last_cmd_nav.linear.x
        cmd_wz = self.last_cmd_nav.angular.z

        # 4. Physics-Based Stopping Distance Calculation
        speed_for_braking = max(abs(cmd_vx), self.current_measured_speed)
        self.current_d_safe = calculate_stopping_distance(
            speed_for_braking,
            self.a_decel_max,
            self.d_margin
        )

        # 5. Sector Filtering for Closest Obstacle
        self.closest_obstacle_dist = filter_scan_points(
            self.latest_scan_pairs,
            cmd_vx,
            cmd_wz,
            forward_cone_deg=self.forward_cone_deg,
            reverse_cone_deg=self.reverse_cone_deg,
            rotation_safety_radius=self.rotation_safety_radius
        )

        # 6. State Transition with Hysteresis
        new_state, scale_factor, reason = transition_safety_state(
            current_state=self.current_state,
            obstacle_distance=self.closest_obstacle_dist,
            d_safe=self.current_d_safe,
            d_warning=self.d_warning,
            release_margin=self.release_margin,
            is_sensor_healthy=is_sensor_ok,
            is_cmd_fresh=is_cmd_fresh
        )
        self.current_state = new_state
        self.safety_reason = reason

        # 7. Apply Speed Gating & Absolute Authority
        if self.current_state in (SafetyState.EMERGENCY_STOP, SafetyState.SENSOR_FAULT_STOP, SafetyState.STALE_COMMAND):
            self.dynamic_speed_limit = 0.0
            out_cmd.linear.x = 0.0
            out_cmd.angular.z = 0.0
        elif self.current_state == SafetyState.SLOWDOWN:
            self.dynamic_speed_limit = scale_factor * abs(cmd_vx)
            out_cmd.linear.x = math.copysign(self.dynamic_speed_limit, cmd_vx)
            out_cmd.angular.z = cmd_wz
        else:  # CLEAR
            self.dynamic_speed_limit = abs(cmd_vx)
            out_cmd.linear.x = cmd_vx
            out_cmd.angular.z = cmd_wz

        self.cmd_vel_pub.publish(out_cmd)

    def publish_telemetry(self):
        """Publishes SafetyStatus diagnostic telemetry on <ns>/safety_status."""
        msg = SafetyStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'{self.robot_name}/base_link'
        msg.robot_id = self.robot_name
        msg.emergency_stop_active = (self.current_state == SafetyState.EMERGENCY_STOP)
        msg.current_speed = float(self.current_measured_speed)
        msg.min_stopping_distance = float(self.current_d_safe)
        msg.closest_obstacle_distance = float(self.closest_obstacle_dist if not math.isinf(self.closest_obstacle_dist) else 999.0)
        msg.dynamic_speed_limit = float(self.dynamic_speed_limit)
        msg.safety_reason = self.safety_reason
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyOverrideNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

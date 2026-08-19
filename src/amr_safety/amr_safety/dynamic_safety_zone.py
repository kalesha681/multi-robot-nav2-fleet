"""
dynamic_safety_zone.py (amr_safety)
Pure Python module containing physics, geometry, and state-transition logic for AMR safety.
Strictly decoupled from ROS 2 (no rclpy dependencies) for 100% deterministic unit testability.
"""

import math
from enum import Enum
from typing import List, Tuple, Optional


class SafetyState(Enum):
    CLEAR = "CLEAR"
    SLOWDOWN = "DYNAMIC_SLOWDOWN"
    EMERGENCY_STOP = "OBSTACLE_WITHIN_SAFETY_MARGIN"
    SENSOR_FAULT_STOP = "SENSOR_FAULT"
    STALE_COMMAND = "STALE_COMMAND"


def calculate_stopping_distance(
    velocity: float,
    a_decel_max: float,
    d_margin: float
) -> float:
    """
    Computes physics-based stopping distance:
        d_safe(v) = v^2 / (2 * a_decel_max) + d_margin

    Args:
        velocity: Current or commanded linear speed (m/s).
        a_decel_max: Maximum deceleration capability of the robot (m/s^2).
        d_margin: Static physical bumper safety margin (m).

    Returns:
        float: Minimum safe stopping distance in meters.
    """
    if a_decel_max <= 0.0:
        raise ValueError("a_decel_max must be strictly positive.")
    speed = abs(velocity)
    return (speed ** 2) / (2.0 * a_decel_max) + d_margin


def clamp_slowdown_velocity(
    v_cmd: float,
    obstacle_distance: float,
    d_safe: float,
    d_warning: float,
    epsilon: float = 1e-4
) -> float:
    """
    Calculates linearly attenuated velocity inside the slowdown zone:
        v_clamped = v_cmd * (d_obs - d_safe) / (d_warning - d_safe)

    Guards against division-by-zero when d_warning approaches d_safe.

    Args:
        v_cmd: Desired linear command (m/s).
        obstacle_distance: Measured distance to closest obstacle in active cone (m).
        d_safe: Dynamic stopping distance (m).
        d_warning: Warning threshold distance (m).
        epsilon: Denominator safety threshold.

    Returns:
        float: Scaled velocity command (m/s).
    """
    denom = d_warning - d_safe
    if denom <= epsilon:
        # Near-zero slowdown zone: binary decision (if obstacle beyond d_safe, full speed, else 0)
        return v_cmd if obstacle_distance > d_safe else 0.0

    ratio = (obstacle_distance - d_safe) / denom
    clamped_ratio = max(0.0, min(1.0, ratio))
    return v_cmd * clamped_ratio


def filter_scan_points(
    ranges_with_angles: List[Tuple[float, float]],
    v_cmd: float,
    w_cmd: float,
    forward_cone_deg: float = 35.0,
    reverse_cone_deg: float = 35.0,
    rotation_safety_radius: float = 0.32,
    v_epsilon: float = 0.05
) -> float:
    """
    Evaluates LaserScan points according to motion-aligned sectors or rotation bumper bubble.

    Args:
        ranges_with_angles: List of (angle_rad, distance_m) pairs.
        v_cmd: Commanded linear velocity (m/s).
        w_cmd: Commanded angular velocity (rad/s).
        forward_cone_deg: Half-angle for forward cone in degrees.
        reverse_cone_deg: Half-angle for reverse cone in degrees.
        rotation_safety_radius: Radial bumper check radius during in-place turning (m).
        v_epsilon: Linear velocity threshold below which robot is considered pivoting/stationary.

    Returns:
        float: Distance in meters to closest obstacle in active sector, or float('inf') if clear.
    """
    forward_cone_rad = math.radians(forward_cone_deg)
    reverse_cone_rad = math.radians(reverse_cone_deg)

    is_forward = v_cmd > v_epsilon
    is_reversing = v_cmd < -v_epsilon
    is_pure_rotation = abs(v_cmd) <= v_epsilon

    min_dist = float('inf')

    for angle_rad, dist_m in ranges_with_angles:
        if math.isnan(dist_m) or math.isinf(dist_m) or dist_m <= 0.0:
            continue

        # Normalize angle to [-pi, pi]
        norm_angle = math.atan2(math.sin(angle_rad), math.cos(angle_rad))

        in_active_sector = False

        if is_forward:
            # Frontal arc +/- forward_cone_rad
            if abs(norm_angle) <= forward_cone_rad:
                in_active_sector = True
        elif is_reversing:
            # Rear arc [pi - reverse_cone_rad, pi] and [-pi, -pi + reverse_cone_rad]
            if abs(norm_angle) >= (math.pi - reverse_cone_rad):
                in_active_sector = True
        elif is_pure_rotation:
            # During pivot turns, evaluate all directions within rotation_safety_radius
            if dist_m <= rotation_safety_radius:
                in_active_sector = True

        if in_active_sector and dist_m < min_dist:
            min_dist = dist_m

    return min_dist


def check_command_watchdog(
    time_since_last_cmd: float,
    expected_cmd_hz: float
) -> bool:
    """
    Evaluates whether navigation command is fresh or stale.
    Timeout threshold is 2x the expected controller period (2.0 / expected_cmd_hz).

    Args:
        time_since_last_cmd: Elapsed time since last cmd_vel_nav message (seconds).
        expected_cmd_hz: Expected update frequency (Hz).

    Returns:
        bool: True if command is fresh, False if stale.
    """
    if expected_cmd_hz <= 0.0:
        return False
    timeout = 2.0 / expected_cmd_hz
    return time_since_last_cmd <= timeout


def transition_safety_state(
    current_state: SafetyState,
    obstacle_distance: float,
    d_safe: float,
    d_warning: float,
    release_margin: float,
    is_sensor_healthy: bool,
    is_cmd_fresh: bool
) -> Tuple[SafetyState, float, str]:
    """
    Pure state-transition function with explicit hysteresis at the emergency boundary.

    Hysteresis Rule:
    - Entering EMERGENCY_STOP: obstacle_distance <= d_safe
    - Exiting EMERGENCY_STOP: obstacle_distance must exceed (d_safe + release_margin)

    Args:
        current_state: Current active SafetyState.
        obstacle_distance: Distance to closest obstacle in active cone (m).
        d_safe: Minimum stopping distance threshold (m).
        d_warning: Slowdown warning distance threshold (m).
        release_margin: Hysteresis recovery margin (m).
        is_sensor_healthy: Boolean watchdog status from sensors.
        is_cmd_fresh: Boolean watchdog status from navigation command.

    Returns:
        Tuple[SafetyState, float, str]: (new_state, speed_scale_factor [0.0..1.0], reason_string)
    """
    # 1. Sensor Fault has absolute veto
    if not is_sensor_healthy:
        return SafetyState.SENSOR_FAULT_STOP, 0.0, "SENSOR_FAULT"

    # 2. Command Stale Watchdog
    if not is_cmd_fresh:
        return SafetyState.STALE_COMMAND, 0.0, "STALE_COMMAND"

    # 3. Emergency Stop State Handling with Hysteresis
    if current_state == SafetyState.EMERGENCY_STOP:
        # Must exceed d_safe + release_margin to clear e-stop
        if obstacle_distance <= (d_safe + release_margin):
            return SafetyState.EMERGENCY_STOP, 0.0, "OBSTACLE_WITHIN_SAFETY_MARGIN"
        
        # Recovered past release margin: check if still in slowdown or clear
        if obstacle_distance <= d_warning:
            scale = clamp_slowdown_velocity(1.0, obstacle_distance, d_safe, d_warning)
            return SafetyState.SLOWDOWN, scale, "DYNAMIC_SLOWDOWN"
        else:
            return SafetyState.CLEAR, 1.0, "CLEAR"

    # 4. Non-Emergency States (CLEAR, SLOWDOWN, or recovering from FAULT/STALE)
    if obstacle_distance <= d_safe:
        return SafetyState.EMERGENCY_STOP, 0.0, "OBSTACLE_WITHIN_SAFETY_MARGIN"
    elif obstacle_distance <= d_warning:
        scale = clamp_slowdown_velocity(1.0, obstacle_distance, d_safe, d_warning)
        return SafetyState.SLOWDOWN, scale, "DYNAMIC_SLOWDOWN"
    else:
        return SafetyState.CLEAR, 1.0, "CLEAR"

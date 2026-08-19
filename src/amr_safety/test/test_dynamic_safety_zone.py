"""
test_dynamic_safety_zone.py
Pytest unit test suite for amr_safety.dynamic_safety_zone pure math/logic module.
Executes with zero ROS 2 (no rclpy) runtime dependencies.
"""

import math
import pytest
from amr_safety.dynamic_safety_zone import (
    SafetyState,
    calculate_stopping_distance,
    clamp_slowdown_velocity,
    filter_scan_points,
    check_command_watchdog,
    transition_safety_state,
)


def test_stopping_distance_kinematics():
    """Verify d_safe increases quadratically with speed and is larger for heavier robots (lower a_decel_max)."""
    d_margin = 0.35
    a_heavy = 1.0   # AMR-1: Heavier (lower max deceleration)
    a_light = 2.0   # AMR-2: Lighter (higher max deceleration)

    # 1. At zero speed, stopping distance equals static bumper margin
    assert calculate_stopping_distance(0.0, a_heavy, d_margin) == d_margin

    # 2. At 0.5 m/s:
    # Heavy: 0.5^2 / (2 * 1.0) + 0.35 = 0.25 / 2.0 + 0.35 = 0.125 + 0.35 = 0.475m
    # Light: 0.5^2 / (2 * 2.0) + 0.35 = 0.25 / 4.0 + 0.35 = 0.0625 + 0.35 = 0.4125m
    d_heavy_05 = calculate_stopping_distance(0.5, a_heavy, d_margin)
    d_light_05 = calculate_stopping_distance(0.5, a_light, d_margin)

    assert d_heavy_05 > d_light_05
    assert pytest.approx(d_heavy_05, 0.001) == 0.475
    assert pytest.approx(d_light_05, 0.001) == 0.4125

    # 3. Monotonic increase with speed
    d_heavy_10 = calculate_stopping_distance(1.0, a_heavy, d_margin)
    assert d_heavy_10 > d_heavy_05
    assert pytest.approx(d_heavy_10, 0.001) == 0.850


def test_hysteresis_state_transitions():
    """
    Verify strict hysteresis behavior at the emergency boundary:
    - CLEAR -> EMERGENCY_STOP when distance <= d_safe
    - Stays EMERGENCY_STOP when distance recovers above d_safe but <= d_safe + release_margin
    - Returns to SLOWDOWN/CLEAR only when distance > d_safe + release_margin
    """
    d_safe = 0.50
    d_warning = 0.90
    release_margin = 0.15  # Recovery threshold = 0.50 + 0.15 = 0.65m

    # 1. Initial CLEAR state with obstacle far away (1.5m)
    state, scale, reason = transition_safety_state(
        current_state=SafetyState.CLEAR,
        obstacle_distance=1.5,
        d_safe=d_safe,
        d_warning=d_warning,
        release_margin=release_margin,
        is_sensor_healthy=True,
        is_cmd_fresh=True
    )
    assert state == SafetyState.CLEAR
    assert scale == 1.0
    assert reason == "CLEAR"

    # 2. Obstacle approaches slowdown zone (0.75m)
    state, scale, reason = transition_safety_state(
        current_state=SafetyState.CLEAR,
        obstacle_distance=0.75,
        d_safe=d_safe,
        d_warning=d_warning,
        release_margin=release_margin,
        is_sensor_healthy=True,
        is_cmd_fresh=True
    )
    assert state == SafetyState.SLOWDOWN
    assert 0.0 < scale < 1.0
    assert reason == "DYNAMIC_SLOWDOWN"

    # 3. Obstacle breaches lethal stopping distance (0.45m <= 0.50m) -> Enters EMERGENCY_STOP
    state, scale, reason = transition_safety_state(
        current_state=SafetyState.SLOWDOWN,
        obstacle_distance=0.45,
        d_safe=d_safe,
        d_warning=d_warning,
        release_margin=release_margin,
        is_sensor_healthy=True,
        is_cmd_fresh=True
    )
    assert state == SafetyState.EMERGENCY_STOP
    assert scale == 0.0
    assert reason == "OBSTACLE_WITHIN_SAFETY_MARGIN"

    # 4. HYSTERESIS HOLD: Obstacle backs off to 0.58m (above d_safe=0.50m, but BELOW d_safe+release=0.65m)
    # MUST REMAIN in EMERGENCY_STOP!
    state, scale, reason = transition_safety_state(
        current_state=SafetyState.EMERGENCY_STOP,
        obstacle_distance=0.58,
        d_safe=d_safe,
        d_warning=d_warning,
        release_margin=release_margin,
        is_sensor_healthy=True,
        is_cmd_fresh=True
    )
    assert state == SafetyState.EMERGENCY_STOP
    assert scale == 0.0
    assert reason == "OBSTACLE_WITHIN_SAFETY_MARGIN"

    # 5. HYSTERESIS RELEASE: Obstacle moves to 0.70m (exceeds recovery threshold 0.65m)
    # Resumes SLOWDOWN
    state, scale, reason = transition_safety_state(
        current_state=SafetyState.EMERGENCY_STOP,
        obstacle_distance=0.70,
        d_safe=d_safe,
        d_warning=d_warning,
        release_margin=release_margin,
        is_sensor_healthy=True,
        is_cmd_fresh=True
    )
    assert state == SafetyState.SLOWDOWN
    assert 0.0 < scale < 1.0
    assert reason == "DYNAMIC_SLOWDOWN"

    # 6. Obstacle clears completely (1.2m) -> CLEAR
    state, scale, reason = transition_safety_state(
        current_state=SafetyState.SLOWDOWN,
        obstacle_distance=1.2,
        d_safe=d_safe,
        d_warning=d_warning,
        release_margin=release_margin,
        is_sensor_healthy=True,
        is_cmd_fresh=True
    )
    assert state == SafetyState.CLEAR
    assert scale == 1.0


def test_slowdown_clamping_and_zero_division_guard():
    """Verify linear attenuation and verify zero division guard when d_warning approaches d_safe."""
    v_cmd = 0.8
    d_safe = 0.50
    d_warning = 0.90  # Span = 0.40m

    # 1. Exact midpoint: d_obs = 0.70m -> ratio = 0.20 / 0.40 = 0.50 -> v_clamped = 0.40 m/s
    v_clamped = clamp_slowdown_velocity(v_cmd, 0.70, d_safe, d_warning)
    assert pytest.approx(v_clamped, 0.001) == 0.40

    # 2. At boundary d_obs = d_warning (0.90m) -> full v_cmd
    assert pytest.approx(clamp_slowdown_velocity(v_cmd, 0.90, d_safe, d_warning), 0.001) == 0.80

    # 3. At boundary d_obs = d_safe (0.50m) -> 0.0
    assert pytest.approx(clamp_slowdown_velocity(v_cmd, 0.50, d_safe, d_warning), 0.001) == 0.0

    # 4. Zero division guard: d_warning == d_safe
    v_zero_span = clamp_slowdown_velocity(v_cmd, 0.55, d_safe=0.50, d_warning=0.50)
    assert v_zero_span == 0.8  # Above d_safe -> binary clear

    v_zero_span_blocked = clamp_slowdown_velocity(v_cmd, 0.45, d_safe=0.50, d_warning=0.50)
    assert v_zero_span_blocked == 0.0  # Below d_safe -> binary e-stop


def test_direction_aware_sector_filtering():
    """
    Verify directional sector selection:
    - Forward motion (v > 0): detects points in forward cone (0 deg), ignores reverse cone (180 deg).
    - Reverse motion (v < 0): detects points in rear cone (180 deg), ignores forward cone (0 deg).
    - Pure rotation (v ~ 0, w != 0): detects side point (90 deg) if within rotation_safety_radius.
    """
    forward_pt = (0.0, 0.40)                    # 0 deg (Front), 0.40m
    rear_pt = (math.pi, 0.40)                   # 180 deg (Back), 0.40m
    side_pt = (math.pi / 2.0, 0.30)             # 90 deg (Left Side), 0.30m
    far_side_pt = (math.pi / 2.0, 1.00)         # 90 deg (Left Side), 1.00m

    # 1. Forward motion (v = 0.5 m/s, w = 0.0)
    # Should catch forward_pt, but ignore rear_pt and side_pt
    scan_forward_and_rear = [forward_pt, rear_pt]
    d_fwd = filter_scan_points(scan_forward_and_rear, v_cmd=0.5, w_cmd=0.0)
    assert d_fwd == 0.40

    scan_only_rear = [rear_pt]
    d_fwd_rear_only = filter_scan_points(scan_only_rear, v_cmd=0.5, w_cmd=0.0)
    assert math.isinf(d_fwd_rear_only)  # Rear point is ignored during forward motion

    # 2. Reverse motion (v = -0.5 m/s, w = 0.0)
    # Should catch rear_pt, but ignore forward_pt
    d_rev = filter_scan_points([forward_pt, rear_pt], v_cmd=-0.5, w_cmd=0.0)
    assert d_rev == 0.40

    d_rev_fwd_only = filter_scan_points([forward_pt], v_cmd=-0.5, w_cmd=0.0)
    assert math.isinf(d_rev_fwd_only)  # Front point is ignored during reverse motion

    # 3. Pure rotation (v = 0.0 m/s, w = 0.8 rad/s)
    # Side point at 0.30m <= rotation_safety_radius (0.32m) is CAUGHT
    d_rot_close = filter_scan_points([side_pt], v_cmd=0.0, w_cmd=0.8, rotation_safety_radius=0.32)
    assert d_rot_close == 0.30

    # Far side point at 1.00m > rotation_safety_radius (0.32m) is IGNORED
    d_rot_far = filter_scan_points([far_side_pt], v_cmd=0.0, w_cmd=0.8, rotation_safety_radius=0.32)
    assert math.isinf(d_rot_far)


def test_command_watchdog():
    """Verify command watchdog timeout calculation at expected controller rate (10Hz)."""
    expected_hz = 10.0  # Period = 0.10s, Timeout = 2 * 0.10s = 0.20s

    # Fresh command at 0.05s elapsed -> Fresh
    assert check_command_watchdog(0.05, expected_hz) is True

    # Fresh command at 0.18s elapsed (< 0.20s) -> Fresh
    assert check_command_watchdog(0.18, expected_hz) is True

    # Stale command at 0.25s elapsed (> 0.20s) -> Stale
    assert check_command_watchdog(0.25, expected_hz) is False

    # Extended silence at 1.0s elapsed -> Stale
    assert check_command_watchdog(1.0, expected_hz) is False

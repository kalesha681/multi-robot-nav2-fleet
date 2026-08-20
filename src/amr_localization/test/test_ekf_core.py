# Copyright 2026 Abhinash
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Pytest unit tests for EKFCore mathematical engine (pure Python + NumPy, no rclpy)."""

import math
import numpy as np
import pytest

from amr_localization.ekf_core import EKFCore, normalize_angle


def test_predict_kinematics():
    """Verify that predict() propagates state according to exact kinematic equations."""
    ekf = EKFCore()
    ekf.reset(x=0.0, y=0.0, theta=0.0)

    # Initial state: x=0, y=0, theta=0, v=1.0 m/s, omega=0.0 rad/s
    ekf.x[3, 0] = 1.0
    dt = 0.5
    a_x = 0.2  # 0.2 m/s^2 forward accel
    omega_z = 0.1  # 0.1 rad/s yaw rate

    ekf.predict(dt=dt, a_x_imu=a_x, omega_z_imu=omega_z)

    # Expected:
    # dx = (v * dt + 0.5 * a_x * dt^2) = (1.0 * 0.5 + 0.5 * 0.2 * 0.25) = 0.5 + 0.025 = 0.525 m
    # dy = 0.0 (theta=0 initially)
    # theta = 0.0 + omega_z * dt = 0.05 rad
    # v = 1.0 + 0.2 * 0.5 = 1.1 m/s
    # omega = 0.1 rad/s
    x, y, yaw = ekf.get_pose()
    v, omega = ekf.get_twist()

    assert pytest.approx(x, abs=1e-3) == 0.525
    assert pytest.approx(y, abs=1e-3) == 0.0
    assert pytest.approx(yaw, abs=1e-3) == 0.05
    assert pytest.approx(v, abs=1e-3) == 1.1
    assert pytest.approx(omega, abs=1e-3) == 0.1


def test_update_wheel_convergence():
    """Verify that update_wheel() pulls a perturbed state toward the true measured velocity."""
    ekf = EKFCore()
    ekf.reset(x=0.0, y=0.0, theta=0.0)

    # Start with v=0.0 m/s
    assert ekf.x[3, 0] == 0.0

    # Repeatedly feed measured v=1.5 m/s, omega=0.2 rad/s with no slip
    target_v = 1.5
    target_omega = 0.2

    for _ in range(50):
        ekf.predict(dt=0.02, a_x_imu=0.0, omega_z_imu=target_omega)
        ekf.update_wheel(v_wheel=target_v, omega_wheel=target_omega, a_x_imu=0.0, dt_wheel=0.02)

    v, omega = ekf.get_twist()
    assert pytest.approx(v, abs=0.05) == target_v
    assert pytest.approx(omega, abs=0.05) == target_omega


def test_adaptive_slip_rejection():
    """Verify that when wheel acceleration contradicts IMU acceleration, slip is mitigated."""
    # Instance 1: Normal traction case (wheel accel matches IMU)
    ekf_normal = EKFCore(slip_threshold=1.5, slip_scale=50.0)
    ekf_normal.reset(x=0.0, y=0.0, theta=0.0)
    ekf_normal.predict(dt=0.1, a_x_imu=0.5, omega_z_imu=0.0)
    # v starts at 0.05; measured v=0.1 (accel = (0.1-0)/0.1 = 1.0 m/s^2, diff with IMU 0.5 is 0.5 <= 1.5)
    is_slip_normal = ekf_normal.update_wheel(v_wheel=0.1, omega_wheel=0.0, a_x_imu=0.5, dt_wheel=0.1)
    v_normal, _ = ekf_normal.get_twist()
    assert not is_slip_normal

    # Instance 2: Wheel spin slip case (wheel spins up to 2.0 m/s while IMU reports 0 m/s^2)
    ekf_slip = EKFCore(slip_threshold=1.5, slip_scale=50.0)
    ekf_slip.reset(x=0.0, y=0.0, theta=0.0)
    ekf_slip.predict(dt=0.1, a_x_imu=0.0, omega_z_imu=0.0)
    # v starts at 0.0; measured v=2.0 (accel = 2.0/0.1 = 20 m/s^2, diff with IMU 0 is 20 > 1.5)
    is_slip_detected = ekf_slip.update_wheel(v_wheel=2.0, omega_wheel=0.0, a_x_imu=0.0, dt_wheel=0.1)
    v_slip, _ = ekf_slip.get_twist()

    assert is_slip_detected
    # Because of slip scaling (R inflated 50x), v_slip should be heavily damped toward 0 rather than jumping to 2.0
    assert v_slip < 0.5


def test_yaw_normalization():
    """Verify that heading angle theta wraps properly within [-pi, pi]."""
    ekf = EKFCore()
    ekf.reset(x=0.0, y=0.0, theta=math.pi - 0.1)  # Almost +pi

    # Propagate with positive yaw rate that pushes it past +pi
    ekf.predict(dt=1.0, a_x_imu=0.0, omega_z_imu=0.5)

    _, _, yaw = ekf.get_pose()
    assert -math.pi <= yaw <= math.pi
    assert yaw < 0.0  # Should wrap around into negative territory

    # Also test standalone normalization function
    assert pytest.approx(normalize_angle(3 * math.pi)) == normalize_angle(math.pi)
    assert pytest.approx(normalize_angle(-3 * math.pi)) == normalize_angle(-math.pi)

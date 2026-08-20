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

"""Pure mathematical 2D Extended Kalman Filter (EKF) core engine for AMR state estimation."""

import math
from typing import Optional, Tuple
import numpy as np


def normalize_angle(angle: float) -> float:
    """Normalize any angle into the range [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class EKFCore:
    """2D Extended Kalman Filter (EKF) Core for differential-drive mobile robots.

    State vector:
        x = [x, y, theta, v, omega]^T
        - x, y: 2D planar position (meters)
        - theta: Heading orientation (radians in [-pi, pi])
        - v: Longitudinal linear velocity (m/s)
        - omega: Yaw angular velocity (rad/s)
    """

    def __init__(
        self,
        q_pos: float = 1e-3,
        q_theta: float = 1e-3,
        q_v: float = 0.5,
        q_omega: float = 0.5,
        r_v_wheel: float = 0.05,
        r_omega_wheel: float = 0.05,
        r_omega_imu: float = 0.01,
        slip_threshold: float = 1.5,
        slip_scale: float = 50.0,
    ):
        # 5x1 State vector: [x, y, theta, v, omega]^T
        self.x = np.zeros((5, 1), dtype=np.float64)

        # 5x5 State Covariance matrix P
        self.P = np.diag([1e-4, 1e-4, 1e-4, 0.1, 0.1]).astype(np.float64)

        # 5x5 Process Noise Covariance matrix Q
        self.Q = np.diag([q_pos, q_pos, q_theta, q_v, q_omega]).astype(np.float64)

        # Base Measurement Noise Covariances
        self.r_v_wheel = r_v_wheel
        self.r_omega_wheel = r_omega_wheel
        self.r_omega_imu = r_omega_imu

        # Adaptive slip parameters
        self.slip_threshold = slip_threshold
        self.slip_scale = slip_scale

        # Previous velocity for wheel acceleration derivation
        self._last_v_wheel = 0.0
        self._last_wheel_time: Optional[float] = None

    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        """Reset state vector to initial pose in local odom frame."""
        self.x = np.zeros((5, 1), dtype=np.float64)
        self.x[0, 0] = x
        self.x[1, 0] = y
        self.x[2, 0] = normalize_angle(theta)
        self.P = np.diag([1e-4, 1e-4, 1e-4, 0.1, 0.1]).astype(np.float64)
        self._last_v_wheel = 0.0
        self._last_wheel_time = None

    def predict(self, dt: float, a_x_imu: float = 0.0, omega_z_imu: Optional[float] = None):
        """Propagate state vector and covariance forward in time by dt using IMU inertial inputs."""
        if dt <= 0.0:
            return

        theta = self.x[2, 0]
        v = self.x[3, 0]
        omega = omega_z_imu if omega_z_imu is not None else self.x[4, 0]

        cos_t = math.cos(theta)
        sin_t = math.sin(theta)

        # 1. Non-linear state propagation f(x, u)
        dx = (v * cos_t + 0.5 * a_x_imu * cos_t * dt) * dt
        dy = (v * sin_t + 0.5 * a_x_imu * sin_t * dt) * dt
        dtheta = omega * dt
        dv = a_x_imu * dt

        self.x[0, 0] += dx
        self.x[1, 0] += dy
        self.x[2, 0] = normalize_angle(self.x[2, 0] + dtheta)
        self.x[3, 0] += dv
        self.x[4, 0] = omega

        # 2. State transition Jacobian F = df/dx
        F = np.eye(5, dtype=np.float64)
        F[0, 2] = -(v * sin_t + 0.5 * a_x_imu * sin_t * dt) * dt
        F[0, 3] = cos_t * dt
        F[1, 2] = (v * cos_t + 0.5 * a_x_imu * cos_t * dt) * dt
        F[1, 3] = sin_t * dt
        F[2, 4] = dt

        # 3. Covariance propagation P = F P F^T + Q * dt
        self.P = F @ self.P @ F.T + self.Q * dt

    def update_wheel(
        self,
        v_wheel: float,
        omega_wheel: float,
        a_x_imu: Optional[float] = None,
        dt_wheel: Optional[float] = None,
    ) -> bool:
        """Correct state with wheel encoder odometry velocity measurements [v, omega].

        Returns True if adaptive slip was detected and mitigated.
        """
        # Measurement matrix H_wheel: observes [v, omega]
        H = np.zeros((2, 5), dtype=np.float64)
        H[0, 3] = 1.0  # v
        H[1, 4] = 1.0  # omega

        # Adaptive slip detection: compare wheel acceleration vs IMU linear acceleration
        is_slip = False
        scale = 1.0
        if a_x_imu is not None and dt_wheel is not None and dt_wheel > 1e-4:
            a_wheel = (v_wheel - self._last_v_wheel) / dt_wheel
            accel_discrepancy = abs(a_wheel - a_x_imu)
            if accel_discrepancy > self.slip_threshold:
                is_slip = True
                scale = self.slip_scale

        self._last_v_wheel = v_wheel

        # Measurement covariance R
        R = np.diag([self.r_v_wheel * scale, self.r_omega_wheel * scale]).astype(np.float64)

        # Innovation: y = z - Hx
        z = np.array([[v_wheel], [omega_wheel]], dtype=np.float64)
        y = z - H @ self.x

        # Innovation covariance: S = H P H^T + R
        S = H @ self.P @ H.T + R

        # Kalman Gain: K = P H^T S^-1
        K = self.P @ H.T @ np.linalg.inv(S)

        # State update: x = x + Ky
        self.x = self.x + K @ y
        self.x[2, 0] = normalize_angle(self.x[2, 0])

        # Covariance update (Joseph form for numerical symmetry and positive-definiteness)
        I_KH = np.eye(5, dtype=np.float64) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T

        return is_slip

    def update_imu(self, omega_z_imu: float):
        """Correct yaw angular velocity directly from high-frequency IMU gyro."""
        H = np.zeros((1, 5), dtype=np.float64)
        H[0, 4] = 1.0  # omega

        R = np.array([[self.r_omega_imu]], dtype=np.float64)

        z = np.array([[omega_z_imu]], dtype=np.float64)
        y = z - H @ self.x

        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.x[2, 0] = normalize_angle(self.x[2, 0])

        I_KH = np.eye(5, dtype=np.float64) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T

    def get_pose(self) -> Tuple[float, float, float]:
        """Return estimated 2D planar pose (x, y, yaw)."""
        return float(self.x[0, 0]), float(self.x[1, 0]), float(self.x[2, 0])

    def get_twist(self) -> Tuple[float, float]:
        """Return estimated longitudinal linear and yaw angular velocities (v, omega)."""
        return float(self.x[3, 0]), float(self.x[4, 0])

    def get_covariance_6x6(self) -> np.ndarray:
        """Return standard 6x6 ROS pose covariance matrix for nav_msgs/Odometry."""
        cov6 = np.zeros((6, 6), dtype=np.float64)
        cov6[0, 0] = self.P[0, 0]  # x
        cov6[0, 1] = self.P[0, 1]
        cov6[1, 0] = self.P[1, 0]
        cov6[1, 1] = self.P[1, 1]  # y
        cov6[5, 5] = self.P[2, 2]  # yaw
        cov6[2, 2] = 1e-9          # z (strictly 2D planar)
        cov6[3, 3] = 1e-9          # roll
        cov6[4, 4] = 1e-9          # pitch
        return cov6

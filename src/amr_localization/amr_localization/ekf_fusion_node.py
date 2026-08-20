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

"""ROS 2 Node wrapping EKFCore to fuse wheel odometry and IMU for AMR localization."""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster

from amr_localization.ekf_core import EKFCore, normalize_angle


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert a planar yaw angle into a ROS geometry_msgs/Quaternion."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class EkfFusionNode(Node):
    """Fuses raw wheel encoder odometry and IMU inertial measurements into clean 2D planar odometry."""

    def __init__(self):
        super().__init__('ekf_fusion_node')

        # Declare parameters
        self.declare_parameter('robot_name', 'bcr_bot_amr1')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('two_d_mode', True)
        self.declare_parameter('q_pos', 1e-3)
        self.declare_parameter('q_theta', 1e-3)
        self.declare_parameter('q_v', 0.5)
        self.declare_parameter('q_omega', 0.5)
        self.declare_parameter('r_v_wheel', 0.05)
        self.declare_parameter('r_omega_wheel', 0.05)
        self.declare_parameter('r_omega_imu', 0.01)
        self.declare_parameter('slip_threshold', 1.5)
        self.declare_parameter('slip_scale', 50.0)

        self.robot_name = self.get_parameter('robot_name').value
        publish_rate = float(self.get_parameter('publish_rate').value)
        self.two_d_mode = bool(self.get_parameter('two_d_mode').value)

        # Coordinate frames
        self.odom_frame = f'{self.robot_name}/odom'
        self.base_frame = f'{self.robot_name}/base_footprint'

        # Initialize EKF Core Engine
        self.ekf = EKFCore(
            q_pos=float(self.get_parameter('q_pos').value),
            q_theta=float(self.get_parameter('q_theta').value),
            q_v=float(self.get_parameter('q_v').value),
            q_omega=float(self.get_parameter('q_omega').value),
            r_v_wheel=float(self.get_parameter('r_v_wheel').value),
            r_omega_wheel=float(self.get_parameter('r_omega_wheel').value),
            r_omega_imu=float(self.get_parameter('r_omega_imu').value),
            slip_threshold=float(self.get_parameter('slip_threshold').value),
            slip_scale=float(self.get_parameter('slip_scale').value),
        )

        # Timestamp tracking
        self.last_imu_time: Optional[float] = None
        self.last_wheel_time: Optional[float] = None
        self.latest_a_x: float = 0.0

        # QoS Profiles
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        odom_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Subscriptions
        self.sub_wheel = self.create_subscription(
            Odometry,
            f'/{self.robot_name}/wheel_odom',
            self._wheel_callback,
            odom_qos,
        )
        self.sub_imu = self.create_subscription(
            Imu,
            f'/{self.robot_name}/imu',
            self._imu_callback,
            sensor_qos,
        )

        # Publishers
        self.pub_odom = self.create_publisher(
            Odometry,
            f'/{self.robot_name}/odom',
            odom_qos,
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        # High-frequency publishing timer
        timer_period = 1.0 / max(1.0, publish_rate)
        self.timer = self.create_timer(timer_period, self._publish_estimate)

        self.get_logger().info(
            f'[{self.robot_name.upper()}_EKF] Initialized: odom_frame={self.odom_frame}, '
            f'base_frame={self.base_frame}, publish_rate={publish_rate:.1f}Hz'
        )

    def _imu_callback(self, msg: Imu):
        """High-frequency IMU prediction step."""
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_imu_time is None:
            self.last_imu_time = current_time
            return

        dt = current_time - self.last_imu_time
        self.last_imu_time = current_time

        if dt <= 0.0 or dt > 1.0:
            return

        self.latest_a_x = msg.linear_acceleration.x
        omega_z = msg.angular_velocity.z

        # Inertial propagation
        self.ekf.predict(dt, a_x_imu=self.latest_a_x, omega_z_imu=omega_z)

    def _wheel_callback(self, msg: Odometry):
        """Measurement update step from wheel encoder odometry."""
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = None
        if self.last_wheel_time is not None:
            dt = current_time - self.last_wheel_time
        self.last_wheel_time = current_time

        v_wheel = msg.twist.twist.linear.x
        omega_wheel = msg.twist.twist.angular.z

        is_slip = self.ekf.update_wheel(
            v_wheel=v_wheel,
            omega_wheel=omega_wheel,
            a_x_imu=self.latest_a_x,
            dt_wheel=dt,
        )
        if is_slip:
            self.get_logger().debug(f'[{self.robot_name.upper()}_EKF] Wheel slip detected and mitigated.')

    def _publish_estimate(self):
        """Publish the fused state estimate on /<robot_name>/odom and broadcast TF."""
        now = self.get_clock().now()
        x, y, yaw = self.ekf.get_pose()
        v, omega = self.ekf.get_twist()
        cov6 = self.ekf.get_covariance_6x6()

        quat = yaw_to_quaternion(yaw)

        # 1. Publish nav_msgs/Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = quat
        odom_msg.pose.covariance = cov6.flatten().tolist()

        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = omega
        odom_msg.twist.covariance = cov6.flatten().tolist()

        self.pub_odom.publish(odom_msg)

        # 2. Broadcast dynamic TF: odom -> base_footprint
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation = quat

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = EkfFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Relay per-robot namespaced /bcr_bot_amrX/tf and tf_static onto global /tf and /tf_static."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class TFRelay(Node):
    def __init__(self):
        super().__init__('tf_relay')
        self.declare_parameter('robot_name', '')
        self.robot_name = self.get_parameter('robot_name').value

        if not self.robot_name:
            raise ValueError('robot_name parameter must be specified')

        dynamic_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        static_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.global_tf_pub = self.create_publisher(TFMessage, '/tf', dynamic_qos)
        self.global_tf_static_pub = self.create_publisher(TFMessage, '/tf_static', static_qos)

        self.create_subscription(
            TFMessage,
            f'/{self.robot_name}/tf',
            self.tf_callback,
            dynamic_qos,
        )
        self.create_subscription(
            TFMessage,
            f'/{self.robot_name}/tf_static',
            self.tf_static_callback,
            static_qos,
        )

        self.get_logger().info(f'[TF_RELAY] Active for robot: {self.robot_name}')

    def tf_callback(self, msg: TFMessage):
        self.global_tf_pub.publish(msg)

    def tf_static_callback(self, msg: TFMessage):
        self.global_tf_static_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TFRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

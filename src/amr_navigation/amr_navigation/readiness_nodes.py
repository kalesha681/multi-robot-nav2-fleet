#!/usr/bin/env python3
"""Readiness gates for starting namespaced Nav2 stacks without timers."""

from enum import Enum, auto
from time import monotonic

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


CLOCK_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class ClockReadinessGate(Node):
    """Publish a latched ready signal after simulation time demonstrably advances."""

    def __init__(self):
        super().__init__('clock_readiness_gate')
        ready_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.ready_publisher = self.create_publisher(Bool, '/fleet/clock_ready', ready_qos)
        self.create_subscription(Clock, '/clock', self.clock_callback, CLOCK_QOS)
        self.last_clock_ns = None
        self.ready = False

    def clock_callback(self, message):
        clock_ns = _time_to_ns(message.clock)
        if self.last_clock_ns is None:
            self.last_clock_ns = clock_ns
            self.get_logger().info(
                f'[FLEET_CLOCK] FIRST_TICK: sim={clock_ns / 1e9:.3f}')
            return

        if clock_ns < self.last_clock_ns:
            self.ready = False
            self.ready_publisher.publish(Bool(data=False))
            self.get_logger().warn(
                '[FLEET_CLOCK] RESET: simulated time moved backwards; waiting for advancement')

        if not self.ready and clock_ns > self.last_clock_ns:
            self.ready = True
            self.ready_publisher.publish(Bool(data=True))
            self.get_logger().info(
                f'[FLEET_CLOCK] READY: sim advanced to {clock_ns / 1e9:.3f}')

        self.last_clock_ns = clock_ns


class ReadinessState(Enum):
    WAITING_FOR_CLOCK = auto()
    WAITING_FOR_MAP_AND_TF = auto()
    WAITING_FOR_MANAGER = auto()
    WAITING_FOR_ACTIVE = auto()
    ACTIVE = auto()
    FAILED = auto()


class RobotReadinessCoordinator(Node):
    """Start one Nav2 namespace only after its map and TF chain are valid."""

    def __init__(self):
        super().__init__('readiness_coordinator')
        self.declare_parameter('robot_name', '')
        self.declare_parameter('global_frame', '')
        self.declare_parameter('startup_timeout_sec', 120.0)
        self.declare_parameter('map_max_age_sec', 15.0)
        self.robot_name = self.get_parameter('robot_name').value
        global_frame_param = self.get_parameter('global_frame').value
        self.startup_timeout_sec = self.get_parameter('startup_timeout_sec').value
        self.map_max_age_ns = int(self.get_parameter('map_max_age_sec').value * 1e9)

        if not self.robot_name:
            raise ValueError('robot_name must be provided')

        ready_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.map_frame = global_frame_param if global_frame_param else f'{self.robot_name}/map'
        self.base_frame = f'{self.robot_name}/base_footprint'
        self.odom_frame = f'{self.robot_name}/odom'
        self.lidar_frame = f'{self.robot_name}/two_d_lidar'
        self.ready_publisher = self.create_publisher(Bool, 'is_ready', ready_qos)
        self.confirmed_links = set()

        self.latest_clock = None
        self.latest_map = None
        self.clock_ready = False
        self.state = ReadinessState.WAITING_FOR_CLOCK
        self.started_at = monotonic()
        self.active_request_pending = False
        self.last_wait_log = 0.0

        self.create_subscription(Bool, '/fleet/clock_ready', self.clock_ready_callback, ready_qos)
        self.create_subscription(Clock, '/clock', self.clock_callback, CLOCK_QOS)
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, map_qos)

        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.startup_client = self.create_client(
            ManageLifecycleNodes, 'lifecycle_manager_navigation/manage_nodes')
        self.active_client = self.create_client(
            Trigger, 'lifecycle_manager_navigation/is_active')
        self.create_timer(0.25, self.evaluate)

        self.get_logger().info(
            f'[{self.robot_name.upper()}_READY] WAITING_FOR_CLOCK')

    def clock_ready_callback(self, message):
        self.clock_ready = message.data

    def clock_callback(self, message):
        self.latest_clock = message.clock

    def map_callback(self, message):
        self.latest_map = message

    def evaluate(self):
        if self.state in (ReadinessState.ACTIVE, ReadinessState.FAILED):
            return

        if not self.clock_ready or self.latest_clock is None:
            return

        if self.state is ReadinessState.WAITING_FOR_CLOCK:
            self.state = ReadinessState.WAITING_FOR_MAP_AND_TF
            self.started_at = monotonic()  # Reset timeout timer once simulation clock is running
            self.get_logger().info(f'[{self.robot_name.upper()}_READY] WAITING_FOR_MAP_AND_TF')

        if monotonic() - self.started_at > self.startup_timeout_sec:
            self.fail(f'timeout after {self.startup_timeout_sec:.0f}s')
            return

        if self.state is ReadinessState.WAITING_FOR_MAP_AND_TF:
            valid, reason = self.map_and_tf_valid()
            if not valid:
                if monotonic() - self.last_wait_log >= 5.0:
                    self.get_logger().info(
                        f'[{self.robot_name.upper()}_READY] WAITING: {reason}')
                    self.last_wait_log = monotonic()
                return
            self.get_logger().info(f'[{self.robot_name.upper()}_READY] ALL_PREREQUISITES_CONFIRMED')
            self.state = ReadinessState.WAITING_FOR_MANAGER

        if self.state is ReadinessState.WAITING_FOR_MANAGER:
            if not self.startup_client.service_is_ready():
                return
            request = ManageLifecycleNodes.Request()
            request.command = ManageLifecycleNodes.Request.STARTUP
            self.startup_client.call_async(request).add_done_callback(self.startup_response)
            self.state = ReadinessState.WAITING_FOR_ACTIVE
            self.get_logger().info(f'[{self.robot_name.upper()}_NAV2] STARTUP_REQUESTED')

        if (
            self.state is ReadinessState.WAITING_FOR_ACTIVE
            and not self.active_request_pending
            and self.active_client.service_is_ready()
        ):
            self.active_request_pending = True
            active_future = self.active_client.call_async(Trigger.Request())
            active_future.add_done_callback(self.active_response)

    def map_and_tf_valid(self):
        # 1. Check wheel odometry TF (odom -> base_footprint)
        if not self.tf_buffer.can_transform(self.odom_frame, self.base_frame, Time()):
            return False, f'waiting for odometry TF ({self.odom_frame} -> {self.base_frame})'
        if 'odom' not in self.confirmed_links:
            self.confirmed_links.add('odom')
            self.get_logger().info(f'[{self.robot_name.upper()}_READY] TF_CONFIRMED: {self.odom_frame} -> {self.base_frame}')

        # 2. Check sensor link TF (base_footprint -> two_d_lidar)
        if not self.tf_buffer.can_transform(self.base_frame, self.lidar_frame, Time()):
            return False, f'waiting for LiDAR sensor TF ({self.base_frame} -> {self.lidar_frame})'
        if 'lidar' not in self.confirmed_links:
            self.confirmed_links.add('lidar')
            self.get_logger().info(f'[{self.robot_name.upper()}_READY] TF_CONFIRMED: {self.base_frame} -> {self.lidar_frame}')

        # 3. Check SLAM map reception
        if self.latest_map is None:
            return False, 'waiting for SLAM map'
        if self.latest_map.header.frame_id != self.map_frame:
            return False, f"map frame mismatch: expected '{self.map_frame}', got '{self.latest_map.header.frame_id}'"
        info = self.latest_map.info
        if not info.width or not info.height or info.resolution <= 0.0:
            return False, 'map has invalid geometry'
        if not any(cell >= 0 for cell in self.latest_map.data):
            return False, 'map contains no known cells'

        # 4. Check SLAM localization TF (map -> odom)
        if not self.tf_buffer.can_transform(self.map_frame, self.odom_frame, Time()):
            return False, f'waiting for SLAM localization TF ({self.map_frame} -> {self.odom_frame})'
        if 'slam' not in self.confirmed_links:
            self.confirmed_links.add('slam')
            self.get_logger().info(f'[{self.robot_name.upper()}_READY] TF_CONFIRMED: {self.map_frame} -> {self.odom_frame}')

        # 5. Check full chain (map -> base_footprint)
        if not self.tf_buffer.can_transform(self.map_frame, self.base_frame, Time()):
            return False, f'full TF chain incomplete ({self.map_frame} -> {self.base_frame})'
        if 'full_chain' not in self.confirmed_links:
            self.confirmed_links.add('full_chain')
            self.get_logger().info(f'[{self.robot_name.upper()}_READY] TF_CONFIRMED: full chain {self.map_frame} -> {self.base_frame}')

        return True, ''

    def startup_response(self, future):
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().warn(f'[{self.robot_name.upper()}_NAV2] STARTUP call error ({error}), retrying...')
            self.state = ReadinessState.WAITING_FOR_MANAGER
            return
        if not response.success:
            self.get_logger().warn(f'[{self.robot_name.upper()}_NAV2] STARTUP rejected by manager, retrying...')
            self.state = ReadinessState.WAITING_FOR_MANAGER
            return
        self.get_logger().info(f'[{self.robot_name.upper()}_NAV2] STARTUP_SUCCEEDED')

    def active_response(self, future):
        self.active_request_pending = False
        try:
            response = future.result()
        except Exception:
            return
        if not response.success:
            return
        self.state = ReadinessState.ACTIVE
        self.ready_publisher.publish(Bool(data=True))
        self.get_logger().info(f'[{self.robot_name.upper()}_NAV2] LIFECYCLE_ACTIVE - ROBOT_READY')

    def fail(self, reason):
        self.state = ReadinessState.FAILED
        self.ready_publisher.publish(Bool(data=False))
        self.get_logger().error(f'[{self.robot_name.upper()}_READY] FAILED: {reason}')


def _time_to_ns(stamp: TimeMsg):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def clock_gate_main(args=None):
    rclpy.init(args=args)
    node = ClockReadinessGate()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def robot_coordinator_main(args=None):
    rclpy.init(args=args)
    node = RobotReadinessCoordinator()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

import time
import math
import unittest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from amr_msgs.msg import SensorHealth, SafetyStatus
from amr_safety.safety_override_node import SafetyOverrideNode


class TestSafetyOverrideNodeLive(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.safety_node = SafetyOverrideNode()

        # Create Test Harness Node to publish inputs and listen to outputs
        self.harness = Node('safety_test_harness')
        self.cmd_nav_pub = self.harness.create_publisher(Twist, '/bcr_bot_amr1/cmd_vel_nav', 10)
        self.scan_pub = self.harness.create_publisher(LaserScan, '/bcr_bot_amr1/scan', 10)
        self.health_pub = self.harness.create_publisher(SensorHealth, '/bcr_bot_amr1/sensor_health', 10)
        self.odom_pub = self.harness.create_publisher(Odometry, '/bcr_bot_amr1/odom', 10)

        self.received_cmd_vel = []
        self.received_status = []

        self.cmd_sub = self.harness.create_subscription(
            Twist, '/bcr_bot_amr1/cmd_vel', lambda msg: self.received_cmd_vel.append(msg), 10
        )
        self.status_sub = self.harness.create_subscription(
            SafetyStatus, '/bcr_bot_amr1/safety_status', lambda msg: self.received_status.append(msg), 10
        )

    def tearDown(self):
        self.safety_node.destroy_node()
        self.harness.destroy_node()

    def _spin_both(self, duration_sec: float = 0.1):
        start = time.time()
        while time.time() - start < duration_sec:
            rclpy.spin_once(self.safety_node, timeout_sec=0.01)
            rclpy.spin_once(self.harness, timeout_sec=0.01)

    def _publish_scan(self, distance: float, num_points: int = 360):
        scan = LaserScan()
        scan.header.stamp = self.harness.get_clock().now().to_msg()
        scan.header.frame_id = 'bcr_bot_amr1/base_link'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = (2.0 * math.pi) / num_points
        scan.range_min = 0.1
        scan.range_max = 15.0
        scan.ranges = [float(distance)] * num_points
        self.scan_pub.publish(scan)

    def _publish_health(self, scan_ok: bool = True, imu_ok: bool = True):
        health = SensorHealth()
        health.header.stamp = self.harness.get_clock().now().to_msg()
        health.robot_id = 'bcr_bot_amr1'
        health.scan_healthy = scan_ok
        health.imu_healthy = imu_ok
        self.health_pub.publish(health)

    def _publish_cmd_nav(self, vx: float = 0.5, wz: float = 0.0):
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.angular.z = float(wz)
        self.cmd_nav_pub.publish(cmd)

    def test_01_clear_passthrough(self):
        """Test Case 1: Obstacle far away (3.0m) -> Passthrough full velocity."""
        self._publish_health(True, True)
        self._publish_scan(3.0)
        self._publish_cmd_nav(0.6, 0.1)
        self._spin_both(0.15)

        self.assertGreater(len(self.received_cmd_vel), 0)
        latest_cmd = self.received_cmd_vel[-1]
        self.assertAlmostEqual(latest_cmd.linear.x, 0.6, places=2)
        self.assertAlmostEqual(latest_cmd.angular.z, 0.1, places=2)
        self.assertEqual(self.safety_node.current_state.value, 'CLEAR')

    def test_02_dynamic_slowdown(self):
        """Test Case 2: Obstacle in slowdown zone (0.65m) -> Attenuated velocity."""
        self._publish_health(True, True)
        self._publish_scan(0.65)
        self._publish_cmd_nav(0.6, 0.0)
        self._spin_both(0.15)

        self.assertGreater(len(self.received_cmd_vel), 0)
        latest_cmd = self.received_cmd_vel[-1]
        self.assertTrue(0.0 < latest_cmd.linear.x < 0.6)
        self.assertEqual(self.safety_node.current_state.value, 'DYNAMIC_SLOWDOWN')

    def test_03_emergency_stop_and_hysteresis(self):
        """Test Case 3 & 4: Obstacle breaches d_safe -> E-STOP. Backing off slightly retains E-STOP until release margin."""
        # 1. Breach lethal margin (0.38m <= d_safe ~ 0.475m)
        self._publish_health(True, True)
        self._publish_scan(0.38)
        self._publish_cmd_nav(0.5, 0.0)
        self._spin_both(0.15)

        self.assertEqual(self.safety_node.current_state.value, 'OBSTACLE_WITHIN_SAFETY_MARGIN')
        self.assertTrue(self.safety_node.safety_override_active if hasattr(self.safety_node, 'safety_override_active') else self.safety_node.current_state.value == 'OBSTACLE_WITHIN_SAFETY_MARGIN')
        self.assertAlmostEqual(self.received_cmd_vel[-1].linear.x, 0.0, places=2)

        # 2. Hysteresis check: Move obstacle to 0.52m (above d_safe=0.475m, but below d_safe+release=0.625m)
        self._publish_health(True, True)
        self._publish_scan(0.52)
        self._publish_cmd_nav(0.5, 0.0)
        self._spin_both(0.15)

        # Must REMAIN in emergency stop due to hysteresis!
        self.assertEqual(self.safety_node.current_state.value, 'OBSTACLE_WITHIN_SAFETY_MARGIN')
        self.assertAlmostEqual(self.received_cmd_vel[-1].linear.x, 0.0, places=2)

        # 3. Release check: Move obstacle to 0.75m (exceeds recovery threshold)
        self._publish_health(True, True)
        self._publish_scan(0.75)
        self._publish_cmd_nav(0.5, 0.0)
        self._spin_both(0.15)

        # Resumes slowdown
        self.assertEqual(self.safety_node.current_state.value, 'DYNAMIC_SLOWDOWN')
        self.assertGreater(self.received_cmd_vel[-1].linear.x, 0.0)

    def test_04_sensor_fault_veto(self):
        """Test Case 5: Sensor health degraded -> Immediate fail-safe e-stop."""
        self._publish_health(scan_ok=False, imu_ok=True)
        self._publish_scan(3.0)
        self._publish_cmd_nav(0.5, 0.0)
        self._spin_both(0.15)

        self.assertEqual(self.safety_node.current_state.value, 'SENSOR_FAULT')
        self.assertAlmostEqual(self.received_cmd_vel[-1].linear.x, 0.0, places=2)

    def test_05_stale_command_watchdog(self):
        """Test Case 6: Navigation command silence > 0.20s -> Stale command zeroing."""
        # 1. Send active command
        self._publish_health(True, True)
        self._publish_scan(3.0)
        self._publish_cmd_nav(0.5, 0.0)
        self._spin_both(0.10)
        self.assertEqual(self.safety_node.current_state.value, 'CLEAR')

        # 2. Stop publishing commands and wait 0.25s
        time.sleep(0.25)
        self._publish_health(True, True)
        self._publish_scan(3.0)
        self._spin_both(0.10)

        # Command watchdog should trigger STALE_COMMAND and publish 0
        self.assertEqual(self.safety_node.current_state.value, 'STALE_COMMAND')
        self.assertAlmostEqual(self.received_cmd_vel[-1].linear.x, 0.0, places=2)


if __name__ == '__main__':
    unittest.main()

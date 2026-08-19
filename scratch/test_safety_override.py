import unittest
import math
import time
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from amr_msgs.msg import SensorHealth, SafetyStatus
from amr_safety.safety_override_node import SafetyOverrideNode

class TestSafetyOverride(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = SafetyOverrideNode()

    def tearDown(self):
        self.node.destroy_node()

    def _create_scan(self, distance: float, num_points: int = 360) -> LaserScan:
        scan = LaserScan()
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = (2 * math.pi) / num_points
        scan.range_min = 0.1
        scan.range_max = 15.0
        scan.ranges = [float(distance)] * num_points
        return scan

    def test_clear_motion(self):
        """When obstacles are far away (e.g. 5.0m), command should pass through unchanged."""
        # 1. Update healthy sensor & recent scan
        self.node.scan_callback(self._create_scan(5.0))
        health = SensorHealth()
        health.laser_healthy = True
        health.imu_healthy = True
        self.node.sensor_health_callback(health)

        # 2. Send forward motion command
        cmd = Twist()
        cmd.linear.x = 0.6
        cmd.angular.z = 0.1
        self.node.cmd_nav_callback(cmd)

        self.node.evaluate_safety_loop()

        self.assertFalse(self.node.e_stop_active)
        self.assertEqual(self.node.safety_reason, 'CLEAR')
        self.assertAlmostEqual(self.node.current_speed_limit, 0.6, places=2)

    def test_emergency_stop_on_close_obstacle(self):
        """When an obstacle breaches lethal stopping distance, e-stop must engage."""
        # At v = 0.6 m/s, d_safe = 0.5 * 0.6^2 + 0.35 = 0.18 + 0.35 = 0.53m
        # If obstacle is at 0.40m, it's inside d_safe -> E-STOP
        self.node.scan_callback(self._create_scan(0.40))
        health = SensorHealth()
        health.laser_healthy = True
        health.imu_healthy = True
        self.node.sensor_health_callback(health)

        cmd = Twist()
        cmd.linear.x = 0.6
        self.node.cmd_nav_callback(cmd)

        self.node.evaluate_safety_loop()

        self.assertTrue(self.node.e_stop_active)
        self.assertEqual(self.node.safety_reason, 'OBSTACLE_WITHIN_SAFETY_MARGIN')
        self.assertAlmostEqual(self.node.current_speed_limit, 0.0, places=2)

    def test_dynamic_slowdown(self):
        """When obstacle is between d_safe and d_warning, speed should be attenuated."""
        # At v = 0.6 m/s: d_safe = 0.53m, d_warning = 0.53 + 0.40 = 0.93m
        # If obstacle is at 0.73m (halfway), speed limit should be ~ 0.30 m/s
        self.node.scan_callback(self._create_scan(0.73))
        health = SensorHealth()
        health.laser_healthy = True
        health.imu_healthy = True
        self.node.sensor_health_callback(health)

        cmd = Twist()
        cmd.linear.x = 0.6
        self.node.cmd_nav_callback(cmd)

        self.node.evaluate_safety_loop()

        self.assertFalse(self.node.e_stop_active)
        self.assertEqual(self.node.safety_reason, 'DYNAMIC_SLOWDOWN')
        self.assertTrue(0.0 < self.node.current_speed_limit < 0.6)

    def test_sensor_fault_fail_safe(self):
        """If sensor health reports a failure, safety node must trigger SENSOR_FAULT e-stop."""
        self.node.scan_callback(self._create_scan(5.0))
        health = SensorHealth()
        health.laser_healthy = False  # Laser degraded/fault
        health.imu_healthy = True
        self.node.sensor_health_callback(health)

        cmd = Twist()
        cmd.linear.x = 0.5
        self.node.cmd_nav_callback(cmd)

        self.node.evaluate_safety_loop()

        self.assertTrue(self.node.e_stop_active)
        self.assertEqual(self.node.safety_reason, 'SENSOR_FAULT')
        self.assertAlmostEqual(self.node.current_speed_limit, 0.0, places=2)

if __name__ == '__main__':
    unittest.main()

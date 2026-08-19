import unittest
import math
import rclpy
from sensor_msgs.msg import LaserScan, Imu
from amr_msgs.msg import SensorHealth
from amr_bsp.sensor_validator_node import SensorValidatorNode

class TestSensorValidator(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = SensorValidatorNode()

    def tearDown(self):
        self.node.destroy_node()

    def test_valid_imu(self):
        """Valid IMU message should pass validation without rejection."""
        imu = Imu()
        imu.angular_velocity.x = 0.1
        imu.angular_velocity.y = 0.05
        imu.angular_velocity.z = 0.5  # Well below max_omega (5.0 rad/s)
        imu.linear_acceleration.x = 0.2
        imu.linear_acceleration.y = 0.1
        imu.linear_acceleration.z = 9.81  # Normal gravity

        self.node.imu_callback(imu)
        self.assertTrue(self.node.last_imu_healthy)
        self.assertEqual(self.node.rejected_imu_count, 0)

    def test_nan_imu_rejection(self):
        """IMU with NaN angular velocity should be rejected."""
        imu = Imu()
        imu.angular_velocity.x = float('nan')
        imu.angular_velocity.y = 0.0
        imu.angular_velocity.z = 0.0

        self.node.imu_callback(imu)
        self.assertFalse(self.node.last_imu_healthy)
        self.assertEqual(self.node.rejected_imu_count, 1)

    def test_excessive_angular_rate_rejection(self):
        """IMU with angular rate > 5.0 rad/s should be rejected."""
        imu = Imu()
        imu.angular_velocity.z = 12.0  # Exceeds max 5.0 rad/s

        self.node.imu_callback(imu)
        self.assertFalse(self.node.last_imu_healthy)
        self.assertEqual(self.node.rejected_imu_count, 1)

    def test_valid_laserscan(self):
        """Valid LaserScan should pass validation."""
        scan = LaserScan()
        scan.range_min = 0.1
        scan.range_max = 10.0
        scan.ranges = [2.5] * 360  # 100% valid beams

        self.node.scan_callback(scan)
        self.assertTrue(self.node.last_scan_healthy)
        self.assertEqual(self.node.rejected_scan_count, 0)

    def test_insufficient_valid_beams_rejection(self):
        """LaserScan with fewer than 10% valid beams should be flagged."""
        scan = LaserScan()
        scan.range_min = 0.1
        scan.range_max = 10.0
        scan.ranges = [float('inf')] * 355 + [2.0] * 5  # Only 5/360 valid (< 10%)

        self.node.scan_callback(scan)
        self.assertFalse(self.node.last_scan_healthy)
        self.assertEqual(self.node.rejected_scan_count, 1)

if __name__ == '__main__':
    unittest.main()

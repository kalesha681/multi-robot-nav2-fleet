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

"""Event-driven entity spawner that gates execution on /fleet/clock_ready signal."""

import sys
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class ClockReadyWatcher(Node):
    """Watches for latched /fleet/clock_ready signal before allowing entity spawner to proceed."""

    def __init__(self):
        super().__init__('clock_ready_watcher')
        self.is_ready = False
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.sub = self.create_subscription(Bool, '/fleet/clock_ready', self._callback, qos)

    def _callback(self, msg: Bool):
        if msg.data:
            self.is_ready = True


def main():
    rclpy.init()
    watcher = ClockReadyWatcher()

    start_monotonic = watcher.get_clock().now()
    # Spin until /fleet/clock_ready is confirmed or safety timeout reached
    while rclpy.ok() and not watcher.is_ready:
        rclpy.spin_once(watcher, timeout_sec=0.1)
        elapsed_ns = (watcher.get_clock().now() - start_monotonic).nanoseconds
        if elapsed_ns > 45e9:  # 45s safety ceiling
            watcher.get_logger().warn('ClockReadyWatcher: 45s timeout on /fleet/clock_ready; proceeding with spawn attempt.')
            break

    watcher.get_logger().info('ClockReadyWatcher: Simulation clock verified active. Dispatching spawner...')
    watcher.destroy_node()
    rclpy.shutdown()

    # Invoke ros_gz_sim create with forwarded command-line arguments
    create_cmd = ['ros2', 'run', 'ros_gz_sim', 'create'] + sys.argv[1:]
    result = subprocess.run(create_cmd)
    sys.exit(result.returncode)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import time


STATUS_NAMES = {
    0: 'UNKNOWN',
    1: 'ACCEPTED',
    2: 'EXECUTING',
    3: 'CANCELING',
    4: 'SUCCEEDED',
    5: 'CANCELED',
    6: 'ABORTED',
}

class MissionManagerNode(Node):
    def __init__(self):
        super().__init__('mission_manager_node', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])
        self.amr1_client = ActionClient(self, NavigateToPose, '/bcr_bot_amr1/navigate_to_pose')
        self.amr2_client = ActionClient(self, NavigateToPose, '/bcr_bot_amr2/navigate_to_pose')

        self.amr1_done = False
        self.amr2_done = False

    def send_goals(self):
        self.get_logger().info('Waiting for action servers...')
        while not self.amr1_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for AMR-1 NavigateToPose server...')
        while not self.amr2_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for AMR-2 NavigateToPose server...')
        self.get_logger().info('Action servers available, sending goals...')

        # Goal for AMR-1
        goal_amr1 = NavigateToPose.Goal()
        goal_amr1.pose.header.frame_id = 'bcr_bot_amr1/map'
        goal_amr1.pose.header.stamp = self.get_clock().now().to_msg()
        goal_amr1.pose.pose.position.x = 1.0
        goal_amr1.pose.pose.position.y = 1.0
        goal_amr1.pose.pose.orientation.w = 1.0

        # Goal for AMR-2
        goal_amr2 = NavigateToPose.Goal()
        goal_amr2.pose.header.frame_id = 'bcr_bot_amr2/map'
        goal_amr2.pose.header.stamp = self.get_clock().now().to_msg()
        goal_amr2.pose.pose.position.x = 2.0
        goal_amr2.pose.pose.position.y = 1.0
        goal_amr2.pose.pose.orientation.w = 1.0

        self.start_time = time.time()

        self.get_logger().info('Sending goal to AMR-1')
        self.future_amr1 = self.amr1_client.send_goal_async(goal_amr1)
        self.future_amr1.add_done_callback(self.amr1_goal_response_callback)

        self.get_logger().info('Sending goal to AMR-2')
        self.future_amr2 = self.amr2_client.send_goal_async(goal_amr2)
        self.future_amr2.add_done_callback(self.amr2_goal_response_callback)

    def amr1_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('AMR-1 goal rejected')
            self.amr1_done = True
            self.check_completion()
            return
        self.get_logger().info('AMR-1 goal accepted')
        self.amr1_result_future = goal_handle.get_result_async()
        self.amr1_result_future.add_done_callback(self.amr1_result_callback)

    def amr2_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('AMR-2 goal rejected')
            self.amr2_done = True
            self.check_completion()
            return
        self.get_logger().info('AMR-2 goal accepted')
        self.amr2_result_future = goal_handle.get_result_async()
        self.amr2_result_future.add_done_callback(self.amr2_result_callback)

    def amr1_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        elapsed = time.time() - self.start_time
        self.get_logger().info(
            f'AMR-1 finished with status: {STATUS_NAMES.get(status, status)} '
            f'({status}) in {elapsed:.2f} seconds')
        self.amr1_done = True
        self.check_completion()

    def amr2_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        elapsed = time.time() - self.start_time
        self.get_logger().info(
            f'AMR-2 finished with status: {STATUS_NAMES.get(status, status)} '
            f'({status}) in {elapsed:.2f} seconds')
        self.amr2_done = True
        self.check_completion()

    def check_completion(self):
        if self.amr1_done and self.amr2_done:
            self.get_logger().info('Both AMRs have completed their missions.')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    node.send_goals()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()

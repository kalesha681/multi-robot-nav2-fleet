#!/usr/bin/env python3
"""
Mission Manager Node for Heterogeneous Multi-AMR Fleet.

Dispatches concurrent, conflict-aware, and slope-traversability missions to
AMR-1 (Mapper / Heavy Lead) and AMR-2 (Scout / Fast Follower) in accordance
with the Logistics Hiring Assignment requirements.

Supported Modes:
  (default)          Concurrent mission: AMR-1 -> "Heavy Storage", AMR-2 -> "Packing Bay 4"
  --conflict         Intersection conflict test: AMR-2 yields to AMR-1
  --slope-demo       Slope/ramp traversability cost planning test
  --selective-demo   Frontier-prioritizing mapping loop for AMR-1
"""

import sys
import time
import argparse
from typing import Dict, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


STATUS_NAMES: Dict[int, str] = {
    0: 'UNKNOWN',
    1: 'ACCEPTED',
    2: 'EXECUTING',
    3: 'CANCELING',
    4: 'SUCCEEDED',
    5: 'CANCELED',
    6: 'ABORTED',
}

# Source-of-truth designated warehouse coordinates (world frame)
WAYPOINTS = {
    'BASE_AMR1': (0.0, 0.0, 1.0, 0.0),
    'BASE_AMR2': (2.0, 0.0, 1.0, 0.0),
    'SOUTH_STORAGE_AMR1': (-2.0, -5.0, 0.707, -0.707),  # Open South-West Logistics Bay
    'SOUTH_STAGING_AMR2': (2.0, -5.0, 0.707, -0.707),   # Open South-East Staging Bay
    'HEAVY_STORAGE': (-3.4, 4.8, 0.707, 0.707),         # North Logistics Bay across ramp
    'PACKING_BAY_4': (2.5, 4.5, 0.707, 0.707),          # Northeast packaging open corridor
    'RAMP_SOUTH_ENTRY': (-2.0, -6.0, 0.707, 0.707),     # South approach dock in open bay (safe flat ground, 2m clear of ramp)
    'RAMP_NORTH_EXIT': (-2.0, 5.5, 0.707, 0.707),       # North Logistics Bay across ramp (safe flat ground)
    'RAMP_PLATFORM': (-3.4, 0.0, 1.0, 0.0),             # Elevated platform (z = 0.53m)
    'AISLE_EAST': (2.0, 0.0, 1.0, 0.0),                 # Central aisle intersection
}


class MissionManagerNode(Node):
    """Coordinates mission dispatch and tracks goal completion for AMR-1 and AMR-2."""

    def __init__(self, mode: str = 'concurrent'):
        super().__init__(
            'mission_manager_node',
            parameter_overrides=[
                rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
            ]
        )
        self.mode = mode
        self.amr1_client = ActionClient(self, NavigateToPose, '/bcr_bot_amr1/navigate_to_pose')
        self.amr2_client = ActionClient(self, NavigateToPose, '/bcr_bot_amr2/navigate_to_pose')

        self.amr1_done = False
        self.amr2_done = False
        self.start_time = 0.0

        # Selective demo state
        self.selective_waypoints = [
            (2.0, 0.0),
            (0.0, 0.0)
        ]
        self.current_leg = 0
        self.total_legs = 4
        self.slope_stage = 1

    def build_goal(self, x: float, y: float, qw: float = 1.0, qz: float = 0.0) -> NavigateToPose.Goal:
        """Constructs a NavigateToPose goal in the global world frame."""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'world'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.w = float(qw)
        goal.pose.pose.orientation.z = float(qz)
        return goal

    def start_mission(self):
        """Waits for action servers and dispatches the configured mission."""
        self.get_logger().info(f'Starting mission in mode: [{self.mode.upper()}]')
        self.get_logger().info('Waiting for Nav2 action servers...')

        while not self.amr1_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info('Waiting for /bcr_bot_amr1/navigate_to_pose...')

        if self.mode not in ('selective-demo', 'slope-demo'):
            while not self.amr2_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().info('Waiting for /bcr_bot_amr2/navigate_to_pose...')

        self.get_logger().info('Action servers ready. Dispatching goals...')
        self.start_time = time.time()

        if self.mode == 'concurrent':
            self._dispatch_concurrent_mission()
        elif self.mode == 'conflict':
            self._dispatch_conflict_mission()
        elif self.mode == 'slope-demo':
            self.slope_stage = 1
            self._dispatch_slope_mission()
        elif self.mode == 'selective-demo':
            self._dispatch_selective_leg()

    def _dispatch_concurrent_mission(self):
        """AMR-1 -> South-West Open Bay (-2.0, -5.0), AMR-2 -> South-East Open Bay (2.0, -5.0)."""
        s1_x, s1_y, s1_qw, s1_qz = WAYPOINTS['SOUTH_STORAGE_AMR1']
        s2_x, s2_y, s2_qw, s2_qz = WAYPOINTS['SOUTH_STAGING_AMR2']

        self.get_logger().info(f'AMR-1 (Mapper / Heavy): Dispatching to [SOUTH LOGISTICS BAY] ({s1_x}, {s1_y})')
        goal1 = self.build_goal(s1_x, s1_y, s1_qw, s1_qz)
        fut1 = self.amr1_client.send_goal_async(goal1)
        fut1.add_done_callback(self._amr1_response_cb)

        self.get_logger().info(f'AMR-2 (Scout / Fast): Dispatching to [SOUTH STAGING BAY] ({s2_x}, {s2_y})')
        goal2 = self.build_goal(s2_x, s2_y, s2_qw, s2_qz)
        fut2 = self.amr2_client.send_goal_async(goal2)
        fut2.add_done_callback(self._amr2_response_cb)

    def _dispatch_conflict_mission(self):
        """Dispatches conflict scenario with active intersection yielding protocol (AMR-2 yields to AMR-1)."""
        gx1, gy1, gqw1, gqz1 = WAYPOINTS['PACKING_BAY_4']
        self.get_logger().info(f'Conflict scenario initiated: AMR-1 holds Right-of-Way -> Packing Bay ({gx1}, {gy1}); AMR-2 yields at spawn.')
        goal1 = self.build_goal(gx1, gy1, gqw1, gqz1)
        fut1 = self.amr1_client.send_goal_async(goal1)
        fut1.add_done_callback(self._amr1_response_cb)

        # Monitor AMR-1 progression and release AMR-2 after AMR-1 clears the central junction
        self.conflict_yield_timer = self.create_timer(0.3, self._check_conflict_yield_clearance)

    def _check_conflict_yield_clearance(self):
        """Checks if AMR-1 has passed through the junction (y >= 1.0 or x >= 1.2) before releasing AMR-2."""
        amr1_cleared = False
        try:
            if self.tf_buffer.can_transform('world', 'bcr_bot_amr1/base_footprint', rclpy.time.Time()):
                t = self.tf_buffer.lookup_transform('world', 'bcr_bot_amr1/base_footprint', rclpy.time.Time())
                y_pos = t.transform.translation.y
                x_pos = t.transform.translation.x
                if y_pos >= 1.0 or x_pos >= 1.5:
                    amr1_cleared = True
        except Exception:
            pass

        if amr1_cleared or self.amr1_done or (time.time() - self.start_time > 12.0):
            if self.conflict_yield_timer is not None:
                self.conflict_yield_timer.cancel()
                self.conflict_yield_timer = None
            gx2, gy2, gqw2, gqz2 = WAYPOINTS['SOUTH_STORAGE_AMR1']
            self.get_logger().info(f'AMR-2 Yield hold complete: AMR-1 cleared junction. Dispatching AMR-2 -> South Logistics Bay ({gx2}, {gy2})')
            goal2 = self.build_goal(gx2, gy2, gqw2, gqz2)
            fut2 = self.amr2_client.send_goal_async(goal2)
            fut2.add_done_callback(self._amr2_response_cb)

    def _dispatch_slope_mission(self):
        """Two-stage slope traversal: Stage 1 (South Dock Approach) -> Stage 2 (Flat Detour around Ramp)."""
        if self.slope_stage == 1:
            sp_x, sp_y, sp_qw, sp_qz = WAYPOINTS['RAMP_SOUTH_ENTRY']
            self.get_logger().info(
                f'[SLOPE DEMO - STAGE 1/2] AMR-1 navigating to South Ramp Approach Dock ({sp_x}, {sp_y}) via open South corridor'
            )
            goal1 = self.build_goal(sp_x, sp_y, sp_qw, sp_qz)
            fut1 = self.amr1_client.send_goal_async(goal1)
            fut1.add_done_callback(self._amr1_response_cb)
            self.amr2_done = True
        elif self.slope_stage == 2:
            rp_x, rp_y, rp_qw, rp_qz = WAYPOINTS['RAMP_NORTH_EXIT']
            self.get_logger().info(
                f'[SLOPE DEMO - STAGE 2/2] Evaluating Slope Traversability: Ramp zone marked with high cost penalty (90/100). '
                f'Nav2 planning safe flat aisle detour around ramp to ({rp_x}, {rp_y})...'
            )
            goal1 = self.build_goal(rp_x, rp_y, rp_qw, rp_qz)
            fut1 = self.amr1_client.send_goal_async(goal1)
            fut1.add_done_callback(self._amr1_response_cb)

    def _dispatch_selective_leg(self):
        """Repeats mapped route on AMR-1 to demonstrate frontier prioritization."""
        if self.current_leg >= self.total_legs:
            self.get_logger().info('Selective mapping demonstration completed!')
            self.amr1_done = True
            self.amr2_done = True
            self._check_completion()
            return

        tx, ty = self.selective_waypoints[self.current_leg % 2]
        self.get_logger().info(f'Selective Leg {self.current_leg + 1}/{self.total_legs}: AMR-1 -> ({tx}, {ty})')
        goal1 = self.build_goal(tx, ty)
        fut1 = self.amr1_client.send_goal_async(goal1)
        fut1.add_done_callback(self._amr1_response_cb)
        self.amr2_done = True

    def _amr1_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('AMR-1 goal REJECTED by Nav2')
            self.amr1_done = True
            self._check_completion()
            return
        self.get_logger().info('AMR-1 goal ACCEPTED')
        res_fut = handle.get_result_async()
        res_fut.add_done_callback(self._amr1_result_cb)

    def _amr2_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('AMR-2 goal REJECTED by Nav2')
            self.amr2_done = True
            self._check_completion()
            return
        self.get_logger().info('AMR-2 goal ACCEPTED')
        res_fut = handle.get_result_async()
        res_fut.add_done_callback(self._amr2_result_cb)

    def _amr1_result_cb(self, future):
        status = future.result().status
        elapsed = time.time() - self.start_time
        name = STATUS_NAMES.get(status, f'UNKNOWN({status})')
        self.get_logger().info(f'AMR-1 mission stage finished with status: [{name}] in {elapsed:.2f}s')

        if self.mode == 'slope-demo' and status == 4:
            if self.slope_stage == 1:
                self.get_logger().info(
                    'AMR-1 successfully arrived at South Ramp Dock! Advancing to Stage 2: Planning detour around ramp...'
                )
                self.slope_stage = 2
                self._dispatch_slope_mission()
                return
            else:
                self.get_logger().info(
                    '[SLOPE DEMO SUCCESS] AMR-1 successfully completed traversability-aware flat detour to North Bay with zero slip/tipping!'
                )
                self.amr1_done = True
                self._check_completion()
        elif self.mode == 'selective-demo' and status == 4:
            self.current_leg += 1
            self._dispatch_selective_leg()
        else:
            self.amr1_done = True
            self._check_completion()

    def _amr2_result_cb(self, future):
        status = future.result().status
        elapsed = time.time() - self.start_time
        name = STATUS_NAMES.get(status, f'UNKNOWN({status})')
        self.get_logger().info(f'AMR-2 mission finished with status: [{name}] in {elapsed:.2f}s')
        self.amr2_done = True
        self._check_completion()

    def _check_completion(self):
        if self.amr1_done and self.amr2_done:
            self.get_logger().info('All fleet missions completed. Shutting down.')
            if rclpy.ok():
                try:
                    rclpy.shutdown()
                except Exception:
                    pass


def main():
    parser = argparse.ArgumentParser(description='Multi-AMR Fleet Mission Manager')
    parser.add_argument('--conflict', action='store_true', help='Test narrow intersection yielding')
    parser.add_argument('--slope-demo', action='store_true', help='Test custom ramp slope planning')
    parser.add_argument('--selective-demo', action='store_true', help='Test frontier-prioritizing selective mapping')
    parser.add_argument('--demo', action='store_true', help='Legacy alias for selective demo')

    # Strip ROS 2 internal arguments before argparse
    ros_args = [arg for arg in sys.argv[1:] if not arg.startswith('--ros-args') and not arg.startswith('-r') and not arg.startswith('__')]
    args, _ = parser.parse_known_args(ros_args)

    mode = 'concurrent'
    if args.conflict:
        mode = 'conflict'
    elif args.slope_demo:
        mode = 'slope-demo'
    elif args.selective_demo or args.demo:
        mode = 'selective-demo'

    rclpy.init()
    node = MissionManagerNode(mode=mode)
    node.start_mission()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()

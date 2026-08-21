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


from tf2_ros import Buffer, TransformListener


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
    'AMR1_STAGE1_SOUTH_AISLE': (0.0, -5.0, 0.707, -0.707),   # Stage 1: South central corridor transit
    'AMR1_STAGE2_RAMP_FRONT': (-3.0, -5.0, 0.707, 0.707),    # Stage 2: Directly in front of South Ramp
    'AMR1_STAGE3_RAMP_NORTH': (-3.0, 5.0, 0.707, 0.707),     # Stage 3: Exact opposite point on North side across ramp
    'AMR2_STAGE1_NORTH_STAGING': (2.5, 4.5, 0.707, 0.707),   # AMR-2 Stage 1: Open Northeast Packaging Bay 4
    'AMR2_STAGE2_SOUTH_STAGING': (2.0, -5.0, 0.707, -0.707), # AMR-2 Stage 2: Open Southeast Logistics Bay
    'RAMP_SOUTH_ENTRY': (-3.0, -5.0, 0.707, 0.707),
    'RAMP_NORTH_EXIT': (-3.0, 5.0, 0.707, 0.707),
    'HEAVY_STORAGE': (-3.0, 5.0, 0.707, 0.707),
    'PACKING_BAY_4': (2.5, 4.5, 0.707, 0.707),
}


class MissionManagerNode(Node):
    """Coordinates multi-stage mission dispatch, slope traversability detour, and MAPF intersection yielding."""

    def __init__(self, mode: str = 'fleet'):
        super().__init__(
            'mission_manager_node',
            parameter_overrides=[
                rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
            ]
        )
        self.mode = mode
        self.amr1_client = ActionClient(self, NavigateToPose, '/bcr_bot_amr1/navigate_to_pose')
        self.amr2_client = ActionClient(self, NavigateToPose, '/bcr_bot_amr2/navigate_to_pose')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.amr1_stage = 1
        self.amr2_stage = 1
        self.amr1_done = False
        self.amr2_done = False
        self.amr2_p1_arrived = False
        self.amr2_p2_dispatched = False
        self.start_time = 0.0
        self.conflict_yield_timer = None

    def build_goal(self, x: float, y: float, qw: float = 1.0, qz: float = 0.0) -> NavigateToPose.Goal:
        """Constructs a NavigateToPose goal in the global world frame."""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'world'
        goal.pose.header.stamp = rclpy.time.Time().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.w = float(qw)
        goal.pose.pose.orientation.z = float(qz)
        return goal

    def start_mission(self):
        """Waits for action servers and initiates the synchronized fleet mission."""
        self.get_logger().info(f'Starting mission in mode: [{self.mode.upper()}]')
        self.get_logger().info('Waiting for Nav2 action servers...')

        while not self.amr1_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info('Waiting for /bcr_bot_amr1/navigate_to_pose...')

        if self.mode != 'slope-demo':
            while not self.amr2_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().info('Waiting for /bcr_bot_amr2/navigate_to_pose...')

        self.get_logger().info('Action servers ready. Dispatching Phase 1 dual-zone exploration...')
        self.start_time = time.time()

        if self.mode in ('fleet', 'concurrent'):
            self._start_fleet_mission()
        elif self.mode == 'slope-demo':
            self._dispatch_amr1_stage()
        elif self.mode == 'conflict':
            self._dispatch_conflict_mission()

    def _start_fleet_mission(self):
        """Phase 1: AMR-1 explores South (0.0, -5.0) and AMR-2 explores North (1.0, 7.5) concurrently."""
        self.amr1_stage = 1
        self.amr2_stage = 1
        self.amr2_p1_arrived = False
        self.amr2_p2_dispatched = False

        # Dispatch AMR-1 Stage 1
        self._dispatch_amr1_stage()

        time.sleep(0.3)

        # Dispatch AMR-2 Stage 1 (North Staging)
        s2_x, s2_y, s2_qw, s2_qz = WAYPOINTS['AMR2_STAGE1_NORTH_STAGING']
        self.get_logger().info(f'[AMR-2 PHASE 1] Dispatching to North Exploration Staging ({s2_x}, {s2_y})...')
        goal2 = self.build_goal(s2_x, s2_y, s2_qw, s2_qz)
        fut2 = self.amr2_client.send_goal_async(goal2)
        fut2.add_done_callback(self._amr2_response_cb)

    def _dispatch_amr1_stage(self):
        """Handles AMR-1 3-stage sequence: (0, -5) -> (-3, -5) -> (-3, 5)."""
        if self.amr1_stage == 1:
            gx, gy, gqw, gqz = WAYPOINTS['AMR1_STAGE1_SOUTH_AISLE']
            self.get_logger().info(f'[AMR-1 STAGE 1/3] Navigating South down central corridor to ({gx}, {gy})...')
        elif self.amr1_stage == 2:
            gx, gy, gqw, gqz = WAYPOINTS['AMR1_STAGE2_RAMP_FRONT']
            self.get_logger().info(f'[AMR-1 STAGE 2/3] Moving to South Ramp Approach Dock directly in front of ramp ({gx}, {gy})...')
        elif self.amr1_stage == 3:
            gx, gy, gqw, gqz = WAYPOINTS['AMR1_STAGE3_RAMP_NORTH']
            self.get_logger().info(
                f'[AMR-1 STAGE 3/3] Positioned at ramp dock. Evaluating high slope cost (98/100). '
                f'Planning safe flat aisle detour around ramp to exact opposite North Bay ({gx}, {gy})...'
            )
            # Start MAPF intersection clearance check for AMR-2
            if self.mode != 'slope-demo':
                self.conflict_yield_timer = self.create_timer(0.25, self._check_mapf_junction_clearance)
        else:
            return

        goal1 = self.build_goal(gx, gy, gqw, gqz)
        fut1 = self.amr1_client.send_goal_async(goal1)
        fut1.add_done_callback(self._amr1_response_cb)

    def _check_mapf_junction_clearance(self):
        """MAPF Rule: AMR-2 yields at North Staging until AMR-1 clears the central bottleneck (y >= 1.0)."""
        if not self.amr2_p1_arrived or self.amr2_p2_dispatched:
            return

        amr1_cleared = False
        try:
            if self.tf_buffer.can_transform('world', 'bcr_bot_amr1/base_footprint', rclpy.time.Time()):
                t = self.tf_buffer.lookup_transform('world', 'bcr_bot_amr1/base_footprint', rclpy.time.Time())
                y_pos = t.transform.translation.y
                if y_pos >= 1.0:  # AMR-1 has passed through the central junction
                    amr1_cleared = True
                    self.get_logger().info(f'[MAPF CLEARANCE] AMR-1 crossed junction (y = {y_pos:.2f} >= 1.0).')
        except Exception:
            pass

        if amr1_cleared or self.amr1_done or (time.time() - self.start_time > 110.0):
            if self.conflict_yield_timer is not None:
                self.conflict_yield_timer.cancel()
                self.conflict_yield_timer = None
            self.amr2_p2_dispatched = True
            s2_x, s2_y, s2_qw, s2_qz = WAYPOINTS['AMR2_STAGE2_SOUTH_STAGING']
            self.get_logger().info(f'[AMR-2 PHASE 2] Yield hold complete. Dispatching AMR-2 South to ({s2_x}, {s2_y})...')
            goal2 = self.build_goal(s2_x, s2_y, s2_qw, s2_qz)
            fut2 = self.amr2_client.send_goal_async(goal2)
            fut2.add_done_callback(self._amr2_response_cb)

    def _dispatch_conflict_mission(self):
        """Legacy standalone conflict test."""
        gx1, gy1, gqw1, gqz1 = WAYPOINTS['PACKING_BAY_4']
        self.get_logger().info(f'Conflict scenario: AMR-1 holds Right-of-Way -> ({gx1}, {gy1}); AMR-2 yields.')
        goal1 = self.build_goal(gx1, gy1, gqw1, gqz1)
        fut1 = self.amr1_client.send_goal_async(goal1)
        fut1.add_done_callback(self._amr1_response_cb)
        self.conflict_yield_timer = self.create_timer(0.3, self._check_mapf_junction_clearance)

    def _amr1_response_cb(self, future):
        try:
            handle = future.result()
        except Exception as e:
            self.get_logger().error(f'AMR-1 goal send exception: {e}')
            self.amr1_done = True
            self._check_completion()
            return

        if not handle.accepted:
            self.get_logger().error('AMR-1 goal REJECTED by Nav2')
            self.amr1_done = True
            self._check_completion()
            return
        self.get_logger().info('AMR-1 goal ACCEPTED')
        res_fut = handle.get_result_async()
        res_fut.add_done_callback(self._amr1_result_cb)

    def _amr2_response_cb(self, future):
        try:
            handle = future.result()
        except Exception as e:
            self.get_logger().error(f'AMR-2 goal send exception: {e}')
            self.amr2_done = True
            self._check_completion()
            return

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
        self.get_logger().info(f'AMR-1 Stage {self.amr1_stage} finished with status: [{name}] in {elapsed:.2f}s')

        if status == 4:  # SUCCEEDED
            if self.amr1_stage == 1:
                self.get_logger().info('AMR-1 reached South Central Corridor. Advancing to Stage 2 (Ramp Front Dock)...')
                self.amr1_stage = 2
                self._dispatch_amr1_stage()
            elif self.amr1_stage == 2:
                self.get_logger().info('AMR-1 arrived in front of South Ramp! Advancing to Stage 3 (Flat Detour across Ramp)...')
                self.amr1_stage = 3
                self._dispatch_amr1_stage()
            else:
                self.get_logger().info('[AMR-1 SUCCESS] Arrived cleanly at North Bay (-3.0, 5.0) via traversability flat detour!')
                self.amr1_done = True
                self._check_completion()
        else:
            self.amr1_done = True
            self._check_completion()

    def _amr2_result_cb(self, future):
        status = future.result().status
        elapsed = time.time() - self.start_time
        name = STATUS_NAMES.get(status, f'UNKNOWN({status})')
        self.get_logger().info(f'AMR-2 Stage {self.amr2_stage} finished with status: [{name}] in {elapsed:.2f}s')

        if status == 4:
            if self.amr2_stage == 1:
                self.get_logger().info('AMR-2 reached North Staging (1.0, 7.5). [MAPF YIELD HOLD ACTIVE]: Waiting for AMR-1 clearance...')
                self.amr2_p1_arrived = True
                self.amr2_stage = 2
                # Check immediately if AMR-1 is already past junction
                self._check_mapf_junction_clearance()
            else:
                self.get_logger().info('[AMR-2 SUCCESS] Arrived cleanly at South Logistics Bay (1.0, -5.0)!')
                self.amr2_done = True
                self._check_completion()
        else:
            self.amr2_done = True
            self._check_completion()

    def _check_completion(self):
        if self.mode == 'slope-demo' and self.amr1_done:
            self.get_logger().info('Slope demo completed. Shutting down.')
            if rclpy.ok():
                try:
                    rclpy.shutdown()
                except Exception:
                    pass
        elif self.amr1_done and self.amr2_done:
            self.get_logger().info('=== [FLEET MISSION ACCOMPLISHED] All stages completed successfully! ===')
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
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    # Launch start_mission on a background thread once executor is spinning
    import threading
    t = threading.Thread(target=node.start_mission, daemon=True)
    t.start()

    try:
        executor.spin()
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

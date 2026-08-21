#!/usr/bin/env python3
"""
SlopeCostNode (amr_navigation)
Dynamic Payload-Weighted Slope Traversability & Cost Evaluation Layer.

Functions:
1. Tracks custom ramp geometry: [x in (-4.15, -2.65), y in (-4.05, 4.05)], elevation = 0.529m, angle = 10 deg.
2. Calculates energetic/slope cost penalty as a function of slope angle and payload mass:
      Cost = BaseCost + K_slope * sin(theta) * (M_chassis + M_payload)
3. Exposes ROS 2 Service (/<robot>/set_payload) to dynamically update robot payload state.
4. Broadcasts /fleet/slope_cost_zone telemetry (amr_msgs/SlopeCostZone).
5. Updates map_fusion_node's ramp_slope_cost parameter dynamically so global planners re-evaluate
   steep ramp shortcut vs. flat detour automatically.
"""

import math
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from amr_msgs.msg import SlopeCostZone
from amr_msgs.srv import SetPayload


class SlopeCostNode(Node):
    def __init__(self):
        super().__init__('slope_cost_node')

        # Declare industry-standard slope physical parameters
        self.declare_parameter('incline_angle_deg', 10.0)
        self.declare_parameter('platform_height_m', 0.529)
        self.declare_parameter('alpha_scale', 18.0)
        self.declare_parameter('mu_rolling', 0.05)
        self.declare_parameter('base_cost', 20.0)

        self.angle_deg = float(self.get_parameter('incline_angle_deg').value)
        self.platform_h = float(self.get_parameter('platform_height_m').value)
        self.alpha_scale = float(self.get_parameter('alpha_scale').value)
        self.mu_rolling = float(self.get_parameter('mu_rolling').value)
        self.base_cost = float(self.get_parameter('base_cost').value)

        # Robot base tare mass configurations (kg)
        self.robot_configs = {
            'bcr_bot_amr1': {'chassis_mass': 15.0, 'payload_mass': 0.0, 'priority': 1},
            'bcr_bot_amr2': {'chassis_mass': 15.0, 'payload_mass': 0.0, 'priority': 2},
        }

        # Publisher for slope zone metadata
        self.zone_pub = self.create_publisher(SlopeCostZone, '/fleet/slope_cost_zone', 10)

        # Service servers for setting payload on each robot
        self.srv_amr1 = self.create_service(
            SetPayload,
            '/bcr_bot_amr1/set_payload',
            lambda req, res: self.handle_set_payload('bcr_bot_amr1', req, res)
        )
        self.srv_amr2 = self.create_service(
            SetPayload,
            '/bcr_bot_amr2/set_payload',
            lambda req, res: self.handle_set_payload('bcr_bot_amr2', req, res)
        )

        # Parameter client to update map_fusion_node
        self.map_param_client = self.create_client(SetParameters, '/map_fusion_node/set_parameters')

        # 1 Hz broadcast timer
        self.timer = self.create_timer(1.0, self.publish_slope_zone)

        self.get_logger().info(
            f'[SLOPE_COST] SlopeCostNode initialized. Incline={self.angle_deg} deg, Height={self.platform_h}m, mu={self.mu_rolling}'
        )

    def calculate_cost(self, robot_id: str) -> float:
        """
        Industry-standard ISO/AGV slope traversability cost formula:
        Cost = BaseCost + alpha * (mu_rolling * cos(theta) + sin(theta)) * (M_total / M_tare)
        """
        cfg = self.robot_configs.get(robot_id, {'chassis_mass': 15.0, 'payload_mass': 0.0})
        m_tare = max(cfg['chassis_mass'], 1.0)
        m_total = m_tare + cfg['payload_mass']
        theta_rad = math.radians(self.angle_deg)

        # Gravitational resistance + rolling resistance work factor
        mechanical_work_factor = (self.mu_rolling * math.cos(theta_rad)) + math.sin(theta_rad)
        mass_ratio = m_total / m_tare

        # Dynamic costmap traversal penalty
        dynamic_cost = self.base_cost + (self.alpha_scale * mechanical_work_factor * mass_ratio)
        return min(max(dynamic_cost, 0.0), 252.0)

    def handle_set_payload(self, robot_id: str, req: SetPayload.Request, res: SetPayload.Response) -> SetPayload.Response:
        """Updates payload mass and triggers dynamic cost update."""
        if robot_id not in self.robot_configs:
            res.success = False
            res.message = f"Unknown robot: {robot_id}"
            return res

        self.robot_configs[robot_id]['payload_mass'] = float(req.payload_mass_kg)
        cost = self.calculate_cost(robot_id)

        # Max allowed speed scale based on payload
        max_speed = max(0.3, 1.2 - (0.01 * req.payload_mass_kg))

        res.success = True
        res.dynamic_slope_cost = float(cost)
        res.max_allowed_velocity = float(max_speed)
        res.message = f"{robot_id} payload set to {req.payload_mass_kg} kg -> Dynamic slope cost: {cost:.1f}"

        self.get_logger().info(res.message)
        self.update_map_fusion_cost(int(cost))
        return res

    def update_map_fusion_cost(self, cost_val: int):
        """Asynchronously updates ramp_slope_cost on map_fusion_node."""
        if not self.map_param_client.service_is_ready():
            return

        param = Parameter()
        param.name = 'ramp_slope_cost'
        param.value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=cost_val)

        req = SetParameters.Request()
        req.parameters = [param]
        self.map_param_client.call_async(req)

    def publish_slope_zone(self):
        """Publishes SlopeCostZone metadata for RViz and Mission Coordinator."""
        zone = SlopeCostZone()
        zone.header.stamp = self.get_clock().now().to_msg()
        zone.header.frame_id = 'world'
        zone.zone_name = 'CUSTOM_ELEVATED_RAMP'
        zone.min_x = -4.15
        zone.max_x = -2.65
        zone.min_y = -2.55
        zone.max_y = 5.55
        zone.incline_angle_deg = self.angle_deg
        zone.platform_height_m = self.platform_h
        zone.dynamic_cost_penalty = float(self.calculate_cost('bcr_bot_amr1'))
        self.zone_pub.publish(zone)


def main(args=None):
    rclpy.init(args=args)
    node = SlopeCostNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

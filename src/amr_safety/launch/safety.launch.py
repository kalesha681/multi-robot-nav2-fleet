#!/usr/bin/env python3
import os
import ast
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    safety_pkg_dir = get_package_share_directory('amr_safety')
    config_file = os.path.join(safety_pkg_dir, 'config', 'safety_params.yaml')

    fleet_robots_str = LaunchConfiguration('fleet_robots').perform(context)
    try:
        fleet_robots = ast.literal_eval(fleet_robots_str)
        if not isinstance(fleet_robots, list):
            fleet_robots = [fleet_robots_str]
    except Exception:
        fleet_robots = [r.strip() for r in fleet_robots_str.split(',') if r.strip()]

    nodes = []
    for robot_name in fleet_robots:
        nodes.append(
            Node(
                package='amr_safety',
                executable='safety_override_node',
                name='safety_override_node',
                namespace=robot_name,
                parameters=[
                    config_file,
                    {
                        'use_sim_time': True,
                        'robot_name': robot_name,
                    }
                ],
                output='screen',
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'fleet_robots',
            default_value="['bcr_bot_amr1', 'bcr_bot_amr2']",
            description="List of robot namespaces in the fleet"
        ),
        OpaqueFunction(function=launch_setup),
    ])

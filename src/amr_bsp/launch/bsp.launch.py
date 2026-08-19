import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Sensor validator for AMR-1
        Node(
            package='amr_bsp',
            executable='sensor_validator_node',
            name='sensor_validator_amr1',
            parameters=[{
                'robot_name': 'bcr_bot_amr1',
                'max_angular_velocity_rad_s': 5.0,
                'max_linear_accel_m_s2': 20.0,
                'min_valid_beam_ratio': 0.10,
                'enable_ramp_ground_filter': True,
            }],
            output='screen',
        ),
        # Sensor validator for AMR-2
        Node(
            package='amr_bsp',
            executable='sensor_validator_node',
            name='sensor_validator_amr2',
            parameters=[{
                'robot_name': 'bcr_bot_amr2',
                'max_angular_velocity_rad_s': 5.0,
                'max_linear_accel_m_s2': 20.0,
                'min_valid_beam_ratio': 0.10,
                'enable_ramp_ground_filter': True,
            }],
            output='screen',
        ),
    ])

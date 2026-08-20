from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    namespace = 'bcr_bot_amr2'

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_frame': f'{namespace}/odom',
            'base_frame': f'{namespace}/base_footprint',
            'map_frame': f'{namespace}/map',
            'scan_topic': f'/{namespace}/scan',
            'mode': 'mapping',
            'map_name': f'/{namespace}/map',
            'position_threshold': 0.1,
            'resolution': 0.05,
            'map_update_interval': 0.5,
            'transform_publish_period': 0.02,
            'minimum_travel_distance': 0.0,
            'minimum_travel_heading': 0.0,
            # The lidar runs at 30 Hz. Async SLAM keeps only the newest scan
            # and processes a bounded 2 Hz stream to avoid a stale TF queue.
            'scan_queue_size': 1,
            'throttle_scans': 15,
            'transform_timeout': 1.0,
            'min_laser_range': 0.6,
            'max_laser_range': 16.0,
        }],
        remappings=[
            ('/map', f'/{namespace}/map'),
            ('/map_metadata', f'/{namespace}/map_metadata'),
            ('/slam_toolbox/scan_matcher_map', f'/{namespace}/slam_toolbox/scan_matcher_map'),
            (
                '/slam_toolbox/graph_visualization',
                f'/{namespace}/slam_toolbox/graph_visualization',
            ),
            ('/tf', f'/{namespace}/tf'),
        ]
    )

    return LaunchDescription([
        slam_node
    ])

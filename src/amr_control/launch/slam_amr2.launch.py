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
            'base_frame': f'{namespace}/base_link',
            'map_frame': f'{namespace}/map',
            'scan_topic': f'/{namespace}/scan',
            'mode': 'mapping',
            'map_name': f'/{namespace}/map',
            'position_threshold': 0.1,
            'resolution': 0.05,
            # The lidar runs at 30 Hz.  Async SLAM keeps only the newest scan
            # and processes a bounded 2 Hz stream to avoid a stale TF queue.
            'scan_queue_size': 1,
            'throttle_scans': 15,
            'transform_timeout': 0.5,
            # Match the Gazebo GPU LiDAR range in gz_amr2.xacro.
            'min_laser_range': 0.6,
            'max_laser_range': 16.0,
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/map', f'/{namespace}/map'),
            ('/map_metadata', f'/{namespace}/map_metadata'),
            ('/slam_toolbox/scan_matcher_map', f'/{namespace}/slam_toolbox/scan_matcher_map'),
            (
                '/slam_toolbox/graph_visualization',
                f'/{namespace}/slam_toolbox/graph_visualization',
            ),
        ]
    )

    return LaunchDescription([
        slam_node
    ])

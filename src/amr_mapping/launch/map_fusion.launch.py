import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Parameters – same defaults as in the node
    amr1_spawn_x = LaunchConfiguration('amr1_spawn_x', default='0.0')
    amr1_spawn_y = LaunchConfiguration('amr1_spawn_y', default='0.0')
    amr1_spawn_yaw = LaunchConfiguration('amr1_spawn_yaw', default='0.0')
    amr2_spawn_x = LaunchConfiguration('amr2_spawn_x', default='2.0')
    amr2_spawn_y = LaunchConfiguration('amr2_spawn_y', default='0.0')
    amr2_spawn_yaw = LaunchConfiguration('amr2_spawn_yaw', default='0.0')
    visit_threshold = LaunchConfiguration('visit_threshold', default='3')
    merge_rate_hz = LaunchConfiguration('merge_rate_hz', default='2.0')
    debug = LaunchConfiguration('debug', default='false')

    # Static transforms – published to global /tf_static for RViz & global tools
    static_tf_amr1_global = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_amr1_global',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '--x', amr1_spawn_x,
            '--y', amr1_spawn_y,
            '--z', '0.0',
            '--yaw', amr1_spawn_yaw,
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', 'world',
            '--child-frame-id', 'bcr_bot_amr1/map',
        ],
        output='log',
    )

    static_tf_amr2_global = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_amr2_global',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '--x', amr2_spawn_x,
            '--y', amr2_spawn_y,
            '--z', '0.0',
            '--yaw', amr2_spawn_yaw,
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', 'world',
            '--child-frame-id', 'bcr_bot_amr2/map',
        ],
        output='log',
    )

    # Map fusion node
    map_fusion_node = Node(
        package='amr_mapping',
        executable='map_fusion_node',
        name='map_fusion_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'amr1_spawn_x': amr1_spawn_x,
            'amr1_spawn_y': amr1_spawn_y,
            'amr1_spawn_yaw': amr1_spawn_yaw,
            'amr2_spawn_x': amr2_spawn_x,
            'amr2_spawn_y': amr2_spawn_y,
            'amr2_spawn_yaw': amr2_spawn_yaw,
            'visit_threshold': visit_threshold,
            'merge_rate_hz': merge_rate_hz,
            'world_frame_id': 'world',
            'debug': debug,
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('amr1_spawn_x', default_value='0.0'),
        DeclareLaunchArgument('amr1_spawn_y', default_value='0.0'),
        DeclareLaunchArgument('amr1_spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument('amr2_spawn_x', default_value='2.0'),
        DeclareLaunchArgument('amr2_spawn_y', default_value='0.0'),
        DeclareLaunchArgument('amr2_spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument('visit_threshold', default_value='5'),
        DeclareLaunchArgument('merge_rate_hz', default_value='1.0'),
        DeclareLaunchArgument('debug', default_value='false'),
        static_tf_amr1_global,
        static_tf_amr2_global,
        map_fusion_node
    ])

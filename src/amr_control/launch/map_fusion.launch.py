import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Parameters – same defaults as in the node
    amr1_spawn_x = LaunchConfiguration('amr1_spawn_x', default='0.0')
    amr1_spawn_y = LaunchConfiguration('amr1_spawn_y', default='0.0')
    amr1_spawn_yaw = LaunchConfiguration('amr1_spawn_yaw', default='0.0')
    amr2_spawn_x = LaunchConfiguration('amr2_spawn_x', default='4.0')
    amr2_spawn_y = LaunchConfiguration('amr2_spawn_y', default='0.0')
    amr2_spawn_yaw = LaunchConfiguration('amr2_spawn_yaw', default='0.0')
    visit_threshold = LaunchConfiguration('visit_threshold', default='5')
    merge_rate_hz = LaunchConfiguration('merge_rate_hz', default='1.0')
    debug = LaunchConfiguration('debug', default='false')

    # Static transforms – published once at launch time
    # world -> amr1/map (identity, because amr1 spawns at origin)
    static_tf_amr1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_amr1',
        arguments=[
            '0', '0', '0',  # translation
            '0', '0', '0',  # rotation (radians)
            'world',
            'bcr_bot_amr1/map',
        ],
        remappings=[('/tf_static', '/bcr_bot_amr1/tf_static')],
        output='log',
    )

    # world -> amr2/map (identity, aligned with world frame)
    static_tf_amr2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_amr2',
        arguments=[
            '0', '0', '0',
            '0', '0', '0',
            'world',
            'bcr_bot_amr2/map',
        ],
        remappings=[('/tf_static', '/bcr_bot_amr2/tf_static')],
        output='log',
    )

    # Map fusion node
    fusion_node = Node(
        package='amr_control',
        executable='map_fusion_node',
        name='map_fusion_node',
        output='screen',
        parameters=[
            {'amr1_spawn_x': amr1_spawn_x},
            {'amr1_spawn_y': amr1_spawn_y},
            {'amr1_spawn_yaw': amr1_spawn_yaw},
            {'amr2_spawn_x': amr2_spawn_x},
            {'amr2_spawn_y': amr2_spawn_y},
            {'amr2_spawn_yaw': amr2_spawn_yaw},
            {'visit_threshold': visit_threshold},
            {'merge_rate_hz': merge_rate_hz},
            {'debug': debug},
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([
        static_tf_amr1,
        static_tf_amr2,
        fusion_node,
    ])

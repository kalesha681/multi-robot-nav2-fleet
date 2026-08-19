import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    namespace = 'bcr_bot_amr2'
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    amr_navigation_dir = get_package_share_directory('amr_navigation')

    params_file = os.path.join(amr_navigation_dir, 'config', 'nav2_params_amr2.yaml')

    nav2_bringup_launch = GroupAction([
        PushRosNamespace(namespace),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': 'True',
                'params_file': params_file,
                'autostart': LaunchConfiguration('autostart'),
                'use_composition': 'False'
            }.items()
        )
    ])

    return LaunchDescription([
        DeclareLaunchArgument('autostart', default_value='false'),
        nav2_bringup_launch
    ])

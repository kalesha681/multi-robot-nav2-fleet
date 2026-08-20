import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, SetRemap


def generate_launch_description():
    namespace = 'bcr_bot_amr1'
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    amr_navigation_dir = get_package_share_directory('amr_navigation')

    params_file = os.path.join(amr_navigation_dir, 'config', 'nav2_params_amr1.yaml')

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(amr_navigation_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': 'True',
            'params_file': params_file,
            'autostart': LaunchConfiguration('autostart'),
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('autostart', default_value='false'),
        nav2_bringup_launch
    ])

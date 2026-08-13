import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    namespace = 'bcr_bot_amr1'
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    amr_control_dir = get_package_share_directory('amr_control')

    params_file = os.path.join(amr_control_dir, 'config', 'nav2_params_amr1.yaml')

    nav2_bringup_launch = GroupAction([
        PushRosNamespace(namespace),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                # navigation_launch.py does not push a namespace itself.  The
                # enclosing GroupAction supplies it for nodes and this argument
                # supplies the matching root key for RewrittenYaml.
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

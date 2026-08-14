import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    amr_sim_dir = get_package_share_directory('amr_sim')
    amr_control_dir = get_package_share_directory('amr_control')

    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(amr_sim_dir, 'launch', 'simulation.launch.py'))
    )
    spawn_amr1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(amr_sim_dir, 'launch', 'spawn_robot.launch.py')),
        launch_arguments={'robot_variant': 'amr1', 'position_x': '3.2', 'position_y': '-1.0', 'orientation_yaw': '-1.5707', 'bridge_clock': 'false'}.items()
    )
    slam_amr1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(amr_control_dir, 'launch', 'slam_amr1.launch.py'))
    )
    nav2_amr1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(amr_control_dir, 'launch', 'nav2_amr1.launch.py')),
        launch_arguments={'autostart': 'false'}.items()
    )
    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/world/default/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        remappings=[('/world/default/clock', '/clock')]
    )
    return LaunchDescription([simulation_launch, clock_bridge, spawn_amr1, slam_amr1, nav2_amr1])

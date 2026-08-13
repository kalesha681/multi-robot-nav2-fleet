import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    amr_sim_dir = get_package_share_directory('amr_sim')
    amr_control_dir = get_package_share_directory('amr_control')

    # Simulation environment
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(amr_sim_dir, 'launch', 'simulation.launch.py')
        )
    )

    # Spawn AMR-1
    spawn_amr1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(amr_sim_dir, 'launch', 'spawn_robot.launch.py')
        ),
        launch_arguments={
            'robot_variant': 'amr1',
            'position_x': '0.0',
            'position_y': '0.0',
            'orientation_yaw': '0.0',
            'bridge_clock': 'false',
        }.items()
    )

    # Spawn AMR-2
    spawn_amr2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(amr_sim_dir, 'launch', 'spawn_robot.launch.py')
        ),
        launch_arguments={
            'robot_variant': 'amr2',
            'position_x': '2.0',
            'position_y': '0.0',
            'orientation_yaw': '0.0',
            'bridge_clock': 'false',
        }.items()
    )

    # SLAM for AMR-1
    slam_amr1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(amr_control_dir, 'launch', 'slam_amr1.launch.py')
        )
    )

    # SLAM for AMR-2
    slam_amr2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(amr_control_dir, 'launch', 'slam_amr2.launch.py')
        )
    )

    # Nav2 for AMR-1
    nav2_amr1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(amr_control_dir, 'launch', 'nav2_amr1.launch.py')
        ),
        launch_arguments={'autostart': 'false'}.items(),
    )

    # Nav2 for AMR-2
    nav2_amr2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(amr_control_dir, 'launch', 'nav2_amr2.launch.py')
        ),
        launch_arguments={'autostart': 'false'}.items(),
    )

    # Clock bridge
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/world/default/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        remappings=[('/world/default/clock', '/clock')]
    )

    clock_readiness_gate = Node(
        package='amr_control',
        executable='clock_readiness_gate',
        output='screen',
    )

    readiness_amr1 = Node(
        package='amr_control',
        executable='robot_readiness_coordinator',
        namespace='bcr_bot_amr1',
        output='screen',
        parameters=[
            {'robot_name': 'bcr_bot_amr1'},
            {'map_max_age_sec': 15.0},
        ],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    readiness_amr2 = Node(
        package='amr_control',
        executable='robot_readiness_coordinator',
        namespace='bcr_bot_amr2',
        output='screen',
        parameters=[
            {'robot_name': 'bcr_bot_amr2'},
            {'map_max_age_sec': 15.0},
        ],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    return LaunchDescription([
        simulation_launch,
        clock_bridge,
        clock_readiness_gate,
        spawn_amr1,
        spawn_amr2,
        slam_amr1,
        slam_amr2,
        # Nav2 processes are launched unconfigured.  Their lifecycle managers
        # are explicitly started only after each robot has a valid SLAM map
        # and a resolvable map-to-base_link TF chain.
        nav2_amr1,
        nav2_amr2,
        readiness_amr1,
        readiness_amr2,
    ])

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    amr_bringup_dir = get_package_share_directory('amr_bringup')
    amr_sim_dir = get_package_share_directory('amr_sim')
    amr_bsp_dir = get_package_share_directory('amr_bsp')
    amr_mapping_dir = get_package_share_directory('amr_mapping')
    amr_navigation_dir = get_package_share_directory('amr_navigation')
    amr_safety_dir = get_package_share_directory('amr_safety')

    # Launch configurations
    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('use_rviz')

    # 1. Simulation Environment Bringup (Gazebo world)
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(amr_sim_dir, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={'headless': headless}.items()
    )

    # 2. Clock Bridge from Gazebo to ROS 2
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='fleet_clock_bridge',
        arguments=['/world/default/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        remappings=[('/world/default/clock', '/clock')],
        output='log'
    )

    # 3. Clock Readiness Gate (ensures monotonic sim time before Nav2 lifecycle activation)
    clock_gate_node = Node(
        package='amr_navigation',
        executable='clock_readiness_gate',
        name='clock_readiness_gate',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # 4. Spawners for AMR-1 and AMR-2
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

    # 5. Board Support Package (BSP) Sensor Validator Layer
    bsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(amr_bsp_dir, 'launch', 'bsp.launch.py')
        )
    )

    # 6. SLAM Toolbox mapping for AMR-1 and AMR-2
    slam_amr1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(amr_mapping_dir, 'launch', 'slam_amr1.launch.py')
        )
    )
    slam_amr2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(amr_mapping_dir, 'launch', 'slam_amr2.launch.py')
        )
    )

    # 7. Cooperative Map Fusion & Selective Frontier Update
    map_fusion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(amr_mapping_dir, 'launch', 'map_fusion.launch.py')
        ),
        launch_arguments={
            'amr1_spawn_x': '0.0',
            'amr1_spawn_y': '0.0',
            'amr1_spawn_yaw': '0.0',
            'amr2_spawn_x': '2.0',
            'amr2_spawn_y': '0.0',
            'amr2_spawn_yaw': '0.0',
        }.items()
    )

    # 8. Nav2 Stacks for AMR-1 and AMR-2
    nav2_amr1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(amr_navigation_dir, 'launch', 'nav2_amr1.launch.py')
        ),
        launch_arguments={'autostart': 'false'}.items()
    )
    nav2_amr2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(amr_navigation_dir, 'launch', 'nav2_amr2.launch.py')
        ),
        launch_arguments={'autostart': 'false'}.items()
    )

    # 9. Readiness Coordinators (activates Nav2 after map and TF are ready)
    coordinator_amr1_node = Node(
        package='amr_navigation',
        executable='robot_readiness_coordinator',
        name='robot_readiness_coordinator',
        namespace='bcr_bot_amr1',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_name': 'bcr_bot_amr1',
            'global_frame': 'bcr_bot_amr1/map',
            'startup_timeout_sec': 120.0,
            'map_max_age_sec': 15.0,
        }],
    )

    coordinator_amr2_node = Node(
        package='amr_navigation',
        executable='robot_readiness_coordinator',
        name='robot_readiness_coordinator',
        namespace='bcr_bot_amr2',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_name': 'bcr_bot_amr2',
            'global_frame': 'bcr_bot_amr2/map',
            'startup_timeout_sec': 120.0,
            'map_max_age_sec': 15.0,
        }],
    )

    # 10. Slope Cost Manager Node
    slope_cost_node = Node(
        package='amr_navigation',
        executable='slope_cost_node',
        name='slope_cost_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'incline_angle_deg': 10.0,
            'platform_height_m': 0.529,
            'k_slope': 1.2,
            'base_cost': 15.0,
        }],
    )

    # 11. Independent Safety Override & Dynamic Braking Layer
    safety_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(amr_safety_dir, 'launch', 'safety.launch.py')
        )
    )

    # 12. RViz2 Visualization
    rviz_config_file = os.path.join(amr_bringup_dir, 'rviz', 'fleet_navigation.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='fleet_rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false', description='Run Gazebo in headless mode'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Launch RViz2 for visualization'),
        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=os.path.join(amr_sim_dir, '..'),
        ),
        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=amr_sim_dir,
        ),
        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=os.path.join(amr_sim_dir, 'meshes'),
        ),
        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=os.path.join(amr_sim_dir, 'models'),
        ),
        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=os.path.join(amr_sim_dir, 'worlds'),
        ),
        AppendEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=os.path.join(amr_sim_dir, '..'),
        ),
        AppendEnvironmentVariable(
            name='GZ_FILE_PATH',
            value=os.path.join(amr_sim_dir, '..'),
        ),
        simulation_launch,
        clock_bridge,
        clock_gate_node,
        spawn_amr1,
        spawn_amr2,
        bsp_launch,
        slam_amr1_launch,
        slam_amr2_launch,
        map_fusion_launch,
        nav2_amr1_launch,
        nav2_amr2_launch,
        coordinator_amr1_node,
        coordinator_amr2_node,
        slope_cost_node,
        safety_launch,
        rviz_node,
    ])

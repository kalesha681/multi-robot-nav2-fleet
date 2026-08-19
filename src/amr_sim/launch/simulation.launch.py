#!/usr/bin/env python3

from os.path import join

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    AppendEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    amr_sim_path = get_package_share_directory("amr_sim")

    world_file = LaunchConfiguration(
        "world_file",
        default=join(amr_sim_path, "worlds", "small_warehouse.sdf")
    )

    gz_sim_share = get_package_share_directory("ros_gz_sim")

    position_x = LaunchConfiguration("position_x", default="0.0")
    position_y = LaunchConfiguration("position_y", default="0.0")
    orientation_yaw = LaunchConfiguration("orientation_yaw", default="0.0")

    headless = LaunchConfiguration("headless", default="false")

    camera_enabled = LaunchConfiguration("camera_enabled", default="true")
    stereo_camera_enabled = LaunchConfiguration(
        "stereo_camera_enabled", default="false"
    )
    two_d_lidar_enabled = LaunchConfiguration(
        "two_d_lidar_enabled", default="true"
    )
    odometry_source = LaunchConfiguration(
        "odometry_source", default="world"
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": PythonExpression([
                "'", world_file, " -r -s' if '", headless, "'.lower() == 'true' else '", world_file, " -r'"
            ])
        }.items()
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            "world_file",
            default_value=world_file
        ),

        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run Gazebo in headless mode (no GUI window)",
        ),

        DeclareLaunchArgument(
            "position_x",
            default_value="0.0"
        ),

        DeclareLaunchArgument(
            "position_y",
            default_value="0.0"
        ),

        DeclareLaunchArgument(
            "orientation_yaw",
            default_value="0.0"
        ),

        DeclareLaunchArgument(
            "camera_enabled",
            default_value="true"
        ),

        DeclareLaunchArgument(
            "stereo_camera_enabled",
            default_value="false"
        ),

        DeclareLaunchArgument(
            "two_d_lidar_enabled",
            default_value="true"
        ),

        DeclareLaunchArgument(
            "odometry_source",
            default_value="world"
        ),

        AppendEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=join(amr_sim_path, "models"),
        ),

        AppendEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=join(amr_sim_path, "worlds"),
        ),

        gz_sim,
    ])

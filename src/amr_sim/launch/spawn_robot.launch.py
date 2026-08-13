#!/usr/bin/env python3

from os.path import join

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    """Resolve robot_variant at launch time and create all nodes accordingly."""

    amr_sim_path = get_package_share_directory("amr_sim")

    # Resolve launch arguments to strings
    robot_variant = LaunchConfiguration("robot_variant").perform(context)
    position_x = LaunchConfiguration("position_x").perform(context)
    position_y = LaunchConfiguration("position_y").perform(context)
    orientation_yaw = LaunchConfiguration("orientation_yaw").perform(context)
    camera_enabled = LaunchConfiguration("camera_enabled").perform(context)
    stereo_camera_enabled = LaunchConfiguration("stereo_camera_enabled").perform(context)
    two_d_lidar_enabled = LaunchConfiguration("two_d_lidar_enabled").perform(context)
    odometry_source = LaunchConfiguration("odometry_source").perform(context)
    bridge_clock = LaunchConfiguration("bridge_clock").perform(context).lower() == "true"

    # Select xacro file and robot name based on variant
    if robot_variant == "amr2":
        xacro_file = join(amr_sim_path, "urdf", "bcr_bot_amr2.xacro")
        robot_name = "bcr_bot_amr2"
    else:
        xacro_file = join(amr_sim_path, "urdf", "bcr_bot_amr1.xacro")
        robot_name = "bcr_bot_amr1"

    # TF frame prefix: "bcr_bot/" or "bcr_bot_amr2/"
    frame_prefix = robot_name + "/"

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot_name,
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command([
                        "xacro ",
                        xacro_file,
                        " camera_enabled:=",
                        camera_enabled,
                        " stereo_camera_enabled:=",
                        stereo_camera_enabled,
                        " two_d_lidar_enabled:=",
                        two_d_lidar_enabled,
                        " odometry_source:=",
                        odometry_source,
                        " wheel_odom_topic:=",
                        robot_name + "/odom",
                        " robot_name:=",
                        robot_name,
                        " sim_gz:=true",
                    ]),
                    value_type=str,
                ),
                # ros_gz_sim/create subscribes to this topic.  Make the
                # robot description explicitly available to late-joining
                # spawners instead of depending on the RSP default.
                "publish_robot_description": True,
                "frame_prefix": frame_prefix,
            }
        ],
        remappings=[
            ("/joint_states", robot_name + "/joint_states"),
            # Nav2 runs in the robot namespace and resolves its internal
            # `tf` remap to /<robot>/tf.  Keep each robot's TF tree on that
            # same topic so its navigation stack can actually consume it.
            ("/tf", "/" + robot_name + "/tf"),
            ("/tf_static", "/" + robot_name + "/tf_static"),
        ],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "/" + robot_name + "/robot_description",
            "-name",
            robot_name,
            "-z",
            "0.28",
            "-x",
            position_x,
            "-y",
            position_y,
            "-Y",
            orientation_yaw,
        ],
    )

    bridge_arguments = [
        "/" + robot_name + "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
        "/" + robot_name + "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        "/" + robot_name + "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        "/kinect_camera@sensor_msgs/msg/Image[gz.msgs.Image",
        "/stereo_camera/left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
        "stereo_camera/right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
        "kinect_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        "stereo_camera/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        "stereo_camera/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        "/kinect_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
        "/world/default/model/" + robot_name + "/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
    ]
    if bridge_clock:
        bridge_arguments.insert(1, "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock")

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=bridge_arguments,
        remappings=[
            ("/world/default/model/" + robot_name + "/joint_state", robot_name + "/joint_states"),
            ("/" + robot_name + "/scan", robot_name + "/scan"),
            ("/kinect_camera", robot_name + "/kinect_camera"),
            ("/stereo_camera/left/image_raw", robot_name + "/stereo_camera/left/image_raw"),
            ("/stereo_camera/right/image_raw", robot_name + "/stereo_camera/right/image_raw"),
            ("/imu", robot_name + "/imu"),
            ("/" + robot_name + "/cmd_vel", robot_name + "/cmd_vel"),
            ("kinect_camera/camera_info", robot_name + "/kinect_camera/camera_info"),
            ("stereo_camera/left/camera_info", robot_name + "/stereo_camera/left/camera_info"),
            ("stereo_camera/right/camera_info", robot_name + "/stereo_camera/right/camera_info"),
            ("/kinect_camera/points", robot_name + "/kinect_camera/points"),
            # Gazebo publishes model poses on one global /tf topic.  Each
            # robot bridge republishes it into the corresponding ROS
            # namespace; frame IDs are already robot-prefixed.
            ("/tf", "/" + robot_name + "/tf"),
        ],
    )

    transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--yaw", "0.0",
            "--pitch", "0.0",
            "--roll", "0.0",
            "--frame-id", frame_prefix + "kinect_camera",
            "--child-frame-id", robot_name + "/base_footprint/kinect_camera",
        ],
        remappings=[
            ("/tf_static", "/" + robot_name + "/tf_static"),
        ],
    )

    return [
        robot_state_publisher,
        gz_spawn_entity,
        transform_publisher,
        gz_ros2_bridge,
    ]


def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            "robot_variant",
            default_value="amr1",
            description="Robot variant to spawn: 'amr1' (bcr_bot, default) or 'amr2' (bcr_bot_amr2)",
        ),

        DeclareLaunchArgument(
            "camera_enabled",
            default_value="true",
        ),

        DeclareLaunchArgument(
            "stereo_camera_enabled",
            default_value="false",
        ),

        DeclareLaunchArgument(
            "two_d_lidar_enabled",
            default_value="true",
        ),

        DeclareLaunchArgument(
            "position_x",
            default_value="0.0",
        ),

        DeclareLaunchArgument(
            "position_y",
            default_value="0.0",
        ),

        DeclareLaunchArgument(
            "orientation_yaw",
            default_value="0.0",
        ),

        DeclareLaunchArgument(
            "odometry_source",
            default_value="world",
        ),

        DeclareLaunchArgument(
            "bridge_clock",
            default_value="true",
            description="Bridge /clock when spawning a robot outside the fleet launch.",
        ),

        OpaqueFunction(function=launch_setup),
    ])

#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    camera_model = "zed2i"
    camera_name = "zed2i"

    # Set LOG format
    os.environ[
        "RCUTILS_CONSOLE_OUTPUT_FORMAT"
    ] = "{time} [{name}] [{severity}] {message}"

    # Define LaunchDescription variable
    ld = LaunchDescription()

    # Configuration variables
    publish_urdf = "true"  # Publish static frames from camera URDF
    # Position X of the camera with respect to the base frame [m].
    cam_pos_x = "0.0"
    # Position Y of the camera with respect to the base frame [m].
    cam_pos_y = "0.0"
    # Position Z of the camera with respect to the base frame [m].
    cam_pos_z = "0.0"
    # Roll orientation of the camera with respect to the base frame [rad].
    cam_roll = "0.0"
    # Pitch orientation of the camera with respect to the base frame [rad].
    cam_pitch = "0.0"
    # Yaw orientation of the camera with respect to the base frame [rad].
    cam_yaw = "0.0"

    # ZED Configurations to be loaded by ZED Node
    config_common_path = os.path.join(
        get_package_share_directory("zed_wrapper"), "config", "indy-common.yaml"
    )
    config_camera_path = os.path.join(
        get_package_share_directory("zed_wrapper"), "config/indy-zed2i.yaml"
    )

    # URDF/xacro file to be loaded by the Robot State Publisher node
    xacro_path = os.path.join(
        get_package_share_directory("zed_wrapper"), "urdf", "zed_descr.urdf.xacro"
    )

    # ZED Wrapper node
    zed_wrapper_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            [
                get_package_share_directory("zed_wrapper"),
                "/launch/include/zed_camera.launch.py",
            ]
        ),
        launch_arguments={
            "camera_model": LaunchConfiguration("camera_model", default="zed2i"),
            "camera_name": LaunchConfiguration("camera_name", default="camera_name"),
            "node_name": LaunchConfiguration("node_name"),
            "config_common_path": LaunchConfiguration("config_common_path"),
            "config_camera_path": LaunchConfiguration("config_camera_path"),
            "publish_urdf": LaunchConfiguration("publish_urdf"),
            "xacro_path": xacro_path,
            "base_frame": LaunchConfiguration("base_frame", default="camera_link"),
            "cam_pos_x": "0.0",
            "cam_pos_y": "0.0",
            "cam_pos_z": "0.0",
            "cam_roll": "0.0",
            "cam_pitch": "0.0",
            "cam_yaw": "0.0",
        }.items(),
    )
    config_common_path = DeclareLaunchArgument(
        "config_common_path",
    )
    config_camera_path = DeclareLaunchArgument(
        "config_camera_path",
    )
    node_name = DeclareLaunchArgument("node_name")
    camera_model = DeclareLaunchArgument("camera_model", default_value="zed2i")
    camera_name = DeclareLaunchArgument("camera_name")
    base_frame = DeclareLaunchArgument("base_frame")
    publish_urdf = DeclareLaunchArgument("publish_urdf")

    # Add nodes to LaunchDescription
    ld.add_action(config_common_path)
    ld.add_action(config_camera_path)
    ld.add_action(node_name)
    ld.add_action(camera_model)
    ld.add_action(camera_name)
    ld.add_action(base_frame)
    ld.add_action(publish_urdf)

    ld.add_action(zed_wrapper_launch)
    return ld

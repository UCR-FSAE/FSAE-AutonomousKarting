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
    camera_0_name = "zed2i_0"
    camera_1_name = "zed2i_1"

    # Rviz2 Configurations to be loaded by ZED Node
    config_rviz2 = os.path.join(
        get_package_share_directory("zed_wrapper"),
        "config",
        "multi-cam" + ".rviz",
    )

    # Set LOG format
    os.environ[
        "RCUTILS_CONSOLE_OUTPUT_FORMAT"
    ] = "{time} [{name}] [{severity}] {message}"

    # Rviz2 node
    rviz2_node = Node(
        package="rviz2",
        namespace="rviz",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[["-d"], [config_rviz2]],
    )

    # Define LaunchDescription variable
    ld = LaunchDescription()

    # Configuration variables
    camera_1_node_name = f"{camera_0_name}_node"  # Zed Node name
    camera_2_node_name = f"{camera_1_name}_node"
    publish_urdf = "true"  # Publish static frames from camera URDF
    # Robot base frame. Note: overrides the parameter `pos_tracking.base_frame` in `common.yaml`.
    base_frame = "base_link"
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
    zed_wrapper_1_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            [
                get_package_share_directory("zed_wrapper"),
                "/launch/include/zed_camera.launch.py",
            ]
        ),
        launch_arguments={
            "camera_model": camera_model,
            "camera_name": camera_0_name,
            "node_name": camera_1_node_name,
            "config_common_path": config_common_path,
            "config_camera_path": config_camera_path,
            "publish_urdf": publish_urdf,
            "xacro_path": xacro_path,
            "base_frame": base_frame,
            "cam_pos_x": cam_pos_x,
            "cam_pos_y": cam_pos_y,
            "cam_pos_z": cam_pos_z,
            "cam_roll": cam_roll,
            "cam_pitch": cam_pitch,
            "cam_yaw": cam_yaw,
            "zed_id": "0",
        }.items(),
    )
    # ZED Wrapper node
    zed_wrapper_2_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            [
                get_package_share_directory("zed_wrapper"),
                "/launch/include/zed_camera.launch.py",
            ]
        ),
        launch_arguments={
            "camera_model": camera_model,
            "camera_name": camera_1_name,
            "node_name": camera_2_node_name,
            "config_common_path": config_common_path,
            "config_camera_path": config_camera_path,
            "publish_urdf": publish_urdf,
            "xacro_path": xacro_path,
            "base_frame": base_frame,
            "cam_pos_x": cam_pos_x,
            "cam_pos_y": cam_pos_y,
            "cam_pos_z": cam_pos_z,
            "cam_roll": cam_roll,
            "cam_pitch": cam_pitch,
            "cam_yaw": cam_yaw,
            "zed_id": "1",
        }.items(),
    )
    # Add nodes to LaunchDescription
    ld.add_action(zed_wrapper_1_launch)
    ld.add_action(zed_wrapper_2_launch)

    ld.add_action(rviz2_node)

    return ld

# Copyright 2023 michael. All rights reserved.
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file.

import os
import launch
from ament_index_python.packages import get_package_share_directory
import launch_ros
from pathlib import Path
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition  # 1

os.environ["RCUTILS_CONSOLE_OUTPUT_FORMAT"] = "{time}: [{name}] [{severity}]\t{message}"


def generate_launch_description():
    base_path = Path(get_package_share_directory("roar-indy-launches"))
    gokart_roar_1: Path = Path("config/")
    rviz_path: Path = base_path / gokart_roar_1 / "gokart_roar_1.rviz"
    assert rviz_path.exists(), f"{rviz_path} does not exist"

    should_launch_rviz_args = DeclareLaunchArgument(
        "rviz",
        default_value="False",  # default_value=[], has the same problem
        description="True to start rviz, false otherwise",
    )
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_path.as_posix()],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # vehicle description launch
    urdf_file_path: Path = (
        Path(get_package_share_directory("roar-gokart-urdf"))
        / "launch"
        / "state_publisher.launch.py"
    )
    assert urdf_file_path.exists(), f"[{urdf_file_path}] does not exist"
    vehicle_urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(urdf_file_path.as_posix())
    )
    # LiDAR node
    livox_file_path: Path = (
        Path(get_package_share_directory("livox_ros2_driver"))
        / "launch"
        / "indy_livox_lidar.launch.py"
    )
    assert livox_file_path.exists()
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(livox_file_path.as_posix()),
        launch_arguments={
            "frame_id": "center_lidar",
            "user_config_path": (
                base_path / "center_livox_lidar_config.json"
            ).as_posix(),
            "lidar_publish_freq": "10.0",
        }.items(),
    )
    # ZED
    zed_file_path: Path = (
        Path(get_package_share_directory("zed_wrapper"))
        / "launch"
        / "indy_zed2i.launch.py"
    )
    zed_config_file_path: Path = Path(base_path / "config" / "zed2i_config.yaml")
    assert zed_file_path.exists()
    assert zed_config_file_path.exists()
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_file_path.as_posix()),
        launch_arguments={
            "config_common_path": zed_config_file_path.as_posix(),
            "config_camera_path": zed_config_file_path.as_posix(),
            "node_name": "center_camera",
            "camera_name": "zed2i",
            "base_frame": "camera_link",
            "publish_urdf": "True",
        }.items(),
    )

    # septentrio_file_path: Path = (
    #     Path(get_package_share_directory("septentrio_gnss_driver"))
    #     / "launch"
    #     / "rover_node.py"
    # )
    # septentrio_config_file_path: Path = (
    #     base_path / "config" / "gokart_roar_1_septentrio_gps_config.yaml"
    # )
    # assert septentrio_file_path.exists()
    # assert septentrio_config_file_path.exists()
    # gps_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(septentrio_file_path.as_posix()),
    #     launch_arguments={
    #         "path_to_config": septentrio_config_file_path.as_posix(),
    #     }.items(),
    # )

    manual_control_node = launch_ros.actions.Node(
        package="manual_controller",
        executable="manual_controller_node",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_path.as_posix()],
        condition=IfCondition(LaunchConfiguration("manual_control")),
    )
    should_launch_manual_control_args = DeclareLaunchArgument(
        "manual_control", default_value="False"
    )

    # pid_control_converter_path: Path = (
    #     Path(get_package_share_directory("control_converter"))
    #     / "launch"
    #     / "pid_control_converter.launch.py"
    # )
    # assert pid_control_converter_path.exists()
    # pid_control_converter = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(pid_control_converter_path.as_posix()),
    #     launch_arguments={}.items(),
    # )

    ld = launch.LaunchDescription()

    ld.add_action(should_launch_rviz_args)
    ld.add_action(should_launch_manual_control_args)

    ld.add_action(rviz_node)
    ld.add_action(zed_launch)
    ld.add_action(manual_control_node)
    # ld.add_action(lidar_launch)
    # ld.add_action(gps_launch)
    ld.add_action(vehicle_urdf_launch)
    # ld.add_action(pid_control_converter)
    return ld


if __name__ == "__main__":
    generate_launch_description()

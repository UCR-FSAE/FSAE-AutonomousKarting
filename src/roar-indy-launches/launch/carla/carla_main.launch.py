# Copyright 2023 michael. All rights reserved.
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file.

import os
import launch
from ament_index_python.packages import get_package_share_directory
import launch_ros
from pathlib import Path
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    base_path = Path(get_package_share_directory("roar-indy-launches"))
    config_file = base_path / "config" / "carla" / "carla_configs.yaml"
    assert config_file.exists(), f"[{config_file}] does not exist"

    base_path = Path(get_package_share_directory("roar-indy-launches"))
    carla_objects_definition_file = (
        base_path / "config" / "carla" / "carla_objects_definition_file.json"
    )
    assert carla_objects_definition_file.exists()
    rviz_path: Path = base_path / "config" / "carla" / "gokart_carla_1.rviz"
    assert rviz_path.exists(), f"{rviz_path} does not exist"

    debug_args = DeclareLaunchArgument(
        "debug",
        default_value="False",
        description="True to start debug outputs",
    )

    should_launch_manual_control_args = DeclareLaunchArgument(
        "should_launch_manual_control",
        default_value="False",
        description="True to start manual control, false otherwise",
    )

    roar_carla_launch_file_path: Path = (
        Path(get_package_share_directory("roar_carla"))
        / "launch"
        / "roar_carla.launch.py"
    )
    assert (
        roar_carla_launch_file_path.exists()
    ), f"[{roar_carla_launch_file_path}] does not exist"
    carla_client_node = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            roar_carla_launch_file_path.as_posix()
        ),
        launch_arguments={
            "town": "Town04",
            "objects_definition_file": carla_objects_definition_file.as_posix(),
            "should_launch_manual_control": LaunchConfiguration(
                "should_launch_manual_control"
            ),
        }.items(),
    )

    pointcloud_to_laser = Node(
        name="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        package="pointcloud_to_laserscan",
        parameters=[config_file],
        remappings=[
            ("cloud_in", "/carla/ego_vehicle/center_lidar"),
            ("scan", "/carla/ego_vehicle/laserscan"),
        ],
    )

    should_launch_rviz_args = DeclareLaunchArgument(
        "should_launch_rviz",
        default_value="False",  # default_value=[], has the same problem
        description="True to start rviz, false otherwise",
    )
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_path.as_posix()],
        condition=IfCondition(LaunchConfiguration("should_launch_rviz")),
    )
    costmap_config_file_path: Path = (
        base_path / "config" / "carla" / "gokart_carla_1_costmap2d_config.yaml"
    )
    assert costmap_config_file_path.exists()
    costmap_manager_launch_file_path: Path = (
        Path(get_package_share_directory("costmap_node_manager"))
        / "launch"
        / "costmap_node_manager.launch.py"
    )
    assert costmap_manager_launch_file_path.exists()

    should_launch_local_costmap_marker_args = DeclareLaunchArgument(
        "should_launch_local_costmap_marker",
        default_value="False",  # default_value=[], has the same problem
        description="true to start emitting local costmap detected obstacle markers. False by default",
    )
    costmap_manager = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            costmap_manager_launch_file_path.as_posix()
        ),
        launch_arguments={
            "costmap_config_file_path": costmap_config_file_path.as_posix(),
            "should_launch_local_costmap_marker": LaunchConfiguration(
                "should_launch_local_costmap_marker"
            ),
        }.items(),
    )

    global_planner_manager_file_path: Path = (
        Path(get_package_share_directory("global_planner_manager"))
        / "launch"
        / "global_planner_manager.launch.py"
    )
    assert global_planner_manager_file_path.exists()
    global_waypoint_file_path: Path = (
        base_path / "config" / "carla" / "carla_waypoints.txt"
    )
    assert global_waypoint_file_path.exists()
    global_planner_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(global_planner_manager_file_path.as_posix()),
        launch_arguments={
            "waypoint_file_path": global_waypoint_file_path.as_posix(),
        }.items(),
    )
    roar_carla_control_path: Path = (
        Path(get_package_share_directory("roar_carla"))
        / "launch"
        / "roar_carla_control.launch.py"
    )
    roar_carla_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(roar_carla_control_path.as_posix())
    )

    controller_path: Path = (
        Path(get_package_share_directory("controller_manager"))
        / "launch"
        / "controller.launch.py"
    )
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(controller_path.as_posix())
    )
    local_planner_manager_launch_file_path: Path = (
        Path(get_package_share_directory("local_planner_manager"))
        / "launch"
        / "local_planner_manager.launch.py"
    )
    local_planner_manager = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            local_planner_manager_launch_file_path.as_posix()
        )
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_main",
        output="screen",
        parameters=[config_file],
    )

    ld = launch.LaunchDescription()
    # add args
    ld.add_action(debug_args)
    ld.add_action(should_launch_local_costmap_marker_args)
    ld.add_action(should_launch_rviz_args)
    ld.add_action(should_launch_manual_control_args)

    # add nodes
    ld.add_action(rviz_node)
    ld.add_action(pointcloud_to_laser)
    ld.add_action(carla_client_node)
    ld.add_action(costmap_manager)
    ld.add_action(global_planner_launcher)
    ld.add_action(local_planner_manager)
    ld.add_action(roar_carla_control)
    ld.add_action(controller)

    ld.add_action(lifecycle_manager)
    return ld


if __name__ == "__main__":
    generate_launch_description()

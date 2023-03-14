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
        parameters=[
            {
                "transform_tolerance": 0.01,
                "min_height": 0.0,
                "max_height": 100.0,
                "angle_min": -1.5708,  # -M_PI/2
                "angle_max": 1.5708,  # M_PI/2
                "angle_increment": 0.0087,  # M_PI/360.0
                "scan_time": 0.3333,
                "range_min": 0.45,
                "range_max": 100.0,
                "use_inf": True,
                "inf_epsilon": 1.0,
            }
        ],
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
            "debug": LaunchConfiguration("debug"),
        }.items(),
    )
    simple_local_planner_file_path: Path = (
        Path(get_package_share_directory("simple_local_planner"))
        / "launch"
        / "simple_local_planner.launch.py"
    )
    simple_local_planner_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simple_local_planner_file_path.as_posix()),
        launch_arguments={"loop_rate": "10.0", "target_spd": "13.0"}.items(),
    )
    roar_carla_control_path: Path = (
        Path(get_package_share_directory("roar_carla"))
        / "launch"
        / "roar_carla_control.launch.py"
    )
    roar_carla_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(roar_carla_control_path.as_posix())
    )

    pid_control_path: Path = (
        Path(get_package_share_directory("pid_control"))
        / "launch"
        / "pid_control.launch.py"
    )
    pid_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pid_control_path.as_posix()),
        launch_arguments={
            "debug": LaunchConfiguration("debug"),
        }.items(),
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"autostart": True},
            {
                "node_names": [
                    "/costmap_node_manager",
                    "/global_planner_manager",
                    "/simple_local_planner",
                    "/pid_control",
                ]
            },
        ],
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
    ld.add_action(simple_local_planner_launcher)
    ld.add_action(roar_carla_control)
    ld.add_action(pid_control)

    ld.add_action(lifecycle_manager)
    return ld


if __name__ == "__main__":
    generate_launch_description()

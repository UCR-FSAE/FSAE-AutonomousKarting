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


def generate_launch_description():
    base_path = Path(get_package_share_directory("global_planner_manager"))

    config_file = (
        base_path / "config" / "configs.yaml"
    )
    assert config_file.exists(), f"[{config_file}] does not exist"

    ld = launch.LaunchDescription()
    waypoint_file_path = DeclareLaunchArgument(
        "waypoint_file_path",
        default_value="./src/roar-indy-launches/config/carla_waypoints.txt",
    )
    speed_zone_and_lookahead_distance_path = DeclareLaunchArgument(
        "speed_zone_and_lookahead_distance_path",
        default_value=(
            base_path / "params" / "lookahead_distance_vs_speed.txt"
        ).as_posix(),
    )
    lookahead_dist = DeclareLaunchArgument(
        "lookahead_dist",
        default_value="5.0",
    )
    odom_topic = DeclareLaunchArgument(
        "odom_topic",
        default_value="/carla/ego_vehicle/odometry",
    )

    waypoint_follower_server_loop_rate = DeclareLaunchArgument(
        "loop_rate",
        default_value="5.0",
    )
    global_planner_manager_node = Node(
        name="global_planner_manager",
        executable="global_planner_manager",
        package="global_planner_manager",
        # parameters=[config_file],
        parameters=[
            {
                "waypoint_file_path": LaunchConfiguration(
                    "waypoint_file_path",
                    default="./src/roar-indy-launches/config/carla_waypoints.txt",
                ),
                "debug": LaunchConfiguration("debug", default=False),
            }
        ],
    )
    waypoint_follower_server_node = Node(
        name="waypoint_follower_server_node",
        executable="waypoint_follower",
        package="global_planner_manager",
        # parameters=[config_file],
        parameters=[
            {
                "lookahead_dist": LaunchConfiguration("lookahead_dist"),
                "loop_rate": LaunchConfiguration("loop_rate"),
                "speed_zone_and_lookahead_distance": LaunchConfiguration(
                    "speed_zone_and_lookahead_distance_path"
                ),
            }
        ],
        remappings=[
            (
                "odom",
                LaunchConfiguration("odom_topic", default="/carla/ego_vehicle/odom"),
            )
        ],
    )
    # args
    ld.add_action(waypoint_file_path)
    ld.add_action(speed_zone_and_lookahead_distance_path)
    ld.add_action(lookahead_dist)
    ld.add_action(odom_topic)
    ld.add_action(waypoint_follower_server_loop_rate)

    # node
    ld.add_action(global_planner_manager_node)
    ld.add_action(waypoint_follower_server_node)

    return ld

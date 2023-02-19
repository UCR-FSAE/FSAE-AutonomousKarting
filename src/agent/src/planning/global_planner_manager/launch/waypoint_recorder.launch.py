# Copyright 2023 michael. All rights reserved.
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file.

import launch
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    ld = launch.LaunchDescription()
    output_file_path = DeclareLaunchArgument(
        "output_file_path",
        default_value="./data/file.txt",
    )
    odom_topic = DeclareLaunchArgument(
        "odom_topic",
        default_value="/carla/ego_vehicle/odometry",
    )
    record_interval = DeclareLaunchArgument(
        "record_interval",
        default_value="1.0",
    )
    waypoint_recorder = Node(
        name="waypoint_recorder_node",
        executable="waypoint_recorder",
        package="global_planner_manager",
        parameters=[
            {
                "output_file_path": LaunchConfiguration("output_file_path"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "record_interval": LaunchConfiguration("record_interval"),
            }
        ],
    )
    # args
    ld.add_action(output_file_path)
    ld.add_action(odom_topic)
    ld.add_action(record_interval)

    # node
    ld.add_action(waypoint_recorder)

    return ld

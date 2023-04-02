# Copyright 2023 michael. All rights reserved.
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file.

import launch
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    ld = launch.LaunchDescription()
    # loop_rate_args = DeclareLaunchArgument(
    #     "loop_rate",
    #     default_value="0.5",
    #     description="loop rate",
    # )
    local_planner_manager_node = Node(
        name="local_planner_manager_node",
        executable="local_planner_manager_node",
        package="local_planner_manager",
        parameters=[
            {
                "loop_rate": LaunchConfiguration("loop_rate"),
            }
        ],
    )
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_local_planning",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"autostart": True},
            {
                "node_names": [
                    "/local_planner/local_planner_manager_node",
                ]
            },
        ],
    )

    # args
    # ld.add_action(loop_rate_args)
    # node
    ld.add_action(local_planner_manager_node)
    ld.add_action(lifecycle_manager)

    return ld

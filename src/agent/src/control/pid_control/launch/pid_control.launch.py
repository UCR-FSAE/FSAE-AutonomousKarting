# Copyright 2023 michael. All rights reserved.
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file.

import launch
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    ld = launch.LaunchDescription()

    loop_rate = DeclareLaunchArgument(
        "loop_rate",
        default_value="30.0",
    )
    debug_args = DeclareLaunchArgument(
        "debug",
        default_value="False",
    )
    node = Node(
        name="pid_control",
        executable="pid_control_node",
        package="pid_control",
        parameters=[
            {
                "loop_rate": LaunchConfiguration("loop_rate"),
                "debug": LaunchConfiguration("debug"),
            }
        ],
    )
    # lifecycle_manager = Node(
    #     package="nav2_lifecycle_manager",
    #     executable="lifecycle_manager",
    #     name="lifecycle_manager_navigation",
    #     output="screen",
    #     parameters=[
    #         {"use_sim_time": True},
    #         {"autostart": True},
    #         {
    #             "node_names": [
    #                 "/pid_control",
    #             ]
    #         },
    #     ],
    # )

    # args
    ld.add_action(loop_rate)
    ld.add_action(debug_args)

    # node
    ld.add_action(node)
    # ld.add_action(lifecycle_manager)

    return ld

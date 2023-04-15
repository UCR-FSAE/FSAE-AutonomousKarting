# Copyright 2023 michael. All rights reserved.
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file.

from ament_index_python.packages import get_package_share_directory
import launch_ros
from pathlib import Path
import launch
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    base_path = Path(get_package_share_directory("simple_local_planner"))
    config_file = (
        base_path / "config" / "configs.yaml"
    )
    assert config_file.exists(), f"[{config_file}] does not exist"
    
    ld = launch.LaunchDescription()

    loop_rate = DeclareLaunchArgument(
        "loop_rate",
        default_value="5.0",
    )
    target_spd = DeclareLaunchArgument(
        "target_spd",
        default_value="2.0",
    )
    simple_local_planner_node = Node(
        name="simple_local_planner",
        executable="simple_local_planner_node",
        package="simple_local_planner",
        parameters=[
            config_file
        ],
        # parameters=[
        #     {
        #         "loop_rate": LaunchConfiguration("loop_rate"),
        #         "target_spd": LaunchConfiguration("target_spd"),
        #     }
        # ],

    )
    # args
    ld.add_action(loop_rate)
    ld.add_action(target_spd)

    # node
    ld.add_action(simple_local_planner_node)

    return ld

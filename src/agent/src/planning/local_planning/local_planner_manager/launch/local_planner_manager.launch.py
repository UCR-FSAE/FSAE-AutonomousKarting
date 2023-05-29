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
    base_path = Path(get_package_share_directory("local_planner_manager"))

    config_file = base_path / "params" / "configs.yaml"
    assert config_file.exists()
    print(f"Local Planner config file: [{config_file}]")
    local_planner_manager_node = Node(
        name="local_planner_manager",
        executable="local_planner_manager_node",
        package="local_planner_manager",
        parameters=[config_file.as_posix()],
        emulate_tty=True,
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_local_planning",
        output="screen",
        parameters=[config_file.as_posix()],
    )

    # node
    ld.add_action(local_planner_manager_node)
    ld.add_action(lifecycle_manager)

    return ld

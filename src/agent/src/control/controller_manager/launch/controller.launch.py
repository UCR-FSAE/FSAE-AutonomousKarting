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
    base_path = Path(get_package_share_directory("controller_manager"))

    config_file = base_path / "params" / "configs.yaml"
    assert config_file.exists()
    controller_manager = Node(
        name="manager",
        executable="controller_manager",
        package="controller_manager",
        parameters=[
                    config_file.as_posix(),
                    {"pid_config_file_path": (base_path / "params" / "carla_pid.json").as_posix()}
                    ],
        emulate_tty=True,
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_controller",
        output="screen",
        parameters=[config_file.as_posix()],
    )

    # node
    ld.add_action(controller_manager)
    ld.add_action(lifecycle_manager)

    return ld

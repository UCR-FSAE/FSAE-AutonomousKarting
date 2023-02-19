# Copyright 2023 michael. All rights reserved.
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file.

import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import launch_ros
import os


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package="roar-gokart-urdf"
    ).find("roar-gokart-urdf")
    default_model_path = os.path.join(pkg_share, "src/gokart/main.urdf")
    should_launch_rviz_args = DeclareLaunchArgument(
        "should_launch_rviz",
        default_value="False",  # default_value=[], has the same problem
        description="True to start rviz, false otherwise",
    )
    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": Command(["xacro ", LaunchConfiguration("model")])}
        ],
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=launch.conditions.UnlessCondition(
            LaunchConfiguration("gui", default="False")
        ),
    )

    return launch.LaunchDescription(
        [
            should_launch_rviz_args,
            launch.actions.DeclareLaunchArgument(
                name="model",
                default_value=default_model_path,
                description="Absolute path to robot urdf file",
            ),
            joint_state_publisher_node,
            robot_state_publisher_node,
        ]
    )

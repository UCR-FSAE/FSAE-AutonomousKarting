# Copyright 2023 michael. All rights reserved.
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file.

import os

import launch
from ament_index_python.packages import get_package_share_directory
import launch_ros


def generate_launch_description():
    base_path = os.path.realpath(
        get_package_share_directory("roar-indy-launches")
    )  # also tried without realpath
    rviz_path = base_path + "/config/sim_all.rviz"
    waypoint_file_path = base_path + "/config/eVGrandPrixWaypoints.txt"

    carla_client_node = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("roar_carla_ros2"),
                "roar_carla_no_rviz.launch.py",
            )
        ),
    )
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_path],
    )
    ground_plane_detector_node = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("roar_ground_plane_detector"),
                "ground_plane_detector.launch.py",
            )
        ),
    )
    cluster_extractor_node = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("obstacle_detector_ros2"),
                "launch/euclidean_clustering.launch.py",
            )
        ),
    )
    costmap_node = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("cost_map"),
                "launch/costmap.launch.py",
            )
        ),
    )

    waypoint_publisher_node = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("roar_waypoint"),
                "launch/waypoint_publisher.launch.py",
            )
        ),
        launch_arguments={
            "waypoint_file_path": launch.substitutions.LaunchConfiguration(
                "waypoint_file_path"
            )
        }.items(),
    )
    ld = launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="waypoint_file_path", default_value=waypoint_file_path
            ),
            carla_client_node,
            rviz_node,
            ground_plane_detector_node,
            cluster_extractor_node,
            costmap_node,
            waypoint_publisher_node,
        ]
    )
    return ld


if __name__ == "__main__":
    generate_launch_description()

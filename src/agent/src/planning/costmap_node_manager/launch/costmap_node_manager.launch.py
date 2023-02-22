import os
import launch
from ament_index_python.packages import get_package_share_directory
import launch_ros
from pathlib import Path
from launch_ros.actions import Node


def generate_launch_description():
    base_path = Path(get_package_share_directory("costmap_node_manager"))
    default_costmap_config_file_path: Path = base_path / "config" / "config.yaml"

    costmap_manager = Node(
        executable="costmap_node_manager",
        package="costmap_node_manager",
        parameters=[
            launch.substitutions.LaunchConfiguration("costmap_config_file_path")
        ],
    )

    ld = launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="costmap_config_file_path",
                default_value=default_costmap_config_file_path.as_posix(),
            ),
            costmap_manager,
        ]
    )
    return ld

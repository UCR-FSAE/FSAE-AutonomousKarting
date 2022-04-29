import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    base_path: Path = Path(get_package_share_directory("roar-indy-launches"))
    config_base = base_path / "config"
    rviz_path = config_base / "berkeley_base.rviz"

    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_path.as_posix()],
    )
    sensors_node = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            [
                get_package_share_directory("roar-indy-launches"),
                "/launch/berkeley_base.launch.py",
            ]
        ),
    )
    return launch.LaunchDescription([rviz_node, sensors_node])

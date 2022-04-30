import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    base_path: Path = Path(get_package_share_directory("roar-indy-launches"))
    config_base = base_path / "config"

    left_livox = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            [
                get_package_share_directory("roar-indy-launches"),
                "/launch/left_livox.launch.py",
            ]
        ),
    )
    center_livox = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            [
                get_package_share_directory("roar-indy-launches"),
                "/launch/center_livox.launch.py",
            ]
        ),
    )
    right_livox = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            [
                get_package_share_directory("roar-indy-launches"),
                "/launch/right_livox.launch.py",
            ]
        ),
    )
    return launch.LaunchDescription([left_livox, center_livox, right_livox])

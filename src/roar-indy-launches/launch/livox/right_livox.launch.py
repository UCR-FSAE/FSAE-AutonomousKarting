import launch
import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    base_path: Path = Path(get_package_share_directory("roar-indy-launches"))
    config_base = base_path / "config"

    config_path = (config_base / "right_livox_lidar_config.json").as_posix()

    node = Node(
        package="livox_ros2_driver",
        executable="livox_ros2_driver_node",
        name="livox_lidar_right",
        output="screen",
        parameters=[
            {"xfer_format": 0},
            {"multi_topic": 1},
            {"data_src": 0},
            {"publish_freq": 30.0},
            {"output_data_type": 0},
            {"frame_id": "right_lidar"},
            {"cmdline_input_bd_code": "3WEDH7600103291"},
            {"user_config_path": config_path},
        ],
    )

    return launch.LaunchDescription(
        [
            node,
        ]
    )

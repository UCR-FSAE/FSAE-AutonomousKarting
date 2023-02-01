import os
import launch
from ament_index_python.packages import get_package_share_directory
import launch_ros
from pathlib import Path
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition  # 1


def generate_launch_description():
    base_path = Path(get_package_share_directory("roar-indy-launches"))

    septentrio_file_path: Path = (
        Path(get_package_share_directory("septentrio_gnss_driver"))
        / "launch"
        / "rover_node.py"
    )
    septentrio_config_file_path: Path = (
        base_path / "config" / "gokart_1" / "gokart_roar_septentrio_gps_config.yaml"
    )
    assert septentrio_file_path.exists()
    assert septentrio_config_file_path.exists()
    print(septentrio_config_file_path.parent.as_posix())
    print(septentrio_config_file_path.name)
    gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(septentrio_file_path.as_posix()),
        launch_arguments={
            "path_to_config": septentrio_config_file_path.as_posix(),
        }.items(),
    )

    ld = launch.LaunchDescription()
    ld.add_action(gps_launch)
    return ld

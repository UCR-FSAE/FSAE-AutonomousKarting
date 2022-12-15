import os
import launch
from ament_index_python.packages import get_package_share_directory
import launch_ros
from pathlib import Path
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    base_path = Path(get_package_share_directory("roar-indy-launches"))
    gokart_roar_1: Path = Path("config/")
    rviz_path: Path = base_path / gokart_roar_1 / "gokart_roar_1.rviz"
    assert rviz_path.exists(), f"{rviz_path} does not exist"
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_path.as_posix()],
    )

    # vehicle description launch
    urdf_file_path: Path = (
        Path(get_package_share_directory("roar-gokart-urdf"))
        / "launch"
        / "state_publisher_no_rviz.launch.py"
    )
    assert urdf_file_path.exists()
    vehicle_urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(urdf_file_path.as_posix())
    )
    # LiDAR node
    livox_file_path: Path = (
        Path(get_package_share_directory("livox_ros2_driver"))
        / "launch"
        / "indy_livox_lidar.launch.py"
    )
    assert livox_file_path.exists()
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(livox_file_path.as_posix()),
        launch_arguments={
            "frame_id": "center_lidar",
            "user_config_path": (
                base_path / "center_livox_lidar_config.json"
            ).as_posix(),
            "lidar_publish_freq": "10.0",
        }.items(),
    )
    # ZED
    zed_file_path: Path = (
        Path(get_package_share_directory("zed_wrapper"))
        / "launch"
        / "indy_zed2i_no_rviz.launch.py"
    )
    assert zed_file_path.exists()
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_file_path.as_posix())
    )

    ld = launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="start_rviz",
                default_value="false",
                description="Start RVIZ or not",
            ),
            launch.actions.DeclareLaunchArgument(
                name="rviz_path",
                default_value="false",
                description="absolute path for RVIZ config file",
            ),
            rviz_node,
            vehicle_urdf_launch,
            lidar_launch,
            zed_launch,
        ]
    )
    return ld


if __name__ == "__main__":
    generate_launch_description()

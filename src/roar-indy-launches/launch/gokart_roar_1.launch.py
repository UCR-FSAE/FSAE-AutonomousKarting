import os
import launch
from ament_index_python.packages import get_package_share_directory
import launch_ros
from pathlib import Path 
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    base_path:Path = Path(os.path.realpath(
        get_package_share_directory("roar-indy-launches")
    ) )

    gokart_roar_1: Path = Path("/config/gokart_roar_1/")
    rviz_path:Path = base_path / gokart_roar_1 / "base.rviz"

    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_path.as_posix()],
    )

    # vehicle description launch
    vehicle_urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
                        get_package_share_directory("roar-gokart-urdf"),
                        "state_publisher_no_rviz.launch.py",
                    ))
    )
    # LiDAR node
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
                        get_package_share_directory("livox_ros2_driver"),
                        "gokart_roar_1.launch.py",
                    )) # TODO: create 
    )
    # ZED 
    zed_launch =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
                        get_package_share_directory("zed_wrapper"),
                        "gokart_roar_1.launch.py",
                    )) 
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
            zed_launch
        ]
    )
    return ld


if __name__ == "__main__":
    generate_launch_description()

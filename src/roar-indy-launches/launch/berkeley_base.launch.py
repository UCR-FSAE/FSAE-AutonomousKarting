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
    rviz_path = base_path / "config" / "berkeley_base.rviz"

    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_path.as_posix()],
    )
    robot_state_node = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("indy_bot_urdf"),
                "launch",
                "state_publisher_no_rviz.launch.py",
            )
        ),
        launch_arguments={"gui": "true"}.items(),
    )
    berkeley_sensors_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("roar-indy-launches"),
                "launch",
                "berkeley_sensors.launch.py",
            )
        )
    )

    return launch.LaunchDescription(
        [
            # rviz_node,
            robot_state_node,
            launch.actions.TimerAction(period=1.0, actions=[berkeley_sensors_launch]),
        ]
    )

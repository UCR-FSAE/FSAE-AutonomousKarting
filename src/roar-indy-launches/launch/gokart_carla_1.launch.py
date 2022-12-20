import os
import launch
from ament_index_python.packages import get_package_share_directory
import launch_ros
from pathlib import Path
from launch_ros.actions import Node


def generate_launch_description():
    base_path = Path(get_package_share_directory("roar-indy-launches"))
    carla_objects_definition_file = (
        base_path / "config" / "carla_objects_definition_file.json"
    )
    assert carla_objects_definition_file.exists()
    rviz_path: Path = base_path / "config" / "gokart_carla_1.rviz"
    assert rviz_path.exists(), f"{rviz_path} does not exist"

    carla_client_node = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("roar_carla_ros2"),
                "roar_carla_no_rviz.launch.py",
            )
        ),
        launch_arguments={
            "town": "Town04",
            "objects_definition_file": carla_objects_definition_file.as_posix(),
        }.items(),
    )
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_path.as_posix()],
    )
    costmap_config_file_path: Path = (
        base_path / "config" / "gokart_carla_1_costmap2d_config.yaml"
    )
    costmap_node = Node(
        executable="nav2_costmap_2d",
        package="nav2_costmap_2d",
        parameters=[costmap_config_file_path.as_posix()],
    )
    costmap_marker_node = Node(
        package="nav2_costmap_2d",
        executable="nav2_costmap_2d_markers",
        remappings=[
            ("voxel_grid", "/costmap/voxel_grid"),
            ("visualization_marker", "/costmap/voxel_grid/visualize"),
        ],
    )
    lifecycle_nodes = ["/costmap/costmap"]
    use_sim_time = True
    autostart = True
    start_lifecycle_manager_cmd = launch_ros.actions.Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="screen",
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": lifecycle_nodes},
        ],
    )
    ld = launch.LaunchDescription(
        [
            carla_client_node,
            rviz_node,
            costmap_node,
            start_lifecycle_manager_cmd,
            costmap_marker_node,
        ]
    )
    return ld


if __name__ == "__main__":
    generate_launch_description()

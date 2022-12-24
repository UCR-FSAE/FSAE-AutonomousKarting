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

    pointcloud_to_laser = Node(
        name="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        package="pointcloud_to_laserscan",
        parameters=[
            {
                "transform_tolerance": 0.01,
                "min_height": 0.0,
                "max_height": 100.0,
                "angle_min": -1.5708,  # -M_PI/2
                "angle_max": 1.5708,  # M_PI/2
                "angle_increment": 0.0087,  # M_PI/360.0
                "scan_time": 0.3333,
                "range_min": 0.45,
                "range_max": 100.0,
                "use_inf": True,
                "inf_epsilon": 1.0,
            }
        ],
        remappings=[
            ("cloud_in", "/carla/ego_vehicle/center_lidar"),
            ("scan", "/carla/ego_vehicle/laserscan"),
        ],
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
    assert costmap_config_file_path.exists()
    costmap_manager_launch_file_path: Path = (
        Path(get_package_share_directory("costmap_node_manager"))
        / "launch"
        / "costmap_node_manager.launch.py"
    )
    assert costmap_manager_launch_file_path.exists()
    costmap_manager = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            costmap_manager_launch_file_path.as_posix()
        ),
        launch_arguments={
            "costmap_config_file_path": costmap_config_file_path.as_posix(),
        }.items(),
    )
    # costmap_marker_node = Node(
    #     package="nav2_costmap_2d",
    #     executable="nav2_costmap_2d_markers",
    #     remappings=[
    #         ("voxel_grid", "/costmap/voxel_grid"),
    #         ("visualization_marker", "/costmap/voxel_grid/visualize"),
    #     ],
    # )  # this node is only used for visualization

    ld = launch.LaunchDescription(
        [
            pointcloud_to_laser,
            carla_client_node,
            rviz_node,
            costmap_manager,
            # costmap_marker_node,
        ]
    )
    return ld


if __name__ == "__main__":
    generate_launch_description()

import os

import launch
from ament_index_python.packages import get_package_share_directory
import launch_ros
from pathlib import Path
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    base_path = Path(get_package_share_directory("roar-indy-launches"))
    costmap_config_file_path: Path = base_path / "config" / "global_costmap.yml"
    print(f"costmap_config = {costmap_config_file_path}")
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
    ld = launch.LaunchDescription(
        [
            pointcloud_to_laser,
            costmap_marker_node,
            costmap_node,
            start_lifecycle_manager_cmd,
        ]
    )
    return ld


if __name__ == "__main__":
    generate_launch_description()

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

    # ZED Configurations to be loaded by ZED Node
    zed_2i_config_path = (config_base / "berkeley_zed2i_config.yaml").as_posix()
    zed_node = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            [
                get_package_share_directory("zed_wrapper"),
                "/launch/include/zed_camera.launch.py",
            ]
        ),
        launch_arguments={
            "camera_model": "zed2i",
            "camera_name": "zed2i",
            "node_name": "center_zed2i",
            "config_common_path": zed_2i_config_path,
            "config_camera_path": zed_2i_config_path,
            "publish_urdf": "true",
            "xacro_path": os.path.join(
                get_package_share_directory("zed_wrapper"),
                "urdf",
                "zed_descr.urdf.xacro",
            ),
            "base_frame": "camera_link",
            "cam_pos_x": "0.0",
            "cam_pos_y": "0.0",
            "cam_pos_z": "0.0",
            "cam_roll": "0.0",
            "cam_pitch": "0.0",
            "cam_yaw": "0.0",
        }.items(),
    )

    left_lidar_config_path = (config_base / "left_livox_lidar_config.json").as_posix()
    center_lidar_config_path = (
        config_base / "center_livox_lidar_config.json"
    ).as_posix()
    right_lidar_config_path = (config_base / "right_livox_lidar_config.json").as_posix()

    left_livox_lidar_node = Node(
        package="livox_ros2_driver",
        executable="livox_ros2_driver_node",
        name="livox_lidar_left",
        output="screen",
        parameters=[
            {"xfer_format": 0},
            {"multi_topic": 1},
            {"data_src": 0},
            {"publish_freq": 10.0},
            {"output_data_type": 0},
            {"frame_id": "left_lidar"},
            {"user_config_path": left_lidar_config_path},
        ],
    )
    center_livox_lidar_node = Node(
        package="livox_ros2_driver",
        executable="livox_ros2_driver_node",
        name="livox_lidar_center",
        output="screen",
        parameters=[
            {"xfer_format": 0},
            {"multi_topic": 1},
            {"data_src": 0},
            {"publish_freq": 10.0},
            {"output_data_type": 0},
            {"frame_id": "center_lidar"},
            {"user_config_path": center_lidar_config_path},
        ],
    )
    right_livox_lidar_node = Node(
        package="livox_ros2_driver",
        executable="livox_ros2_driver_node",
        name="livox_lidar_right",
        output="screen",
        parameters=[
            {"xfer_format": 0},
            {"multi_topic": 1},
            {"data_src": 0},
            {"publish_freq": 10.0},
            {"output_data_type": 0},
            {"frame_id": "right_lidar"},
            {"user_config_path": right_lidar_config_path},
        ],
    )
    return launch.LaunchDescription(
        [
            zed_node,
            # launch.actions.TimerAction(period=1.0, actions=[left_livox_lidar_node]),
            launch.actions.TimerAction(period=1.0, actions=[center_livox_lidar_node]),
            # launch.actions.TimerAction(period=1.0, actions=[right_livox_lidar_node]),
        ]
    )

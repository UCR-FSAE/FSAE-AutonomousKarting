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
    config_base = base_path / "config"
    rviz_path = config_base / "single_lidar_zed_object_detection.rviz"

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
    center_lidar_config_path = (
        config_base / "center_livox_lidar_config.json"
    ).as_posix()
    center_lidar_node = Node(
        package="livox_ros2_driver",
        executable="livox_ros2_driver_node",
        name="livox_lidar_center",
        output="screen",
        parameters=[
            {"xfer_format": 0},
            {"multi_topic": 0},
            {"data_src": 0},
            {"publish_freq": 10.0},
            {"output_data_type": 0},
            {"frame_id": "center_lidar"},
            {"user_config_path": center_lidar_config_path},
        ],
    )

    zed_node = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            [
                get_package_share_directory("cone_detector_ros2"),
                "/launch/cone_detector.launch.py",
            ]
        ),
        launch_arguments={
            "rgb_camera_topic": "/zed2i/center_zed2i/left/image_rect_color",
            "rgb_camera_info_topic": "/zed2i/center_zed2i/left/camera_info",
            "lidar_topics": "[/livox/lidar]",
        }.items(),
    )

    return launch.LaunchDescription(
        [rviz_node, robot_state_node, center_lidar_node, zed_node]
    )

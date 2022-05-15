import os
from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch_ros.actions import Node
import launch

aruco_node = Node(
        package="ros2_aruco",
        executable="aruco_node",
        name="aruco_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "marker_size": 0.192
            },
            {
                "image_topic": "/zed2/zed_node/left/image_rect_color"
            },
            {
                "camera_info_topic": "/zed2/zed_node/left/camera_info"
            },
            {
                "camera_frame": "zed2_base_link"
            },
        ],
    )

def generate_launch_description():

    return LaunchDescription([aruco_node ])
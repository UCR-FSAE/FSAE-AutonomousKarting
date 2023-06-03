from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition #1
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
import launch

import os
from pathlib import Path

def generate_launch_description():

    ld = LaunchDescription()
    print("Hi from carla")
    return ld

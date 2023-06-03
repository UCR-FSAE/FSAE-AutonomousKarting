from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.conditions import IfCondition #1
from launch.actions import IncludeLaunchDescription
import launch
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
import os
from pathlib import Path
from launch import LaunchDescription, LaunchContext, LaunchService
from launch.actions import OpaqueFunction
import sys
record_dir = Path('./default')

def generate_launch_description():
    ld = LaunchDescription()

    base_path = Path(get_package_share_directory("roar-indy-launches"))

    """CARLA"""
    should_record_carla_arg = DeclareLaunchArgument('carla', 
                                                    default_value='false',
                                                    description='record carla waypoints')
    ld.add_action(should_record_carla_arg)
    carla_launch_path = (base_path / "launch"/ "recordWaypoints" / "record_waypoint_carla.launch.py").as_posix()
    ld.add_action(LogInfo(msg=f"carla_launch_path: {carla_launch_path}", condition=LaunchConfigurationEquals("carla", "true")))
    LaunchConfiguration("carla")

    carla_launch = IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(carla_launch_path),
            condition=LaunchConfigurationEquals("carla", "true")   #3
    )
    ld.add_action(carla_launch)
    
    """Pointone nav"""
    pointonenav_arg = DeclareLaunchArgument('pointonenav', 
                                        default_value='false',
                                        description='record pointonenav waypoints')
    ld.add_action(pointonenav_arg)

    pointonenav_launch_path = (base_path / "launch"/ "recordWaypoints" / "record_waypoint_pointonenav.launch.py").as_posix()
    ld.add_action(LogInfo(msg=f"pointonenav_launch_path: {pointonenav_launch_path}", condition=LaunchConfigurationEquals("pointonenav", "true")))
    LaunchConfiguration("pointonenav")

    pointonenav_launch = IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(pointonenav_launch_path),
            
            condition=LaunchConfigurationEquals("pointonenav", "true")   
    )
    # ld.add_action(pointonenav_launch)

    # start_navigation = IncludeLaunchDescription(condition=LaunchConfigurationEquals("pointonenav", "true"))
    ld.add_action(ExecuteProcess(cmd=['curl', '-X', 'POST', 'http://10.0.0.2/api/v1/application/start'],
                                 output='screen',
                                 condition=LaunchConfigurationEquals("pointonenav", "true") 
                                 ))

    deplayed_actions = launch.actions.TimerAction(
        period=5.0,
        actions=[pointonenav_launch],
        condition=LaunchConfigurationEquals("pointonenav", "true") 
    )
    ld.add_action(deplayed_actions)
    
    # start_navigation.add_action(ExecuteProcess(cmd=['sleep', '5']))
    # ld.add_action(start_navigation)          


    
    """ 
    adding actions for common parameters
    """
    rosbag_record_dir_args = DeclareLaunchArgument('dir', 
                                                default_value='./default',
                                                description='choose path to save rosbag')
    ld.add_action(rosbag_record_dir_args)

    ros2_bag_record = ExecuteProcess(cmd=['ros2', 'bag', 'record', f'-o {get_record_dir()}', '-a'],output='screen')
    
    ld.add_action(LogInfo(msg=f"record_dir: {record_dir} "))
    ld.add_action(ros2_bag_record)
    return ld
    

def get_record_dir() ->str:
    args = sys.argv # ['/opt/ros/foxy/bin/ros2', 'launch', 'roar-indy-launches', 'record_waypoints.launch.py', 'pointonenav:=true', 'dir:=./neil']
    output = "./default"
    for arg in args:
        if ":=" in arg:
            output = arg.split(":=")[1]
    return output

if __name__ == '__main__':
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
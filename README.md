# ROAR Indy collaboration repo. 
This is the parent working directory for the ROAR indy collaboration. 


## Quick start
Prerequsite:
0. Install ROS2, recommend following the  building from source approach. 
https://docs.ros.org/en/foxy/Installation/Ubuntu-Development-Setup.html

1. Install Carla via Quick Start
https://carla.readthedocs.io/en/latest/start_quickstart/

1. Install Livox SDK
https://github.com/wuxiaohua1011/Livox-SDK

2. Install ZED SDK
https://www.stereolabs.com/developers/release/

3. Make sure carla ros bridge is installed 
https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_installation_ros2/

Install this repo: 

1. clone the repo  

        git clone --recursive https://gitlab.com/wuxiaohua1011/roar-indy-ws.git 

2. colcon build with symlink and source the install

        colcon build --symlink-install
        source install/setup.bash
There might be some warnings being thrown. So long as there are no error, you should be fine. 

3. start Carla server. Download your desired version here:
https://drive.google.com/file/d/1njsDLaZ8j55AbOAHdEnM_OnEriSDK-lC/view?usp=sharing

4. 

*Note: Please refer to each submodule for detailed installation instruction, if any.*

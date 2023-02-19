<!--
 Copyright 2023 michael. All rights reserved.
 Use of this source code is governed by a BSD-style
 license that can be found in the LICENSE file.
-->

# ROAR Indy WS

This repo is the parent repo for the ROAR Go-Kart project. 

## Quick Start
- Prereqs:
  - Install ROS2 Foxy
    - https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
  - [Optional] if using Carla simulator, install `carla-ros-bridge` for ros2
    - https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_installation_ros2/
  - [Optional] If using hardware, make sure to install corresponding SDKs. More documentation provided after cloning, in each respective repo's README

- Installing this repo:
  - Sample command: ```git submodule update --init --recursive src/agent/ src/simulation/```
    - For detail: [Stackoverflow](https://stackoverflow.com/questions/16728866/how-to-only-update-specific-git-submodules)
  - In the `roar-indy-ws` directory, execute ```colcon build```
  - remember to source by `source install/setup.bash` for both the ROS2 install AND this repo's. 


- Usage
  - To main script
    - `./scripts/launch_simulation_client.sh`


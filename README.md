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
  - [Optional] To contribute please install commit linters
    - upgrade NodeJS: https://blog.hubspot.com/website/update-node-js
    - Install linter: https://github.com/legend80s/git-commit-msg-linter


- For Core: 
`vcs import src < core.repos`


- For simulation  
`vcs import src < simulator.repos`

- For hardware
`vcs import src < hardware.repos`


- Installation
  `colcon build`

- Usage
  - Simulation:
    - `./scripts/launch_simulation_server.sh`
    - `./scripts/launch_simulation_client.sh`


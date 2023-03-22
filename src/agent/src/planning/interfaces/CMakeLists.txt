cmake_minimum_required(VERSION 3.5)
project(planning_interfaces)

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Trajectories.msg"
  "msg/TrajectoryScore.msg"
  "srv/TrajectoryScoring.srv"
  "action/TrajectoryGeneration.action"
  DEPENDENCIES nav_msgs
)
ament_package()

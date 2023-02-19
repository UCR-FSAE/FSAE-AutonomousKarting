cmake_minimum_required(VERSION 3.5)
project(global_planner_manager)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_waypoint_follower REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)

link_libraries(stdc++fs)

# global_planner_manager
add_executable(global_planner_manager src/global_planner_manager.cpp)
ament_target_dependencies(global_planner_manager
  rclcpp
  rclcpp_action
  std_msgs
  nav2_msgs
  nav2_util
  nav2_core
  nav2_waypoint_follower
)

target_include_directories(global_planner_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS global_planner_manager
  DESTINATION lib/${PROJECT_NAME})

# waypoint_recorder
add_executable(waypoint_recorder src/waypoint_recorder.cpp)
ament_target_dependencies(waypoint_recorder
  rclcpp
  rclcpp_action
  std_msgs
  nav2_msgs
  nav_msgs
)
target_include_directories(waypoint_recorder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS waypoint_recorder
  DESTINATION lib/${PROJECT_NAME})

# waypoint_follower
add_executable(waypoint_follower src/waypoint_follower.cpp)
ament_target_dependencies(waypoint_follower
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  nav_msgs
  nav2_msgs
  nav2_util
  tf2_ros
)
target_include_directories(waypoint_follower PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS waypoint_follower
  DESTINATION lib/${PROJECT_NAME})

include_directories(
  include
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()

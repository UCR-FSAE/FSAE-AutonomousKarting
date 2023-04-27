cmake_minimum_required(VERSION 3.5)
project(controller_manager)

# find dependencies
find_package(ament_cmake REQUIRED)
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
find_package(visualization_msgs REQUIRED)
find_package(control_interfaces REQUIRED)

include_directories(
  include
)

add_library(controller_manager_core SHARED
  src/controller_manager_ros.cpp
)

set(dependencies
  rclcpp
  rclcpp_action
  std_msgs
  nav2_msgs
  nav2_util
  nav2_core
  control_interfaces
)

ament_target_dependencies(controller_manager_core
  ${dependencies}
)

add_executable(controller_manager src/controller_manager_node.cpp)
ament_target_dependencies(controller_manager
  ${dependencies}
)

target_link_libraries(controller_manager
  controller_manager_core
)

target_include_directories(controller_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
  controller_manager_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  controller_manager
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(controller_manager_core)
ament_export_dependencies(${dependencies})
ament_export_definitions("PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_package()

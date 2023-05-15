cmake_minimum_required(VERSION 3.5)
project(a_star)


find_package(ament_cmake_auto REQUIRED)
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
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(trajectory_generator REQUIRED)
find_package(planning_interfaces REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(a_star SHARED
  src/a_star.cpp
)

ament_auto_package(
  INSTALL_TO_SHARE
)



# include_directories(
#   include
# )

# add_library(a_star_node SHARED
#   src/a_star_node.cpp
# )
# target_compile_definitions(a_star_node PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# set(dependencies
#   rclcpp
#   rclcpp_action
#   std_msgs
#   nav2_msgs
#   nav2_util
#   nav2_core
#   planning_interfaces
#   trajectory_generator
#   tf2_ros
# )

# ament_target_dependencies(a_star_node
#   ${dependencies}
# )


# add_executable(a_star src/a_star.cpp)
# ament_target_dependencies(a_star
#   ${dependencies}
# )

# target_link_libraries(a_star
#   a_star_node
# )

# target_include_directories(a_star PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)


# install(TARGETS
#   a_star_node
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# install(TARGETS
#   a_star
#   RUNTIME DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY include/
#   DESTINATION include/
# )

# ament_export_include_directories(include)
# ament_export_libraries(a_star_node)
# ament_export_libraries(a_star)
# ament_export_dependencies(${dependencies})
# ament_export_definitions("PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# ament_package()

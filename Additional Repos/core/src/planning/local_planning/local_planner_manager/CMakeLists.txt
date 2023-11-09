cmake_minimum_required(VERSION 3.5)
project(local_planner_manager)

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
find_package(trajectory_scorer REQUIRED)
find_package(trajectory_picker REQUIRED)
find_package(planning_interfaces REQUIRED)
find_package(control_interfaces REQUIRED)
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)
find_package(OpenMP REQUIRED)
find_package(smac_planner REQUIRED)
find_package(diagnostic_msgs REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wextra -Wdeprecated -fPIC)
endif()

include_directories(
  include
  ${CERES_INCLUDES}
  ${OMPL_INCLUDE_DIRS}
  ${OpenMP_INCLUDE_DIRS}
)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

if(MSVC)
  add_compile_definitions(_USE_MATH_DEFINES)
endif()

add_executable(local_planner_manager_node src/local_planner_manager_node.cpp)
ament_target_dependencies(local_planner_manager_node
  rclcpp
  rclcpp_action
  std_msgs
  nav2_msgs
  nav2_util
  nav2_core
  trajectory_generator
  trajectory_scorer
  trajectory_picker
  planning_interfaces
  control_interfaces
  eigen3_cmake_module
  smac_planner
  diagnostic_msgs
)
target_link_libraries(local_planner_manager_node ${CERES_LIBRARIES} ${OMPL_LIBRARIES} ${OpenMP_LIBRARIES} OpenMP::OpenMP_CXX)

target_include_directories(local_planner_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS local_planner_manager_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})

ament_package()

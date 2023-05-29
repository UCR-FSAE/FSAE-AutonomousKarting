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
find_package(smac_planner REQUIRED)
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)
find_package(OpenMP REQUIRED)

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

set(library_name a_star_core)
set(dependencies
  rclcpp
  rclcpp_action
  std_msgs
  nav2_msgs
  nav2_util
  nav2_core
  planning_interfaces
  trajectory_generator
  tf2_ros
  eigen3_cmake_module
)

include_directories(include)

add_library(${library_name} SHARED
  src/a_star.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)
target_link_libraries(${library_name} ${CERES_LIBRARIES} ${OMPL_LIBRARIES} ${OpenMP_LIBRARIES} OpenMP::OpenMP_CXX)
target_include_directories(${library_name} PUBLIC ${Eigen3_INCLUDE_DIRS})
target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")


add_executable(a_star src/a_star_node.cpp)
ament_target_dependencies(a_star
  ${dependencies}
)

target_link_libraries(a_star
  a_star_core
)
target_include_directories(a_star PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
  a_star_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  a_star
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(a_star_core)
ament_export_dependencies(${dependencies})
ament_export_definitions("PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_package()

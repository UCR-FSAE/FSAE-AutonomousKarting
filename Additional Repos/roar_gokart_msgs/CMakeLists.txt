cmake_minimum_required(VERSION 3.5)
project(roar_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EgoVehicleControl.msg"
  "msg/VehicleStatus.msg"
  DEPENDENCIES std_msgs
)

ament_package()

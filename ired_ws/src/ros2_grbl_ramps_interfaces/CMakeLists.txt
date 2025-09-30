cmake_minimum_required(VERSION 3.8)
project(ros2_grbl_ramps_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GrblStatus.msg"
  "srv/SendGcode.srv"
  "srv/JogXYZ.srv"
  "srv/Home.srv"
  "srv/Realtime.srv"
  "srv/SetUnits.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

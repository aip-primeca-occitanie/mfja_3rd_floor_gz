cmake_minimum_required(VERSION 3.8)
project(mfja_rail_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NamedState.msg"
  "msg/SwitchCommand.msg"
  "msg/SwitchState.msg"
  "msg/StopperCommand.msg"
  "msg/StopperState.msg"
  "msg/ShuttleCommand.msg"
  "msg/ShuttleState.msg"
  "msg/SensorReading.msg"
  "msg/SensorFeedback.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

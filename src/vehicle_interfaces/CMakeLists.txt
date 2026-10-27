cmake_minimum_required(VERSION 3.16)
project(vehicle_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/State.msg"
  "msg/Version.msg"
  "msg/CanFrame.msg"
  "msg/Acknowledgement.msg"
  "msg/WheelMileage.msg"
  "msg/DriverMotion.msg"
  "msg/DriverStatus.msg"
  "msg/Remote.msg"
  "msg/Sensors.msg"
  "msg/Faults.msg"
  "srv/Command.srv"
  DEPENDENCIES std_msgs builtin_interfaces)
ament_export_dependencies(rosidl_default_runtime)
ament_package()

cmake_minimum_required(VERSION 3.10)
project(vehicle_interfaces)
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)
add_message_files(FILES State.msg Version.msg CanFrame.msg Acknowledgement.msg
  WheelMileage.msg DriverMotion.msg DriverStatus.msg Remote.msg Sensors.msg Faults.msg
  ImuStatus.msg Heading.msg ImuCalibration.msg MotionStatus.msg)
add_service_files(FILES Command.srv ExecuteMotion.srv)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

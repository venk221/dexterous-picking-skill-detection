cmake_minimum_required(VERSION 3.0.2)
project(variable_palm)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  PosCommand.srv
  ReadMotor.srv
  WriteMotor.srv
)


generate_messages(
 DEPENDENCIES
 std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
 CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
)

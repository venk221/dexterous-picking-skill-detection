cmake_minimum_required(VERSION 3.0.2)
project(conveyor_sorting_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  message_generation
)

add_message_files(
  FILES
  SweepingAction.msg
)

add_service_files(
  FILES
  Sweep.srv
  OT.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

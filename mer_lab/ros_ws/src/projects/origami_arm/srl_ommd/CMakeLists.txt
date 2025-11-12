cmake_minimum_required(VERSION 3.0.2)
project(srl_ommd)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES srl_ommd
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
catkin_install_python(PROGRAMS
  scripts/board_test.py
  scripts/fake_vel_cmd.py
  scripts/PID.py
  scripts/__init__.py
  setup.py
  # scripts/low_level_controllers.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
)
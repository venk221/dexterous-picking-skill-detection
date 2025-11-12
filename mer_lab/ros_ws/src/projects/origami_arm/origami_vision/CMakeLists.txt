cmake_minimum_required(VERSION 3.0.2)
project(origami_vision)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  curve_msg.srv
#   Service1.srv
#   Service2.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES origami_vision
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/origami_vision.cpp
# )

# add_executable(${PROJECT_NAME}_node src/vs_control_node.cpp)
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )


catkin_install_python(PROGRAMS
      
scripts/marker_detect.py
# scripts/vs_experiment_vis.py
# scripts/test_depth_image.py
scripts/astar.py
# scripts/skeletonize.py
# scripts/test_skeletonize.py
# scripts/test.py
scripts/__init__.py
setup.py
scripts/clothoid_fit_server.py
# scripts/clothoid_fit_client.py

DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
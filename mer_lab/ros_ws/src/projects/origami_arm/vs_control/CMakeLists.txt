cmake_minimum_required(VERSION 3.0.2)
project(vs_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SingleModuleJacobian.srv
  TwoModuleJacobian.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
 LIBRARIES vs_control
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(origami_control_node src/controller.cpp)
add_dependencies(origami_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(origami_control_node ${catkin_LIBRARIES})

add_executable(origami_vs_data_record_node src/recorder.cpp)
add_dependencies(origami_vs_data_record_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(origami_vs_data_record_node ${catkin_LIBRARIES})

add_executable(origami_vs_simple_init_node src/initialize_module.cpp)
add_dependencies(origami_vs_simple_init_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(origami_vs_simple_init_node ${catkin_LIBRARIES})

 catkin_install_python(PROGRAMS
      
      scripts/marker_detect.py
      scripts/single_module_jacobian_server.py
      scripts/single_module_jacobian_client.py
      scripts/two_module_jacobian_server.py
      scripts/two_module_jacobian_client.py
      scripts/experiment_vis.py
      scripts/single_module_Jacobian

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

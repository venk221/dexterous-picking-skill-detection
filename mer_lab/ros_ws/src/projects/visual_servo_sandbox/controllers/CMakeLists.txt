cmake_minimum_required(VERSION 3.0.2)
project(controllers)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
)

add_service_files(
  FILES
  jacobian.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ibvs_controller_node src/ibvs_controller.cpp)
add_dependencies(ibvs_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ibvs_controller_node ${catkin_LIBRARIES})

add_executable(two_link_jac_srv src/compute_kinematics.cpp)
add_dependencies(two_link_jac_srv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(two_link_jac_srv ${catkin_LIBRARIES})

add_executable(hardware_api_node src/sim_robot_interface.cpp)
add_dependencies(hardware_api_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hardware_api_node ${catkin_LIBRARIES})

add_executable(target_node src/reference_pub.cpp)
add_dependencies(target_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(target_node ${catkin_LIBRARIES})
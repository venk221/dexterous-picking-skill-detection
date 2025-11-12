cmake_minimum_required(VERSION 3.0.2)
project(origami_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SingleModuleJacobian.srv
  TwoModuleJacobian.srv
  init_estimate.srv
  adaptive_update.srv
  skeleton_init_estimate.srv
  skeleton_adaptive_update.srv
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
#  LIBRARIES origami_control
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/origami_control.cpp
# )

# add_executable(${PROJECT_NAME}_node src/vs_control_node.cpp)
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

add_executable(origami_vs_control_node src/archive/vs_control.cpp)
add_dependencies(origami_vs_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(origami_vs_control_node ${catkin_LIBRARIES})

# add_executable(origami_vs_record_node src/recorder.cpp)
# add_dependencies(origami_vs_record_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(origami_vs_record_node ${catkin_LIBRARIES})

add_executable(origami_vs_init_node src/archive/initialize_module.cpp)
add_dependencies(origami_vs_init_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(origami_vs_init_node ${catkin_LIBRARIES})

add_executable(origami_adaptive_vs_node src/archive/adaptive_vs.cpp)
add_dependencies(origami_adaptive_vs_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(origami_adaptive_vs_node ${catkin_LIBRARIES})

add_executable(origami_estimation_node src/archive/initial_estimate.cpp)
add_dependencies(origami_estimation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(origami_estimation_node ${catkin_LIBRARIES})

add_executable(origami_adaptive_update_node src/archive/adaptive_update.cpp)
add_dependencies(origami_adaptive_update_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(origami_adaptive_update_node ${catkin_LIBRARIES})

add_executable(origami_motion_test_node src/archive/motion_test.cpp)
add_dependencies(origami_motion_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(origami_motion_test_node ${catkin_LIBRARIES})

add_executable(origami_skeleton_vs_node src/origami_skeleton_vs.cpp)
add_dependencies(origami_skeleton_vs_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(origami_skeleton_vs_node ${catkin_LIBRARIES})

add_executable(origami_skeleton_estimation_node src/skeleton_initial_estimate.cpp)
add_dependencies(origami_skeleton_estimation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(origami_skeleton_estimation_node ${catkin_LIBRARIES})

add_executable(origami_skeleton_adaptive_update_node src/skeleton_adaptive_update.cpp)
add_dependencies(origami_skeleton_adaptive_update_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(origami_skeleton_adaptive_update_node ${catkin_LIBRARIES})

add_executable(multi_origami_skeleton_vs_node src/archive/multi_skeleton_vs.cpp)
add_dependencies(multi_origami_skeleton_vs_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(multi_origami_skeleton_vs_node ${catkin_LIBRARIES})

add_executable(multi_module_motion_test_node src/archive/multi_module_motion_test.cpp)
add_dependencies(multi_module_motion_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(multi_module_motion_test_node ${catkin_LIBRARIES})

add_executable(origami_velocity_interface_node src/velocity_msg_interface.cpp)
add_dependencies(origami_velocity_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(origami_velocity_interface_node ${catkin_LIBRARIES})

add_executable(origami_keyboard_node src/keyboard_control.cpp)
add_dependencies(origami_keyboard_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(origami_keyboard_node ${catkin_LIBRARIES})

add_executable(origami_kappa_test_node src/feature_client.cpp)
add_dependencies(origami_kappa_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(origami_kappa_test_node ${catkin_LIBRARIES})

# add_executable(origami_clothoid_node src/clothoid_test.cpp)
# add_dependencies(origami_clothoid_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(origami_clothoid_node ${catkin_LIBRARIES} libClothoids_linux.so)

add_executable(origami_sim_ibvs_node src/origami_sim_ibvs.cpp)
add_dependencies(origami_sim_ibvs_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(origami_sim_ibvs_node ${catkin_LIBRARIES})

add_executable(origami_sim_ref_pub_node src/origami_sim_ref_pub.cpp)
add_dependencies(origami_sim_ref_pub_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(origami_sim_ref_pub_node ${catkin_LIBRARIES})

 catkin_install_python(PROGRAMS
      
      scripts/archive/single_module_jacobian_server.py
      scripts/archive/single_module_jacobian_client.py
      scripts/curve_ref_gen.py
      scripts/reference_clothoid.py
      scripts/simulation_control_interface.py
      # scripts/keyboard_control.py
      
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

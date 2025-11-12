cmake_minimum_required(VERSION 3.0.2)
project(encoderless_vs)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rosbag
  cv_bridge
  image_transport
  message_generation
  sensor_msgs
  Eigen3
)

catkin_python_setup()


find_package(Boost REQUIRED COMPONENTS system)
# find_package(Armadillo REQUIRED)

## Generate services in the 'srv' folder
 add_service_files(
  FILES
  energyFuncMsg.srv
  control_points.srv
  bin_img.srv
  cp_goal.srv
  cp_test.srv
  franka_control_points.srv
  franka_control_points_test.srv
  franka_bin_img.srv
  franka_cp_goal.srv
  franka_cp_vis.srv
  franka_vis_bin_img.srv
  three_link_test.srv
  three_link_bin_test.srv
  vel_start.srv
  dream_goal_img.srv
  dream_kp.srv
  dream_img.srv
 )


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES encoderless_vs
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  # ${ARMADILLO_INCLUDE_DIRS}
)

add_executable(energy_func_node src/compute_energy_functional.cpp)
add_dependencies(energy_func_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(energy_func_node ${catkin_LIBRARIES})

add_executable(redundant_energy_func_node src/redundant_compute_energy_functional.cpp)
add_dependencies(redundant_energy_func_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(redundant_energy_func_node ${catkin_LIBRARIES})

add_executable(franka_energy_func_node src/franka_compute_energy_functional.cpp)
add_dependencies(franka_energy_func_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(franka_energy_func_node ${catkin_LIBRARIES})

add_executable(franka_shape_energy_func_node src/franka_shape_compute_energy_functional.cpp)
add_dependencies(franka_shape_energy_func_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(franka_shape_energy_func_node ${catkin_LIBRARIES})

add_executable(servo_controller_node src/servo_controller.cpp)
add_dependencies(servo_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(servo_controller_node ${catkin_LIBRARIES})

add_executable(vsbot_record_node src/recorder.cpp)
add_dependencies(vsbot_record_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vsbot_record_node ${catkin_LIBRARIES})

add_executable(redundant_record_node src/redundant_recorder.cpp)
add_dependencies(redundant_record_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(redundant_record_node ${catkin_LIBRARIES})

# add_executable(vsbot_vis_node src/visualization.cpp)
# add_dependencies(vsbot_vis_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(vsbot_vis_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(shape_servo_controller_node src/shape_servo_controller.cpp)
add_dependencies(shape_servo_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(shape_servo_controller_node ${catkin_LIBRARIES} ${ARMADILL_LIBRARIES})

add_executable(time_recorder_node src/time_recorder.cpp)
add_dependencies(time_recorder_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(time_recorder_node ${catkin_LIBRARIES})

add_executable(test_cp_node src/test_control_points.cpp)
add_dependencies(test_cp_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_cp_node ${catkin_LIBRARIES})

add_executable(franka_joint_pub src/franka_joint_pub.cpp)
add_dependencies(franka_joint_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(franka_joint_pub ${catkin_LIBRARIES})

add_executable(franka_adaptive_vs_node src/franka_adaptive_vs.cpp)
add_dependencies(franka_adaptive_vs_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(franka_adaptive_vs_node ${catkin_LIBRARIES})

add_executable(franka_vel_test_node src/franka_vel_test.cpp)
add_dependencies(franka_vel_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(franka_vel_test_node ${catkin_LIBRARIES})

add_executable(franka_data_record_node src/franka_adaptive_vs_recorder.cpp)
add_dependencies(franka_data_record_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(franka_data_record_node ${catkin_LIBRARIES})

add_executable(franka_shape_record_node src/franka_shape_vs_recorder.cpp)
add_dependencies(franka_shape_record_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(franka_shape_record_node ${catkin_LIBRARIES})

add_executable(franka_shape_controller_node src/franka_shape_vs.cpp)
add_dependencies(franka_shape_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(franka_shape_controller_node ${catkin_LIBRARIES})

add_executable(redundant_shape_controller_node src/redundant_shape_servo_controller.cpp)
add_dependencies(redundant_shape_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(redundant_shape_controller_node ${catkin_LIBRARIES})

add_executable(franka_dream_controller_node src/franka_dream_vs.cpp)
add_dependencies(franka_dream_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(franka_dream_controller_node ${catkin_LIBRARIES})

add_executable(franka_dream_energy_func_node src/franka_dream_compute_energy_functional.cpp)
add_dependencies(franka_dream_energy_func_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(franka_dream_energy_func_node ${catkin_LIBRARIES})

add_executable(franka_multiref_shape_vs_node src/franka_multiref_shape_vs.cpp)
add_dependencies(franka_multiref_shape_vs_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(franka_multiref_shape_vs_node ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
   scripts/test_nodes/vsbot_joint_controller.py
   scripts/test_nodes/3linkbot_joint_controller.py
   scripts/old_nodes/vsbot_marker_detection.py
   scripts/vsbot_vis_adaptive_vs.py
   scripts/vsbot_vis_shape_servo.py
   scripts/vsbot_bin_svc_server.py
   scripts/vsbot_cp_svc_server.py
   scripts/test_nodes/vsbot_cp_svc_server_test.py
   scripts/test_nodes/vsbot_cp_svc_client_test.py
   scripts/vsbot_cp_svc_client.py
   scripts/vsbot_goal_bin_svc.py
   scripts/vsbot_goal_cp_svc.py
   scripts/vsbot_goal_img_pub.py
   scripts/depth/vsdepth_bin_svc_server.py
   scripts/depth/vsdepth_cp_svc_server.py
   scripts/depth/vsdepth_cp_svc_client.py
   scripts/depth/vsbot_image_sub.py
   scripts/depth/vsdepth_real_preprocess.py
   scripts/depth/vsdepth_real_joint_publisher.py
   scripts/depth/vsdepth_aruco_test.py
   scripts/marker_detect.py
   scripts/franka_adaptive_vs_vis_node.py
   scripts/franka_vs_vis_node.py
   scripts/franka_visual_servoing.py
   scripts/franka_plot_node.py
   scripts/franka_bin_svc_server.py
   scripts/franka_cp_svc_server.py
   scripts/franka_cp_svc_client.py
   scripts/franka_goal_bin_svc.py
   scripts/franka_goal_cp_svc.py
   scripts/franka_goal_cp_svc_rqst.py
   scripts/franka_goal_img_pub.py
   scripts/test_nodes/franka_vis_bin_svc.py
   scripts/test_nodes/franka_vis_cp_svc.py
   scripts/test_nodes/franka_vis_cp_svc_rqst.py
   scripts/franka_joint_impedance.py
   scripts/franka_joint_impedance_client.py
   scripts/img_write.py
   scripts/depth_write.py
   scripts/test_nodes/3link_bin_svc_test.py
   scripts/test_nodes/3link_cp_svc_test.py
   scripts/test_nodes/3link_cp_client_test.py
   scripts/3link_bin_svc.py
   scripts/3link_cp_svc.py
   scripts/3link_cp_client.py
   scripts/3link_goal_bin_svc.py
   scripts/3link_goal_cp_svc.py
   scripts/3link_goal_cp_svc_rqst.py
   scripts/3link_goal_img_pub.py   
   scripts/3link_goal_img_pub.py
   scripts/redundant_vis.py
   scripts/test_nodes/launch_skel_vs_dream_ros.py
   scripts/franka_dream_client.py
   scripts/franka_dream_kp_svc.py
   scripts/franka_dream_vs_vis_node.py
   scripts/kalman/franka_goal_dream_client.py
   scripts/kalman/franka_goal_dream_svc.py
   scripts/dream_goal_img_pub.py
   scripts/franka_skel_grasp.py
   scripts/multiref.py
   scripts/22dr_camera_pub.py
   scripts/22dr_controller_pub.py
   scripts/kalman/const_vel_missing_feature_kalman_filter.py
   scripts/kalman/franka_vel_missing_feature_detector.py
   scripts/kalman/franka_pos_dream_depth_ds_gen.py
   scripts/kalmanfilter.py
   scripts/__init__.py
   setup.py  


   
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

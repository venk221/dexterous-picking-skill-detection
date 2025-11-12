cmake_minimum_required(VERSION 3.8)
project(scrap_burning)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-std=c++17)

find_package(PCL REQUIRED COMPONENTS
  registration
  surface
  filters
  common
  io
  visualization
  search
  kdtree
  features
  segmentation
  )

# Figure out which libraries to link to depending on whether we're in the experiment or simulation
option(PCL_LINK_LAB_LIBRARIES OFF)
if(PCL_LINK_LAB_LIBRARIES)
  message("Linking to PCL lab libraries")
  set(NEW_PCL_LIBS
    ${PCL_LIBRARIES}
    /usr/local/lib/libpcl_surface.so.1.11
    /usr/local/lib/libpcl_common.so.1.11
    /usr/lib/x86_64-linux-gnu/libflann_cpp.so.1.9
    /usr/local/lib/libpcl_octree.so.1.11
    /usr/local/lib/libpcl_io.so.1.11
    /usr/local/lib/libpcl_filters.so.1.11
    /usr/local/lib/libpcl_registration.so.1.11
    /usr/local/lib/libpcl_kdtree.so.1.11
    /usr/local/lib/libpcl_features.so.1.11
    /usr/local/lib/libpcl_recognition.so.1.11
    /usr/local/lib/libpcl_visualization.so.1.11
    /usr/local/lib/libpcl_search.so.1.11)
else()
  message("Linking to PCL sim libraries")
  set(NEW_PCL_LIBS
    ${PCL_LIBRARIES})
endif()

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospack
  geometry_msgs
  message_generation
  message_runtime
  moveit_planner
  roscpp
  sensor_msgs
  gazebo_msgs
  geometry_msgs
  ros_pcl_manip
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  tf
  visualization_msgs
  ros_pcl_manip
  qnd
  )

find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)

set(CURSES_NEED_NCURSES TRUE)
find_package(Curses REQUIRED)

find_package(qnd REQUIRED)

add_service_files(
  FILES
  CurveFitting.srv
  FilterPath.srv
  PCLConcatRequest.srv
  Skeletonize.srv
  Traverse.srv
  RecordRequest.srv
  AddToRecord.srv
  BeginActive.srv
  ComputeNextBest.srv
  AddTrajWaypoint.srv
  StopRecordingTraj.srv
  StartRecordingTraj.srv
  OutputRecordedTraj.srv
  )

generate_messages(
  DEPENDENCIES
  sensor_msgs
  geometry_msgs
  std_msgs
  )

catkin_package()

add_subdirectory(ext/octomap/octomap)
# octomap target does not include the necessaray folders as PUBLIC headers, so we need to patch it here
set(octomap_INCLUDE_DIRS ext/octomap/octomap/include)

include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

link_directories(${PCL_LIBRARY_DIRS})

add_executable(ground_truth_generator_node src/benchmarking/ground_truth_generator.cpp)
target_include_directories(ground_truth_generator_node PRIVATE ${catkin_INCLUDE_DIRS} include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(ground_truth_generator_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${NEW_PCL_LIBS})

add_executable(skeleton_vis_node src/benchmarking/skeleton_vis.cpp)
target_include_directories(skeleton_vis_node PRIVATE ${catkin_INCLUDE_DIRS} PRIVATE include)
target_link_libraries(skeleton_vis_node PRIVATE ${catkin_LIBRARIES} PRIVATE ${NEW_PCL_LIBS})
add_dependencies(skeleton_vis_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(filter_vis_node src/benchmarking/filter_vis.cpp)
target_include_directories(filter_vis_node PRIVATE ${catkin_INCLUDE_DIRS} PRIVATE include PRIVATE ${CURSES_INCLUDE_DIRS})
target_link_libraries(filter_vis_node PRIVATE ${catkin_LIBRARIES} PRIVATE ${CURSES_LIBRARIES} PRIVATE ${NEW_PCL_LIBS})
add_dependencies(filter_vis_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(metric_calculator_node src/benchmarking/metric_calculator.cpp)
target_include_directories(metric_calculator_node PRIVATE ${catkin_INCLUDE_DIRS} PRIVATE include)
target_link_libraries(metric_calculator_node PRIVATE ${catkin_LIBRARIES} PRIVATE ${NEW_PCL_LIBS})
add_dependencies(metric_calculator_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(movement_test_node src/movement_test.cpp)
target_include_directories(movement_test_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(movement_test_node ${catkin_LIBRARIES})
add_dependencies(movement_test_node moveit_planner_generate_messages)

# add_executable(col_octree_test src/col_octree_test.cpp)
# target_link_libraries(col_octree_test octomap-static octomath-static)

add_executable(scrap_spawn src/scrap_spawn_node.cpp)
target_include_directories(scrap_spawn PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(scrap_spawn ${catkin_LIBRARIES} ${NEW_PCL_LIBS})

add_executable(octomap_concat_node src/octomap_concat.cpp)
target_link_libraries(octomap_concat_node octomap-static octomath-static ${catkin_LIBRARIES})
target_include_directories(octomap_concat_node PRIVATE ${octomap_INCLUDE_DIRS} PRIVATE ${catkin_INCLUDE_DIRS})
add_dependencies(octomap_concat_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(path_seg_node src/path_seg_node.cpp)
add_dependencies(path_seg_node ros_pcl_manip_generate_messages ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_include_directories(path_seg_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(path_seg_node ${catkin_LIBRARIES} ${NEW_PCL_LIBS})

add_executable(octomap_analysis_node src/octomap_analysis.cpp)
target_include_directories(octomap_analysis_node PRIVATE ${octomap_INCLUDE_DIRS})
target_link_libraries(octomap_analysis_node octomap-static octomath-static)

# Active vision stuff
add_library(active_vision_lib src/active_vision/camera.cpp src/active_vision/grid.cpp src/active_vision/active.cpp)
target_include_directories(active_vision_lib PUBLIC include PUBLIC ${octomap_INCLUDE_DIRS} PUBLIC qnd)
target_link_libraries(active_vision_lib ${catkin_LIBRARIES} ${NEW_PCL_LIBS} qnd)

add_executable(active_vision_node src/active_vision/active_vision_node.cpp)
# NOTE: Although linking to active_vision_lib does pull in the octomap include directories, it will still cause issues when we include catkin since catkin's octomap headers will supersede it
#       So, we set the include dirs here in the order we want before linking to ensure OUR octomap headers get found first
target_include_directories(active_vision_node PRIVATE ${octomap_INCLUDE_DIRS} PRIVATE ${catkin_INCLUDE_DIRS} PRIVATE qnd)
target_link_libraries(active_vision_node octomap-static octomath-static active_vision_lib ${catkin_LIBRARIES} qnd)

find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# Commented out since we are slowly eliminating the PCL curve fitting methods
add_executable(curve_fitting src/curve_fitting.cpp)
target_link_libraries(curve_fitting ${NEW_PCL_LIBS} ${catkin_LIBRARIES})
target_include_directories(curve_fitting PRIVATE ${catkin_INCLUDE_DIRS})
add_dependencies(curve_fitting ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(concat_node src/point_concat.cpp)
target_link_libraries(concat_node ${NEW_PCL_LIBS} ${catkin_LIBRARIES})
target_include_directories(concat_node PRIVATE ${catkin_INCLUDE_DIRS} PRIVATE include)
add_dependencies(concat_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS scripts/skeletonize.py
  DESTINATION $CATKIN_PACKAGE_BIN_DESTINATION)

# Commented out since we are slowly eliminating the PCL curve fitting methods
# add_executable(test_fit src/fit_tut.cpp)
# target_link_libraries(test_fit ${NEW_PCL_LIBS})

add_executable(traj_vis src/benchmarking/traj_vis.cpp)
target_link_libraries(traj_vis ${NEW_PCL_LIBS} ${catkin_LIBRARIES})
target_include_directories(traj_vis PRIVATE ${catkin_INCLUDE_DIRS} PRIVATE include)

add_executable(benchmark_fit_node src/benchmark_fit.cpp)
target_link_libraries(benchmark_fit_node ${NEW_PCL_LIBS} ${catkin_LIBRARIES})
target_include_directories(benchmark_fit_node PRIVATE ${catkin_INCLUDE_DIRS})
add_dependencies(benchmark_fit_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(path_follow_node src/path_follow_node.cpp)
target_link_libraries(path_follow_node ${catkin_LIBRARIES})
target_include_directories(path_follow_node PRIVATE ${catkin_INCLUDE_DIRS} PRIVATE include)
add_dependencies(path_follow_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(add_noise_node src/add_noise.cpp)
target_include_directories(add_noise_node PRIVATE ${catkin_INCLUDE_DIRS} PRIVATE include)
target_link_libraries(add_noise_node ${catkin_LIBRARIES})

add_executable(traverse_node src/traversal_node.cpp)
target_link_libraries(traverse_node ${catkin_LIBRARIES})
target_include_directories(traverse_node PRIVATE ${catkin_INCLUDE_DIRS})
add_dependencies(traverse_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(pcl_to_csv_node src/pcl_to_csv.cpp)
target_link_libraries(pcl_to_csv_node ${NEW_PCL_LIBS})

add_executable(traj_recorder_node src/path_recorder.cpp)
target_include_directories(traj_recorder_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(traj_recorder_node ${catkin_LIBRARIES})
add_dependencies(traj_recorder_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

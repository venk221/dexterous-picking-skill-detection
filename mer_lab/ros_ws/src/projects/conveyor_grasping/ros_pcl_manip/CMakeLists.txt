cmake_minimum_required(VERSION 2.8.3)
project(ros_pcl_manip)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  message_generation
  message_runtime)
find_package(PCL REQUIRED COMPONENTS
  common
  filters
  sample_consensus
  segmentation
  search
  kdtree
  features
  recognition
  console
  io
  visualization)

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

add_service_files(
  FILES
  Downsample.srv
  SegmentPlane.srv
  ESeg.srv
  CorrGroup.srv
  LoadFile.srv
  ToFile.srv
  Filter.srv
  NormalEst.srv
  OutlierRemoval.srv
  Voxelize.srv)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs)

catkin_package(
  INCLUDE_DIRS include)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_executable(ros_pcl_manip_node src/ros_pcl_manip_node)
target_link_libraries(ros_pcl_manip_node ${catkin_LIBRARIES} ${NEW_PCL_LIBS} ${PCL_LIBRARIES})
add_dependencies(ros_pcl_manip_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(test_vision_node src/test_vision_node)
target_link_libraries(test_vision_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(test_vision_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

# add_executable(to_pcd_node src/toPCD.cpp)
# target_link_libraries(to_pcd_node ${PCL_LIBRARIES})

# add_executable(correspondence_node src/correspondence.cpp)
# target_link_libraries(correspondence_node ${PCL_LIBRARIES})

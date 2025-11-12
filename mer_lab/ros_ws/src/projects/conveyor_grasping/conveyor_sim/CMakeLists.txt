cmake_minimum_required(VERSION 2.8.3)
project(conveyor_sim)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_msgs
  message_generation
  message_runtime
  gazebo_plugins
)
find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

add_service_files(
  FILES
  SpawnTrigger.srv)

add_message_files(
  FILES
  MetaImage.msg
  Box.msg)

generate_messages(
  DEPENDENCIES sensor_msgs)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} include)

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_executable(conveyor_sim_node src/conveyor_sim.cpp)
target_link_libraries(conveyor_sim_node ${catkin_LIBRARIES})

add_executable(conveyor_object_spawner src/object_spawner.cpp)
target_link_libraries(conveyor_object_spawner ${catkin_LIBRARIES})
add_dependencies(conveyor_object_spawner ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(conveyor_object_despawner src/object_despawner.cpp)
target_link_libraries(conveyor_object_despawner ${catkin_LIBRARIES})

add_executable(bounding_box_generator_node src/bounding_box_generator.cpp)
target_link_libraries(bounding_box_generator_node ${catkin_LIBRARIES})

# Conveyor Plugin
# Commented out for now since it is failing on noetic
# if(${gazebo_VERSION_MAJOR} EQUAL "11")
#   message("Conveyor Sim - Gazebo version 11 found, adding plugins")
#   add_library(ConveyorBeltPlugin SHARED plugins/gazebo_conveyor_plugin.cc)
#   add_library(BoundingBoxCameraPlugin SHARED plugins/gazebo_bb_camera_plugin.cc)
#   target_link_libraries(ConveyorBeltPlugin ${GAZEBO_LIBRARIES})
#   target_link_libraries(BoundingBoxCameraPlugin ${GAZEBO_LIBRARIES} CameraPlugin ${catkin_LIBRARIES} Eigen3::Eigen)
#   add_dependencies(BoundingBoxCameraPlugin ${${PROJECT_NAME}_EXPORTED_TARGETS})
#   install(TARGETS ConveyorBeltPlugin BoundingBoxCameraPlugin
#     ARCHIVE DESTINATION lib
#     LIBRARY DESTINATION lib
#     RUNTIME DESTINATION lib
#     )
# endif()

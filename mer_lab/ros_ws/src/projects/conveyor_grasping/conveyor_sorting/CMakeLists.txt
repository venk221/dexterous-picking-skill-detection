cmake_minimum_required(VERSION 3.0.2)
project(conveyor_sorting)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  geometry_msgs
  gazebo_msgs
  cv_bridge
  moveit_planner
  tf2
  tf2_ros
  tf2_geometry_msgs
  moveit_msgs
  shape_msgs
  conveyor_sorting_msgs)
find_package(OpenCV REQUIRED)

find_package(qnd REQUIRED)

catkin_package()

# Build pugixml
add_subdirectory(ext/pugixml)

add_executable(pile_spawner_node src/pile_spawner.cpp)
target_link_libraries(pile_spawner_node PRIVATE ${catkin_LIBRARIES} PRIVATE stdc++fs PRIVATE pugixml PRIVATE qnd)
target_include_directories(pile_spawner_node PRIVATE ${catkin_INCLUDE_DIRS} PRIVATE include)
set_target_properties(pile_spawner_node PROPERTIES
  CXX_STANDARD 17)

add_executable(topographical_mapper_node src/topographical_mapper.cpp)
target_link_libraries(topographical_mapper_node PRIVATE ${catkin_LIBRARIES} PRIVATE ${OpenCV_LIBRARIES})
target_include_directories(topographical_mapper_node PRIVATE ${catkin_INCLUDE_DIRS} PRIVATE ${OpenCV_INCLUDE_DIRS} PRIVATE include)

add_executable(display_topmap_node src/display_topmap.cpp)
target_link_libraries(display_topmap_node PRIVATE ${catkin_LIBRARIES} PRIVATE ${OpenCV_LIBRARIES})
target_include_directories(display_topmap_node PRIVATE ${catkin_INCLUDE_DIRS} PRIVATE ${OpenCV_INCLUDE_DIRS} PRIVATE include)

add_executable(table_creator_node src/table_creator.cpp)
target_link_libraries(table_creator_node PRIVATE ${catkin_LIBRARIES} PRIVATE pugixml PRIVATE qnd)
target_include_directories(table_creator_node PRIVATE ${catkin_INCLUDE_DIRS} PRIVATE include)

add_executable(ot_sweeper_node src/ot_sweeper.cpp)
target_link_libraries(ot_sweeper_node PRIVATE ${catkin_LIBRARIES} PRIVATE ${OpenCV_LIBRARIES} PRIVATE pugixml PRIVATE qnd)
target_include_directories(ot_sweeper_node PRIVATE ${catkin_INCLUDE_DIRS} PRIVATE ${OpenCV_INCLUDE_DIRS} PRIVATE include)
add_dependencies(ot_sweeper_node ${conveyor_sorting_msgs_EXPORTED_TARGETS})

add_executable(sweeper_node src/sweeper.cpp)
target_link_libraries(sweeper_node PRIVATE ${catkin_LIBRARIES} PRIVATE qnd)
target_include_directories(sweeper_node PRIVATE ${catkin_INCLUDE_DIRS} PRIVATE include)
add_dependencies(sweeper_node ${conveyor_sorting_msgs_EXPORTED_TARGETS})

add_executable(qnd_tester_node src/qnd_tester.cpp)
target_link_libraries(qnd_tester_node PRIVATE ${catkin_LIBRARIES} PRIVATE qnd)
target_include_directories(qnd_tester_node PRIVATE ${catkin_INCLUDE_DIRS} PRIVATE include)

add_executable(scorer_node src/scorer.cpp)
target_link_libraries(scorer_node PRIVATE ${catkin_LIBRARIES} PRIVATE qnd)
target_include_directories(scorer_node PRIVATE ${catkin_INCLUDE_DIRS} PRIVATE include)

add_executable(depth_translator_node src/depth_translator.cpp)
target_link_libraries(depth_translator_node PRIVATE ${catkin_LIBRARIES} PRIVATE qnd PRIVATE ${OpenCV_LIBRARIES})
target_include_directories(depth_translator_node PRIVATE ${catkin_INCLUDE_DIRS} PRIVATE include PRIVATE ${OpenCV_INCLUDE_DIRS})

add_executable(depth_viewer_node src/dep_viewer.cpp)
target_link_libraries(depth_viewer_node PRIVATE ${catkin_LIBRARIES} PRIVATE qnd PRIVATE ${OpenCV_LIBRARIES})
target_include_directories(depth_viewer_node PRIVATE ${catkin_INCLUDE_DIRS} PRIVATE include PRIVATE ${OpenCV_INCLUDE_DIRS})

option(BUILD_TESTS "Enable testing of optimal transport code" OFF)
if(${BUILD_TESTS})
  find_package(GTest REQUIRED)

  include(CTest)

  add_executable(ot_tester_node src/ot_tester.cpp)
  target_include_directories(ot_tester_node PRIVATE ${catkin_INCLUDE_DIRS} PRIVATE include PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(ot_tester_node PRIVATE gtest_main PRIVATE ${OpenCV_LIBRARIES} PRIVATE ${catkin_LIBRARIES} PRIVATE qnd)
endif()

install(PROGRAMS src/pot_solver.py src/pot_solver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

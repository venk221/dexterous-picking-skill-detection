cmake_minimum_required(VERSION 3.0.2)
project(srl_sim)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_python_setup()


catkin_package(
#  INCLUDE_DIRS include
 LIBRARIES srl_sim
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
catkin_install_python(PROGRAMS
  # scripts/sim_FK.py
  scripts/continuum_misc.py
  scripts/plot.py
  scripts/__init__.py
  setup.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
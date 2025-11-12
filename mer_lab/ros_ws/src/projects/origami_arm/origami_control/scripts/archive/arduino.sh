#!/bin/bash

# This script runs sets up serial comm with arduino through a ROS node

# Start roscore
cd ~/ &&
./mer_lab/ros_ws/src/projects/origami_arm/vs_control/scripts/core.sh  &&

# Run rosserial

source /opt/ros/noetic/setup.bash &&
rosrun rosserial_arduino serial_node.py _port:=/dev/ttyACM0
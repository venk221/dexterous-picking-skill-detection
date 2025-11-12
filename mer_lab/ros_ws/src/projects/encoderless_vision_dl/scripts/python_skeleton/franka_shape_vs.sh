#!/bin/bash
mkdir ~/.ros/raw &&
mkdir ~/.ros/d_raw &&
cd ~/mer_lab/ros_ws

source devel/setup.bash

roslaunch encoderless_vision_dl franka_shape_vs.launch &&
# roslaunch encoderless_vs franka_shape_multiref_vs.launch &&
roslaunch encoderless_vision_dl franka_shape_plot.launch &&

cd ~/mer_lab/ros_ws/src/projects/encoderless_vision_dl/scripts/python_skeleton/

./franka_export_shape_results.sh

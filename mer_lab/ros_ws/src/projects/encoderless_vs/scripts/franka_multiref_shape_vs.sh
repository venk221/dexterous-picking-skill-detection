#!/bin/bash
mkdir ~/.ros/raw &&
mkdir ~/.ros/d_raw &&
cd ~/mer_lab/ros_ws

source devel/setup.bash

roslaunch encoderless_vs franka_multiref_shape_vs.launch &&

roslaunch encoderless_vs franka_shape_plot.launch &&

cd ~/mer_lab/ros_ws/src/projects/encoderless_vs/scripts/

./franka_export_shape_results.sh

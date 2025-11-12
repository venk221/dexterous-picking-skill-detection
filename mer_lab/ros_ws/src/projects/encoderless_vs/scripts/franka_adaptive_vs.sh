#!/bin/bash
mkdir ~/.ros/raw &&

cd ~/mer_lab/ros_ws

source devel/setup.bash

roslaunch encoderless_vs franka_adaptive_vs.launch &&

roslaunch encoderless_vs franka_plot.launch &&

cd ~/mer_lab/ros_ws/src/projects/encoderless_vs/scripts/

./franka_export_results.sh


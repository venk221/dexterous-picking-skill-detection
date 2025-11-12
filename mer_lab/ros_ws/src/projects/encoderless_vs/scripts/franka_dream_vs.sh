#!/bin/bash
mkdir ~/.ros/raw &&
mkdir ~/.ros/d_raw &&
cd ~/mer_lab/ros_ws

source devel/setup.bash

roslaunch encoderless_vs franka_dream_service.launch i:=/home/merlab/DREAM/trained_models/panda_dream_resnet_h.pth b:=panda_link0 &&

# roslaunch encoderless_vs franka_dream_service.launch &&

# roslaunch encoderless_vs franka_shape_plot.launch &&

cd ~/mer_lab/ros_ws/src/projects/encoderless_vs/scripts/

./franka_export_shape_results.sh

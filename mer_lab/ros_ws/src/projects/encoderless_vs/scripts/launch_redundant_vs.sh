#!/bin/bash
mkdir ~/.ros/raw &&
cd ~/mer_lab/ros_ws

roslaunch encoderless_vs redundant_shape_vs.launch &&

cd ~/mer_lab/ros_ws/src/projects/encoderless_vs/scripts/

# ./export_results.sh

./export_redundant_shape_results.sh


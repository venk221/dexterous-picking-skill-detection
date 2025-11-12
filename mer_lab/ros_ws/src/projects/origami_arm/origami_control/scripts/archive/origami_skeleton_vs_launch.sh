#!/bin/bash

cd ~/mer_lab/ros_ws

source devel/setup.bash

roslaunch origami_control skeleton_vs.launch &&

cd ~/mer_lab/ros_ws/src/projects/origami_arm/origami_log/scripts/

./skeleton_bag_to_csv.sh
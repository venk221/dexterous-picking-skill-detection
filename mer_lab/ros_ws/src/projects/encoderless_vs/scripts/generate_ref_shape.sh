#!/bin/bash

cd ~/mer_lab/ros_ws

roslaunch encoderless_vs goal_feature.launch &&

# cd ~/mer_lab/ros_ws/src/projects/encoderless_vs/scripts/

# ./export_results.sh

# ./export_shape_results.sh

mv ~/.ros/features.yaml ~/mer_lab/ros_ws/src/projects/encoderless_vs/config/
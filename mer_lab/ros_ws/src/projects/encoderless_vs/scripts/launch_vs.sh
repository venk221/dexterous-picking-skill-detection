#!/bin/bash

cd ~/mer_lab/ros_ws

source devel/setup.bash

roslaunch encoderless_vs servo_controller_test.launch &&

cd ~/mer_lab/ros_ws/src/projects/encoderless_vs/scripts/

./export_results.sh


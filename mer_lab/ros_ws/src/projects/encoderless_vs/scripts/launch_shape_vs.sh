#!/bin/bash
mkdir ~/.ros/raw &&
cd ~/mer_lab/ros_ws

roslaunch encoderless_vs shape_servo_controller.launch &&

cd ~/mer_lab/ros_ws/src/projects/encoderless_vs/scripts/

# ./export_results.sh

./export_shape_results.sh


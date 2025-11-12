#!/bin/bash
mkdir ~/.ros/raw
mkdir ~/.ros/d_raw
cd ~/mer_lab/ros_ws

roslaunch encoderless_vision_dl franka_goal_feature.launch &&

mv ~/.ros/franka_features.yaml ~/mer_lab/ros_ws/src/projects/encoderless_vision_dl/config/

# Renaming the dir
mv ~/.ros/raw ~/.ros/goal_raw
mv ~/.ros/d_raw ~/.ros/goal_d_raw
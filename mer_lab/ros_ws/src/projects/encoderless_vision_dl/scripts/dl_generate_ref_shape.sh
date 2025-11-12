#!/bin/bash
mkdir ~/.ros/raw
# mkdir ~/.ros/d_raw
cd ~/mer_lab/ros_ws

# roslaunch encoderless_vs franka_dream_feature.launch i:=/home/merlab/DREAM/trained_models/panda_dream_resnet_h.pth b:=panda_link0 &&

roslaunch encoderless_vision_dl franka_dl_goal_feature.launch
# roslaunch encoderless_vs franka_dream_service.launch i:=/home/merlab/DREAM/trained_models/panda_dream_resnet_h.pth b:=panda_link0

mv ~/.ros/dl_features.yaml ~/mer_lab/ros_ws/src/projects/encoderless_vision_dl/config/

# Renaming the dir
mv ~/.ros/raw ~/.ros/goal_raw
# mv ~/.ros/d_raw ~/.ros/goal_d_raw
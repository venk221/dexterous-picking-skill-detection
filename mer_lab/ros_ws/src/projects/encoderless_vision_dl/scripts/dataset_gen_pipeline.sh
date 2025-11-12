#!/bin/sh

cd ~/mer_lab/ros_ws

roslaunch encoderless_vision_dl dream_depth_ds_gen.launch &&

cd ~/mer_lab/ros_ws/src/projects/encoderless_vision_dl/scripts/

# python3 merge_folder_rename.py
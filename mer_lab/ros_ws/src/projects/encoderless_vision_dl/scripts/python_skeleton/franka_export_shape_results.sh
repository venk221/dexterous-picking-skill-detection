#!/bin/bash

read exp_no

cd ~/Pictures/Dl_Exps/shape_vs/servoing/exps/ 

mkdir $exp_no

echo $dir_path

mv ~/.ros/dr.csv ~/Pictures/Dl_Exps/shape_vs/servoing/exps/$exp_no/

mv ~/.ros/ds.csv ~/Pictures/Dl_Exps/shape_vs/servoing/exps/$exp_no/

mv ~/.ros/err.csv ~/Pictures/Dl_Exps/shape_vs/servoing/exps/$exp_no/

mv ~/.ros/j1vel.csv ~/Pictures/Dl_Exps/shape_vs/servoing/exps/$exp_no/

mv ~/.ros/j2vel.csv ~/Pictures/Dl_Exps/shape_vs/servoing/exps/$exp_no/

mv ~/.ros/cp.csv ~/Pictures/Dl_Exps/shape_vs/servoing/exps/$exp_no/

mv ~/.ros/modelerror.csv ~/Pictures/Dl_Exps/shape_vs/servoing/exps/$exp_no/

mv ~/.ros/*.jpg ~/Pictures/Dl_Exps/shape_vs/servoing/exps/$exp_no/

mv ~/.ros/exp_vid.avi ~/Pictures/Dl_Exps/shape_vs/servoing/exps/$exp_no/

mkdir ~/Pictures/Dl_Exps/shape_vs/servoing/exps/$exp_no/imgs

mv ~/.ros/*.png ~/Pictures/Dl_Exps/shape_vs/servoing/exps/$exp_no/imgs/

mv ~/.ros/raw ~/Pictures/Dl_Exps/shape_vs/servoing/exps/$exp_no/
mv ~/.ros/goal_raw ~/Pictures/Dl_Exps/shape_vs/servoing/exps/$exp_no/
mv ~/.ros/goal_d_raw ~/Pictures/Dl_Exps/shape_vs/servoing/exps/$exp_no/
mv ~/.ros/d_raw ~/Pictures/Dl_Exps/shape_vs/servoing/exps/$exp_no/

cp ~/mer_lab/ros_ws/src/projects/encoderless_vision_dl/config/franka_shape_config.yaml  ~/Pictures/Dl_Exps/shape_vs/servoing/exps/$exp_no/
cp ~/mer_lab/ros_ws/src/projects/encoderless_vision_dl/config/franka_features.yaml  ~/Pictures/Dl_Exps/shape_vs/servoing/exps/$exp_no/

# cp ~/mer_lab/ros_ws/src/projects/encoderless_vs/config/franka_dream_config.yaml  ~/Pictures/Dl_Exps/shape_vs/servoing/exps/$exp_no/
# cp ~/mer_lab/ros_ws/src/projects/encoderless_vs/config/dream_features.yaml  ~/Pictures/Dl_Exps/shape_vs/servoing/exps/$exp_no/
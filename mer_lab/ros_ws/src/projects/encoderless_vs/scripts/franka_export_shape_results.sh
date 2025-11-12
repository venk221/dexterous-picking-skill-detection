#!/bin/bash

read exp_no

cd ~/Pictures/shape_vs/servoing/exps/ 

mkdir $exp_no

echo $dir_path

mv ~/.ros/dr.csv ~/Pictures/shape_vs/servoing/exps/$exp_no/

mv ~/.ros/ds.csv ~/Pictures/shape_vs/servoing/exps/$exp_no/

mv ~/.ros/err.csv ~/Pictures/shape_vs/servoing/exps/$exp_no/

mv ~/.ros/j1vel.csv ~/Pictures/shape_vs/servoing/exps/$exp_no/

mv ~/.ros/j2vel.csv ~/Pictures/shape_vs/servoing/exps/$exp_no/

mv ~/.ros/cp.csv ~/Pictures/shape_vs/servoing/exps/$exp_no/

mv ~/.ros/modelerror.csv ~/Pictures/shape_vs/servoing/exps/$exp_no/

mv ~/.ros/*.jpg ~/Pictures/shape_vs/servoing/exps/$exp_no/

mv ~/.ros/exp_vid.avi ~/Pictures/shape_vs/servoing/exps/$exp_no/

mkdir ~/Pictures/shape_vs/servoing/exps/$exp_no/imgs

mv ~/.ros/*.png ~/Pictures/shape_vs/servoing/exps/$exp_no/imgs/

mv ~/.ros/raw ~/Pictures/shape_vs/servoing/exps/$exp_no/
mv ~/.ros/goal_raw ~/Pictures/shape_vs/servoing/exps/$exp_no/
mv ~/.ros/goal_d_raw ~/Pictures/shape_vs/servoing/exps/$exp_no/
mv ~/.ros/d_raw ~/Pictures/shape_vs/servoing/exps/$exp_no/

cp ~/mer_lab/ros_ws/src/projects/encoderless_vs/config/franka_shape_config.yaml  ~/Pictures/shape_vs/servoing/exps/$exp_no/
cp ~/mer_lab/ros_ws/src/projects/encoderless_vs/config/franka_features.yaml  ~/Pictures/shape_vs/servoing/exps/$exp_no/

# cp ~/mer_lab/ros_ws/src/projects/encoderless_vs/config/franka_dream_config.yaml  ~/Pictures/shape_vs/servoing/exps/$exp_no/
# cp ~/mer_lab/ros_ws/src/projects/encoderless_vs/config/dream_features.yaml  ~/Pictures/shape_vs/servoing/exps/$exp_no/
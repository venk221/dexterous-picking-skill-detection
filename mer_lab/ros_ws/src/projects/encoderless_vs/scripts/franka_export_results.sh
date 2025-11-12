#!/bin/bash

read exp_no

cd ~/Pictures/adaptive_vs/servoing/exps/ 

mkdir $exp_no

echo $dir_path

mv ~/.ros/dr.csv ~/Pictures/adaptive_vs/servoing/exps/$exp_no/

mv ~/.ros/ds.csv ~/Pictures/adaptive_vs/servoing/exps/$exp_no/

mv ~/.ros/err.csv ~/Pictures/adaptive_vs/servoing/exps/$exp_no/

mv ~/.ros/j1vel.csv ~/Pictures/adaptive_vs/servoing/exps/$exp_no/
mv ~/.ros/j2vel.csv ~/Pictures/adaptive_vs/servoing/exps/$exp_no/

mv ~/.ros/modelerror.csv ~/Pictures/adaptive_vs/servoing/exps/$exp_no/

mv ~/.ros/ee_pos.csv ~/Pictures/adaptive_vs/servoing/exps/$exp_no/

mv ~/.ros/status.csv ~/Pictures/adaptive_vs/servoing/exps/$exp_no/

mv ~/.ros/*.jpg ~/Pictures/adaptive_vs/servoing/exps/$exp_no/

mv ~/.ros/exp_vid.avi ~/Pictures/adaptive_vs/servoing/exps/$exp_no/

mkdir ~/Pictures/adaptive_vs/servoing/exps/$exp_no/imgs

mv ~/.ros/*.png ~/Pictures/adaptive_vs/servoing/exps/$exp_no/imgs/

mv ~/.ros/raw ~/Pictures/adaptive_vs/servoing/exps/$exp_no/

cp ~/mer_lab/ros_ws/src/projects/encoderless_vs/config/franka_config.yaml  ~/Pictures/adaptive_vs/servoing/exps/$exp_no/
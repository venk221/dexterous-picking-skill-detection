#!/bin/bash

read exp_no

cd ~/Pictures/adaptive_baseline_gazebo/servoing/exps/ 

mkdir $exp_no

echo $dir_path

mv ~/.ros/dr.csv ~/Pictures/adaptive_baseline_gazebo/servoing/exps/$exp_no/

mv ~/.ros/ds.csv ~/Pictures/adaptive_baseline_gazebo/servoing/exps/$exp_no/

mv ~/.ros/err.csv ~/Pictures/adaptive_baseline_gazebo/servoing/exps/$exp_no/

mv ~/.ros/j1vel.csv ~/Pictures/adaptive_baseline_gazebo/servoing/exps/$exp_no/

mv ~/.ros/j2vel.csv ~/Pictures/adaptive_baseline_gazebo/servoing/exps/$exp_no/

mv ~/.ros/modelerror.csv ~/Pictures/adaptive_baseline_gazebo/servoing/exps/$exp_no/

mv ~/.ros/traj.jpg ~/Pictures/adaptive_baseline_gazebo/servoing/exps/$exp_no/

mv ~/.ros/exp_vid.avi ~/Pictures/adaptive_baseline_gazebo/servoing/exps/$exp_no/

cp ~/mer_lab/ros_ws/src/projects/encoderless_vs/config/config.yaml  ~/Pictures/adaptive_baseline_gazebo/servoing/exps/$exp_no/
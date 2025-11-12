#!/bin/bash

read exp_no

cd ~/Pictures/curve_estimation_adaptive_vs_gazebo/servoing/exps/ 

mkdir $exp_no

echo $dir_path

mv ~/.ros/dr.csv ~/Pictures/curve_estimation_adaptive_vs_gazebo/servoing/exps/$exp_no/

mv ~/.ros/ds.csv ~/Pictures/curve_estimation_adaptive_vs_gazebo/servoing/exps/$exp_no/

mv ~/.ros/err.csv ~/Pictures/curve_estimation_adaptive_vs_gazebo/servoing/exps/$exp_no/

mv ~/.ros/j1vel.csv ~/Pictures/curve_estimation_adaptive_vs_gazebo/servoing/exps/$exp_no/

mv ~/.ros/j2vel.csv ~/Pictures/curve_estimation_adaptive_vs_gazebo/servoing/exps/$exp_no/

mv ~/.ros/j3vel.csv ~/Pictures/curve_estimation_adaptive_vs_gazebo/servoing/exps/$exp_no/

mv ~/.ros/modelerror.csv ~/Pictures/curve_estimation_adaptive_vs_gazebo/servoing/exps/$exp_no/

mv ~/.ros/traj.jpg ~/Pictures/curve_estimation_adaptive_vs_gazebo/servoing/exps/$exp_no/

mv ~/.ros/exp_vid.avi ~/Pictures/curve_estimation_adaptive_vs_gazebo/servoing/exps/$exp_no/

mv ~/.ros/raw ~/Pictures/curve_estimation_adaptive_vs_gazebo/servoing/exps/$exp_no/

cp ~/.ros/published_goal_image.jpg ~/Pictures/curve_estimation_adaptive_vs_gazebo/servoing/exps/$exp_no/

cp ~/mer_lab/ros_ws/src/projects/encoderless_vs/config/3link_config.yaml  ~/Pictures/curve_estimation_adaptive_vs_gazebo/servoing/exps/$exp_no/

cp ~/mer_lab/ros_ws/src/projects/encoderless_vs/config/features.yaml  ~/Pictures/curve_estimation_adaptive_vs_gazebo/servoing/exps/$exp_no/

cp ~/mer_lab/ros_ws/src/projects/encoderless_vs/launch/shape_servo_controller.launch  ~/Pictures/curve_estimation_adaptive_vs_gazebo/servoing/exps/$exp_no/



cd ~/mer_lab/Matlab/encoderless_VS/

matlab -nodisplay -r "plotter_func($exp_no); exit"
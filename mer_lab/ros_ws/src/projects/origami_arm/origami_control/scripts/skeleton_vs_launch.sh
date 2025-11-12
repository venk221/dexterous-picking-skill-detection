#!/bin/bash

# This script runs the adaptive visual servoing algorithm on the origami arm
# and then runs a script to extract and process experiment data

clear &&

# Experiment number
read exp_no &&

# Initialize robot
./init.sh

# Run experiment
roslaunch origami_control skeleton_vs.launch &&

# Extract logs
cd ~/mer_lab/ros_ws/src/projects/origami_arm/origami_log/scripts/ &&
./skeleton_bag_to_csv.sh &&

# Plot results
cd ~/mer_lab/ros_ws/src/projects/origami_arm/origami_vis/scripts/ &&
# ./plot_skeleton_results.py &&
matlab -nodisplay -r "plot_skeleton_results; exit" &&

# Organize data
# Create experiment folder
cd ~/Pictures/origami_skeleton_vs/test/ &&
mkdir $exp_no &&

# Move all data to exps folder
mv ~/.ros/error.csv ~/Pictures/origami_skeleton_vs/test/$exp_no &&
mv ~/.ros/model_error.csv ~/Pictures/origami_skeleton_vs/test/$exp_no &&
mv ~/.ros/velocity.csv ~/Pictures/origami_skeleton_vs/test/$exp_no &&
mv ~/.ros/features.csv ~/Pictures/origami_skeleton_vs/test/$exp_no &&
mv ~/.ros/aruco.csv ~/Pictures/origami_skeleton_vs/test/$exp_no &&

# Move all plots to exps folder
mv ~/.ros/feature_error.png ~/Pictures/origami_skeleton_vs/test/$exp_no &&
mv ~/.ros/error_norm.png ~/Pictures/origami_skeleton_vs/test/$exp_no &&
mv ~/.ros/velocity.png ~/Pictures/origami_skeleton_vs/test/$exp_no &&
mv ~/.ros/control_velocity.png ~/Pictures/origami_skeleton_vs/test/$exp_no &&
mv ~/.ros/model_error.png ~/Pictures/origami_skeleton_vs/test/$exp_no &&

# copy experiment profile
cp ~/mer_lab/ros_ws/src/projects/origami_arm/origami_control/config/config.yaml ~/Pictures/origami_skeleton_vs/test/$exp_no &&
cp ~/mer_lab/ros_ws/src/projects/origami_arm/origami_control/config/origami_features.yaml ~/Pictures/origami_skeleton_vs/test/$exp_no &&
cp ~/.ros/ref_img.jpg ~/Pictures/origami_skeleton_vs/test/$exp_no &&
cp ~/.ros/ref_raw.jpg ~/Pictures/origami_skeleton_vs/test/$exp_no &&

# move bag file
export bag_path=$(find -L ~/.ros/ -name "*.bag") &&
mv $bag_path ~/Pictures/origami_skeleton_vs/test/$exp_no &&

echo "Processing completed, exiting"
exit 1
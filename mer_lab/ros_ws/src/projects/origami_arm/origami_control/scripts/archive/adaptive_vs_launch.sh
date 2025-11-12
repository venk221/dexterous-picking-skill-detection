#!/bin/bash

# This script runs the adaptive visual servoing algorithm on the origami arm
# and then runs a script to extract and process experiment data

# Run experiment
roslaunch origami_control adaptive_vs.launch &&

# Go to log pkg scripts folder
cd ~/mer_lab/ros_ws/src/projects/origami_arm/origami_log/scripts &&

# Run script to process exp video, extract data from bag and plot
./bag_to_csv.sh 
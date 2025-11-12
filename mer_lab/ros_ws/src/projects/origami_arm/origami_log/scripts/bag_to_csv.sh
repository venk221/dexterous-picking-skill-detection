#!/bin/sh

read exp_no &&

# Find bag name in .ROS/
export bag_path=$(find -L ~/.ros/ -name "*.bag") &&

# # Launch bag and image view node to make video
# roslaunch origami_vis extract_imgs.launch ARG_NAME:=$bag_path &&

# # Save frames to temp folder
# mkdir ~/.ros/exp_imgs &&
# mv ~/.ros/frame*.jpg ~/.ros/exp_imgs/ &&

# # Stitch frames
# ffmpeg -r 1/15 -i frame%04d.jpg -c:v libx264 -profile:v high -crf 15 -pix_fmt yuv420p ~/.ros/output.mp4 &&

# export topics to csv
rostopic echo -b $bag_path -p /error >~/.ros/error.csv &&
rostopic echo -b $bag_path -p /model_error >~/.ros/model_error.csv &&
rostopic echo -b $bag_path -p /velocity >~/.ros/velocity.csv &&
rostopic echo -b $bag_path -p /ee_pose >~/.ros/ee_pose.csv &&

# run script to visualize data
cd ~/mer_lab/ros_ws/src/projects/origami_arm/origami_vis/scripts/ &&
./plot_results.py &&

# Create experiment folder
cd ~/Pictures/origami_adaptive_vs/test/ &&
mkdir $exp_no &&

# Move all data to exps folder
mv ~/.ros/error.csv ~/Pictures/origami_adaptive_vs/test/$exp_no &&
mv ~/.ros/model_error.csv ~/Pictures/origami_adaptive_vs/test/$exp_no &&
mv ~/.ros/velocity.csv ~/Pictures/origami_adaptive_vs/test/$exp_no &&
mv ~/.ros/ee_pose.csv ~/Pictures/origami_adaptive_vs/test/$exp_no &&

# mv ~/.ros/exp_imgs ~/Pictures/origami_adaptive_vs/test/$exp_no &&
# mv ~/.ros/output.mp4 ~/Pictures/origami_adaptive_vs/test/$exp_no &&

# copy experiment profile
cp ~/mer_lab/ros_ws/src/projects/origami_arm/origami_control/config/config.yaml ~/Pictures/origami_adaptive_vs/test/$exp_no &&

# move bag file
mv $bag_path ~/Pictures/origami_adaptive_vs/test/$exp_no &&

echo "Processing completed, exiting"
exit 1
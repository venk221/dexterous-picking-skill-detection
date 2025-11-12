#!/bin/sh

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
rostopic echo -b $bag_path -p /features >~/.ros/features.csv &&
rostopic echo -b $bag_path -p /aruco_pose >~/.ros/aruco.csv
#!/bin/bash

# This script extracts image and video data from rosbag after the experiment is complete
echo -n "Enter exp number: "
read exp_no &&

folderPath=/home/abhinav/Pictures/origami_skeleton_vs/test/clothoids_3-3/$exp_no &&

cd $folderPath &&
bagName=$(find -type f -name "*.bag") &&

bagPath=$folderPath/$bagName &&

topicName=curve_image &&



# Extract images to home folder
roslaunch origami_vis extract_imgs.launch bag_path:=$bagPath topic_name:=$topicName  &&

# Move imgs if needed
cd $folderPath
mkdir exp_imgs
mv ~/.ros/frame*.jpg exp_imgs/

# Stitch video
cd exp_imgs
ffmpeg -framerate 10 -i frame%04d.jpg -c:v libx264 -profile:v high -crf 1 -pix_fmt yuv420p exp_video.mp4

# Store things appropriately
mv exp_video.mp4 $folderPath

echo "Video extraction completed, exiting"
exit 1
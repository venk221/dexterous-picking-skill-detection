#!/bin/bash

source ~/mer_lab/ros_ws/devel/setup.bash

# Launch ref generation
roslaunch origami_control ref_gen.launch  &&
# roslaunch origami_control reference_clothoid.launch
# Move yaml to control package
mv ~/.ros/origami_features.yaml ~/mer_lab/ros_ws/src/projects/origami_arm/origami_control/config/
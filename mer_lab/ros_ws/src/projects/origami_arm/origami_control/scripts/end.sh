#!/bin/bash

# This script publishes true to the end_flag topic to end experiment. 
# The publisher latches for 3 seconds and automatically quits after that.

rostopic pub -1 /origami_vs/end_flag std_msgs/Bool "data: true"
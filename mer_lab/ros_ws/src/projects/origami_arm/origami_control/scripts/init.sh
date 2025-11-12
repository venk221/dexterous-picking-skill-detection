#!/bin/bash

# This script communicates with the low level controllers
# For board and firmware v3.0

rostopic pub /origami_vs/OMMD_control_mode std_msgs/Int32 "data: 3"
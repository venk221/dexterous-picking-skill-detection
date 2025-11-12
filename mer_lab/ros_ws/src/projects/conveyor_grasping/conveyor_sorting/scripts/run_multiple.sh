#!/bin/bash

# Path constants
OUTPUT_PATH="/tmp"
SCORE_FILENAME="scores.csv"
DATA_FILE="${OUTPUT_PATH}/output.csv"
PILE_SEED_PARAM="/pile_simulation/seed"
PILE_RESET_TOPIC="/pile_simulator/reset"
SWEEP_COUNT_PARAM="/conveyor_sorting/sweeper/max_iter"

# Arguments given
PILE_SEED=$1
TRIAL_COUNT=$2
SWEEP_COUNT=$3
COMBINED_OUTPUT_PATH=$4

if [ $# -ne 4 ]
then
    echo "Invalid argument count! Got $# but expected 4."
    exit
fi

SCORE_FILEPATH=$COMBINED_OUTPUT_PATH/$SCORE_FILENAME

# Upload params
rosparam set $PILE_SEED_PARAM $PILE_SEED
rosparam set $SWEEP_COUNT_PARAM $SWEEP_COUNT

# Create the output directory if needed
mkdir $COMBINED_OUTPUT_PATH

# Overwrite the score file with nothing to reset it or create it if needed
cat /dev/null > $SCORE_FILEPATH

# Start loop
for (( i=1; i<=$TRIAL_COUNT; i++ ))
do
    echo "======Running trial ${i}======"
    # Reset the environment
    rosservice call $PILE_RESET_TOPIC "{}"

    # Run the sweeping node
    rosrun conveyor_sorting ot_sweeper_node

    # Create the ith output directory
    mkdir $COMBINED_OUTPUT_PATH/$i
    # Copy the images over
    for (( j=0; j<=$SWEEP_COUNT; j++ ))
    do
	# Copy images over
	cp $OUTPUT_PATH/${j}rgb.png $COMBINED_OUTPUT_PATH/$i/${j}rgb.png
	cp $OUTPUT_PATH/${j}top.png $COMBINED_OUTPUT_PATH/$i/${j}top.png
    done

    # Copy the output csv file
    cp $DATA_FILE $COMBINED_OUTPUT_PATH/$i/stats.csv

    # Append newly-obtained data into the score file
    tail -n +2 $DATA_FILE | awk -F "," 'BEGIN {ORS=" "} {print $1 " " $8}' >> $SCORE_FILEPATH
    echo >> $SCORE_FILEPATH
done

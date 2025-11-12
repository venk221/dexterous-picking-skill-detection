ERR_POS=0.0
ERR_VEL=0.0

if [ $# -lt 3 ]; then
    echo "Please provide 3 arguments: grasp_type[anticipation|normal], error[none|normal|high], model_name"
    exit 1
fi

if [ "$2" = "normal" ]; then
    ERR_POS=0.05
    ERR_VEL=0.01
elif [ "$2" = "high" ]; then
    ERR_POS=0.1
    ERR_VEL=0.025
fi

echo "Running $1 with errors $ERR_POS and $ERR_VEL on $3"
# Actually run the command
if [ "$1" = "anticipation" ]; then
    rosrun conveyor_picking anticipation_node 3 0.1 0.1 0 0 $ERR_POS 0 $ERR_VEL 0.4 0.5 $3
else
    rosrun conveyor_picking main_picking_node 3 0.1 0.1 0 0 $ERR_POS 0 $ERR_VEL -0.1 $3
fi

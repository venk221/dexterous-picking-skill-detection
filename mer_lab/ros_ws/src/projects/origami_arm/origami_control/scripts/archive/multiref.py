#!/usr/bin/env python3

import roslib
import sys
import rospy
import numpy as np
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray, Int32

ref_list = []
no_of_features = 0
ref_no = 1
no_of_refs = 0
end_flag_pub = None
user_input = None
reference = None


def userInputCallback(msg):
    global ref_no, ref_pub, user_input, pub_list

    user_input = msg.data
    
    if ref_no >= no_of_refs:
        end_flag_pub.publish(True)  
    
    # publishing first goal
    elif not user_input:
        reference.data = ref_list[0]

    # publishing next goals
    else:
        reference.data = ref_list[ref_no]
        ref_no = ref_no + 1

def main(args):
    global ref_list, no_of_features, no_of_refs, end_flag_pub, ref_list, reference
    # Initialize node
    rospy.init_node('muliple_refs')

    # Initialize subscribers
    user_input_sub = rospy.Subscriber("vsbot/user_input", Bool, userInputCallback, queue_size=1)    

    # Initialize publishers
    ref_pub = rospy.Publisher("vsbot/cur_goal", Float64MultiArray, queue_size=1)
    end_flag_pub = rospy.Publisher("vsbot/end_flag", Bool, queue_size=1)
    
    # Read references from YAML
    ref_list = rospy.get_param("shape_controller/goal_features")
    no_of_features = rospy.get_param("vsbot/shape_control/no_of_features")

    # Shape reference list
    ref_list = np.array(ref_list)
    no_of_refs = len(ref_list)/no_of_features
    ref_list = ref_list.reshape(int(no_of_refs), int(no_of_features))

    # Publish goals
    reference = Float64MultiArray()

    # Publish grasp action


    while not rospy.is_shutdown():
        ref_pub.publish(reference)   
        

    rospy.spin()

if __name__ == "__main__":
    main(sys.argv)
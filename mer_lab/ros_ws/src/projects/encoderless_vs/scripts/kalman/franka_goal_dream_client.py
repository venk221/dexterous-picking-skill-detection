#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from encoderless_vs.srv import dream_goal_img, dream_goal_imgResponse
from scipy.interpolate import splev
from cv_bridge import CvBridge, CvBridgeError


def main():
    # initialize the node
    rospy.init_node('franka_dream_vis_node') 
    
    rospy.sleep(20)
    # to convert cv to ros
    bridge = CvBridge()
    # wait for control points service to be up
    rospy.wait_for_service('dream_img_service_response')

    # create a handle to call the service
    dream_img_service_response = rospy.ServiceProxy('dream_img_service_response', dream_goal_img)    
    # service request is sent to receive tck
    srv_resp = dream_img_service_response(1)
    
    ## Ouput a YAML file with these parameters
    yaml_file = open("dream_features.yaml","w")
    s = ""
    s += "shape_controller:\n"
    s += "  goal_features: [" + (','.join(map(str,srv_resp.pix.data))) +"]\n"
    
    yaml_file.write(s)
    yaml_file.close()
        
    cv_img = bridge.imgmsg_to_cv2(srv_resp.img, "rgb8")  

    # save the goal curve image in .ros file
    cv2.imwrite("dream_published_goal_image.jpg", cv_img)

    rospy.signal_shutdown("Wrote features to file")
         

if __name__ == '__main__':
    main()

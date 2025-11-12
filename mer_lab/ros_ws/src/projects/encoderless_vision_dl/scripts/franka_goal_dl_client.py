#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from encoderless_vision_dl.srv import dl_img, dl_imgResponse
from scipy.interpolate import splev
from cv_bridge import CvBridge, CvBridgeError


def main():
    # initialize the node
    rospy.init_node('franka_dl_goal_vis_node') 
    
    rospy.sleep(25)
    # to convert cv to ros
    bridge = CvBridge()
    # wait for control points service to be up
    rospy.wait_for_service('franka_kp_dl_service')

    # create a handle to call the service
    franka_kp_dl_service = rospy.ServiceProxy('franka_kp_dl_service', dl_img)    
    # service request is sent to receive tck
    srv_resp = franka_kp_dl_service(1)

    print("did service get called")

    # rospy.sleep(20)

    print("goal image type", type(srv_resp.img))
    print("goal kp", srv_resp.kp.data)    
    
    ## Ouput a YAML file with these parameters
    yaml_file = open("dl_features.yaml","w")
    s = ""
    s += "dl_controller:\n"
    s += "  goal_features: [" + (','.join(map(str,srv_resp.kp.data))) +"]\n"
    
    yaml_file.write(s)
    yaml_file.close()
        
    cv_img = bridge.imgmsg_to_cv2(srv_resp.img, "bgr8")  

    i = 0
    while i<(len(srv_resp.kp.data)):
        x = int(srv_resp.kp.data[i])
        y = int(srv_resp.kp.data[i+1])
        cv2.circle(cv_img,(x, y),5,(0,0,255),-1)
        i += 2
    # for i in range(len(srv_resp.kp.data)):
    #     x = int(srv_resp.kp.data[i])
    #     y = int(srv_resp.kp.data[i+1])
    #     cv2.circle(cv_img,(x, y),2,(0,0,255),-1)

    # save the goal curve image in .ros file
    cv2.imwrite("dl_published_goal_image.jpg", cv_img)    

    rospy.signal_shutdown("Wrote features to file")
         

if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from std_msgs.msg import Float64MultiArray, Bool
from sensor_msgs.msg import Image
# from encoderless_vs.srv import dream_img, dream_imgResponse
# from encoderless_vs.srv import franka_control_points, franka_control_pointsResponse
from encoderless_vs.srv import dream_kp, dream_kpResponse
from scipy.interpolate import splev
from cv_bridge import CvBridge, CvBridgeError

cp_flag = None
bridge = CvBridge()

def flag_cb(msg):
  global cp_flag
  cp_flag = msg.data

def main():
    # initialize the node
    rospy.init_node('franka_dream_node') 
    
    rospy.sleep(25)
    # declare the subscriber for the flag 
    # flag_sub = rospy.Subscriber("/franka/control_flag", Bool, flag_cb, queue_size=1)

    # # wait for control points service to be up
    # rospy.wait_for_service('franka_control_service')

    # # create a handle to call the service
    # franka_control_service = rospy.ServiceProxy('franka_control_service', franka_control_points)
    
    # # assigning a rate at which the control points service will be called
    # r = rospy.get_param('vsbot/estimation/rate')
    # rate = rospy.Rate(r)

    # while not rospy.is_shutdown():
    #   if cp_flag == True:
    #     srv_resp = franka_control_service(1)
    #     control_pts = srv_resp.cp.data
        

    #     print("Control Points", control_pts)
           
    #   rate.sleep()    
    ## The following set of code is for only key points
    # bridge = CvBridge()
    # # wait for control points service to be up
    rospy.wait_for_service('dream_kp_service_response')

    # # create a handle to call the service
    # # dream_img_service_response = rospy.ServiceProxy('dream_img_service_response', dream_goal_img)
    dream_kp_service_response = rospy.ServiceProxy('dream_kp_service_response', dream_kp)     

    # # service request is sent to receive tck
    
    
    # # ## Ouput a YAML file with these parameters
    # # yaml_file = open("dream_features_kp.yaml","w")
    # # s = ""
    # # s += "shape_controller:\n"
    # # s += "  goal_features: [" + (','.join(map(str,srv_resp.pix.data))) +"]\n"
    
    # # yaml_file.write(s)
    # # yaml_file.close()
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
      # if cp_flag == True:
        srv_resp = dream_kp_service_response(1)
        key_points = srv_resp.pix.data
        print("Joints Points", key_points)           
        rate.sleep()    
    
    cv_img = bridge.imgmsg_to_cv2(srv_resp.img, "rgb8")  

    # save the goal curve image in .ros file
    # cv2.imwrite("dream_published_goal_image.jpg", cv_img)

    # rospy.signal_shutdown("Wrote features to file")
         

if __name__ == '__main__':
    main()

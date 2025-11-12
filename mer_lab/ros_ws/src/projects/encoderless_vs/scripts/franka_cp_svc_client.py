#!/usr/bin/env python3

import rospy
import numpy as np
import sys
import time
import cv2
from std_msgs.msg import Bool
import glob
import os
from sensor_msgs.msg import Image
import csv
from cv_bridge import CvBridge, CvBridgeError
from encoderless_vs.srv import franka_control_points, franka_control_pointsResponse

# global flag to start the control point service

cp_flag = None
bridge = CvBridge()

def flag_cb(msg):
  global cp_flag
  cp_flag = msg.data

def main(args):
    global bridge
    # initialize the node
    rospy.init_node('cp_service_request_node')
    rospy.sleep(30)
    # declare the subscriber for the flag 
    flag_sub = rospy.Subscriber("/franka/control_flag", Bool, flag_cb, queue_size=1)

    # wait for control points service to be up
    rospy.wait_for_service('franka_control_service')

    # create a handle to call the service
    franka_control_service = rospy.ServiceProxy('franka_control_service', franka_control_points)
    
    # assigning a rate at which the control points service will be called
    r = rospy.get_param('vsbot/estimation/rate')
    rate = rospy.Rate(r)

    while not rospy.is_shutdown():
      if cp_flag == True:
        srv_resp = franka_control_service(1)
        control_pts = srv_resp.cp.data
        

        print("Control Points", control_pts)
           
      rate.sleep()    
    
         

if __name__ == '__main__':
    main(sys.argv)
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
from encoderless_vs.srv import three_link_test, three_link_testResponse

# global flag to start the control point service

cp_flag = None
bridge = CvBridge()
# cv_img = None
l=0

def flag_cb(msg):
  global cp_flag
  cp_flag = msg.data

def main(args):
    global l
    # initialize the node
    rospy.init_node('cp_service_request_node')

    # declare the subscriber for the flag 
    flag_sub = rospy.Subscriber("/vsbot/control_flag", Bool, flag_cb, queue_size=1)
    # img_sub = rospy.Subscriber("/vsbot/camera1/image_raw", Image, vis_callback, queue_size=1)

    # wait for control points service to be up
    rospy.wait_for_service('control_points_output')

    # create a handle to call the service
    control_points_output = rospy.ServiceProxy('control_points_output', three_link_test)
    
    # assigning a rate at which the control points service will be called
    r = rospy.get_param('3linkbot/vs_baseline/pub_rate')
    rate = rospy.Rate(r)

    # l = 0
    while not rospy.is_shutdown():
      if cp_flag == True:   
        # tstart = time.time()
        srv_resp = control_points_output(1)
        cp = srv_resp.cp.data
        img = srv_resp.rgb
        cv_img = bridge.imgmsg_to_cv2(img, "bgr8")
        fname = str(l) + ".jpg"
        cv2.imwrite(fname, cv_img)   
      l+=1
      
         

if __name__ == '__main__':
    main(sys.argv)
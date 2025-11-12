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
from scipy.interpolate import splprep, splev
from cv_bridge import CvBridge, CvBridgeError
from encoderless_vs.srv import franka_cp_vis, franka_cp_visResponse

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
    rospy.sleep(5)
    
    # declare the subscriber for the flag 
    flag_sub = rospy.Subscriber("/franka/control_flag", Bool, flag_cb, queue_size=1)

    # wait for control points service to be up
    rospy.wait_for_service('franka_control_service')

    # create a handle to call the service
    franka_control_service = rospy.ServiceProxy('franka_control_service', franka_cp_vis)

    # f = open('/home/janch-ros/Documents/vs_recorder/time_record.csv', 'w')

    # printing the coeffs to csv
    # f = open('coeff_record.csv', 'w')
    # writer = csv.writer(f)
    # writer.writerow(['Coeff1', 'Coeff2', 'Coeff3', 'Coeff4', 'Coeff5', 'Coeff6', 'lim[0]', 'lim[-1]']) #, 'Service Request Time','Binary Img Rqst Start', 
    # 'Binary Img Rqst Stop', 'Skeleton Start', 'Skeleton End', 'Ordering Start', 
    #         'Ordering End', 'Spline Start', 'Spline End', 'Service Response Time'])

    
    # assigning a rate at which the control points service will be called
    r = rospy.get_param('vsbot/vs_baseline/pub_rate')
    rate = rospy.Rate(r)

    l = 0
    while not rospy.is_shutdown():
      if cp_flag == True:
        srv_resp = franka_control_service(1)
        ros_rgb_img = srv_resp.rgb

        cv_rgb_img = bridge.imgmsg_to_cv2(ros_rgb_img, "bgr8")

        # cv2.imwrite("franka_robot"+str(l)+".jpg", cv_rgb_img)

        control_pts = srv_resp.cp.data
        print("Control Points", control_pts)
        t = np.array(srv_resp.t)
        cx = np.array(srv_resp.cx)
        cy = np.array(srv_resp.cy)
        k = srv_resp.k

        print(t, cx, cy, k)

        tck = [t, [cx, cy], k]    

        # generating new parameters to be evaluated
        params = np.linspace(0, 1, 200)

        # new points after evaluation
        newPts =  np.array(splev(params, tck))
        x = newPts[0]
        y = newPts[1]


        points = np.c_[x,y]

        ct_pts = np.c_[cx, cy]

        # Create a blank image with black background
        # cv_img = np.zeros((720, 1280, 3), dtype = "uint8") 

        for i in range(len(points)):
          x = np.int(points[i][0])
          y = np.int(points[i][1])
          cv2.circle(cv_rgb_img,(x, y),3,(0,0,255),-1)

        for i in range(len(ct_pts)):
          x = np.int(ct_pts[i][0])
          y = np.int(ct_pts[i][1])
          cv2.circle(cv_rgb_img,(x, y),5,(0,255,255),-1)


        # save the goal curve image in .ros file
        cv2.imwrite("franka_published_goal_image"+str(l)+".jpg", cv_rgb_img)        
        
        # fname = str(l)+'.jpg'
        # cv2.imwrite(fname, cv_img)
        l +=1        
      rate.sleep()    



if __name__ == '__main__':
    main(sys.argv)
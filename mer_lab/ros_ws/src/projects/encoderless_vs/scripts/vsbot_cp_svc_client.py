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
from encoderless_vs.srv import control_points, control_pointsResponse

# global flag to start the control point service

cp_flag = None
bridge = CvBridge()
# cv_img = None

eval_pts = []
control_pts = []
knot_pt = []
l=0

def flag_cb(msg):
  global cp_flag
  cp_flag = msg.data

def vis_callback(img):
    global l, cv_img
    # if control_pts:
    cv_img = bridge.imgmsg_to_cv2(img, "bgr8")

    ep = list(eval_pts)
    cp = list(control_pts)
    kp = list(knot_pt)
    
  
    i = 0
    while i <=(len(ep)-1):
      x = int(ep[i])
      y = int(ep[i+1])
      print(x, y)
      cv2.circle(cv_img,(x, y),3,(0,0,255),-1)
      i = i+2
      
    j = 0
    while j <=(len(cp)-1):
      x = int(cp[j])
      y = int(cp[j+1])
      cv2.circle(cv_img,(x, y),3,(0,255,255),-1)
      j = j+2

    k = 0
    while k <=(len(kp)-1):
      x = int(kp[k])
      y = int(kp[k+1])
      cv2.circle(cv_img,(x, y),3,(255,0,255),-1)
      k = k+2

    # if control_pts and cv_img is not None:
    fname = str(l) + ".png"
    cv2.imwrite(fname, cv_img)
    l= l+1    


def main(args):
    global eval_pts, control_pts, knot_pt
    # initialize the node
    rospy.init_node('cp_service_request_node')

    # declare the subscriber for the flag 
    flag_sub = rospy.Subscriber("/vsbot/control_flag", Bool, flag_cb, queue_size=1)
    # img_sub = rospy.Subscriber("/vsbot/camera1/image_raw", Image, vis_callback, queue_size=1)

    # wait for control points service to be up
    rospy.wait_for_service('control_points_output')

    # create a handle to call the service
    control_points_output = rospy.ServiceProxy('control_points_output', control_points)

    # f = open('/home/janch-ros/Documents/vs_recorder/time_record.csv', 'w')

    # printing the coeffs to csv
    # f = open('coeff_record.csv', 'w')
    # writer = csv.writer(f)
    # writer.writerow(['Coeff1', 'Coeff2', 'Coeff3']) #, 'Service Request Time','Binary Img Rqst Start', 
    # 'Binary Img Rqst Stop', 'Skeleton Start', 'Skeleton End', 'Ordering Start', 
    #         'Ordering End', 'Spline Start', 'Spline End', 'Service Response Time'])

    
    # assigning a rate at which the control points service will be called
    r = rospy.get_param('vsbot/vs_baseline/pub_rate')
    rate = rospy.Rate(r)

    l = 0
    while not rospy.is_shutdown():
      if cp_flag == True:   
        # tstart = time.time()
        srv_resp = control_points_output(1)
        control_pts = srv_resp.cp.data
        print("Control Points", control_pts)

        # eval_pts = srv_resp.ep.data
        # knot_pt = srv_resp.kp.data

        # ep = list(eval_pts)
        # cp = list(control_pts)
        # kp = list(knot_pt)

        # cv_img = np.zeros((300, 300, 3), dtype = "uint8")

        # i = 0
        # while i <=(len(ep)-1):
        #   x = int(ep[i])
        #   y = int(ep[i+1])
        #   print(x, y)
        #   cv2.circle(cv_img,(x, y),3,(0,0,255),-1)
        #   i = i+2

        # j = 0
        # while j <=(len(cp)-1):
        #   x = int(cp[j])
        #   y = int(cp[j+1])
        #   cv2.circle(cv_img,(x, y),3,(0,255,255),-1)
        #   j = j+2

        # k = 0
        # while k <=(len(kp)-1):
        #   x = int(kp[k])
        #   y = int(kp[k+1])
        #   cv2.circle(cv_img,(x, y),3,(255,0,255),-1)
        #   k = k+2

        # fname = str(l) + ".png"
        # cv2.imwrite(fname, cv_img)
        # l= l+1
# 
        # 
# 
        # coeff1 = srv_resp.coeffs.data[0]
        # coeff2 = srv_resp.coeffs.data[1]
        # coeff3 = srv_resp.coeffs.data[2]
# 
        
        # skel_tstart = srv_resp.skel_tstart
        # skel_tstop = srv_resp.skel_tstop
        # nn_tstart = srv_resp.nn_tstart
        # nn_tstop = srv_resp.nn_tstop
        # spline_tstart = srv_resp.spl_tstart
        # spline_tstop = srv_resp.spl_tstop
        # bin_rqst_tstart = srv_resp.bin_tstart
        # bin_rqst_tstop = srv_resp.bin_tstop

        # tstop = time.time()
        # row_data = [control_pts, coeffs, tstart , bin_rqst_tstart, bin_rqst_tstop, skel_tstart, 
                    # skel_tstop, nn_tstart, nn_tstop, spline_tstart, spline_tstop, tstop]
      #   row_data = [coeff1, coeff2, coeff3]
      #   writer.writerow(row_data)
      #   # cp_flag = False
      rate.sleep()

      
    
    # f.close()   
         

if __name__ == '__main__':
    main(sys.argv)
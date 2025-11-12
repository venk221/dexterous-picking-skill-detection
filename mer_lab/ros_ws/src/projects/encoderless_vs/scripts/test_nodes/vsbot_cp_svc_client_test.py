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
from encoderless_vs.srv import cp_test, cp_testResponse

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
      # print(x, y)
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
    control_points_output = rospy.ServiceProxy('control_points_output', cp_test)

    # f = open('/home/janch-ros/Documents/vs_recorder/time_record.csv', 'w')

    # # printing the coeffs to csv
    f1 = open('coeff_record.csv', 'w')
    f2 = open('cp_record.csv', 'w')
    f3 = open('bf_record.csv', 'w')
    writer1 = csv.writer(f1)
    # writer1.writerow(['Coeff1', 'Coeff2', 'Coeff3', 'Coeff4'])
    
    writer2 = csv.writer(f2)
    writer3 = csv.writer(f3)
    # writer2.writerow(['bf1', 'Coeff2', 'Coeff3', 'Coeff4']) #'Coeff5', 'Coeff6', 'lim[0]', 'lim[-1]']) #, 'Service Request Time','Binary Img Rqst Start', 
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
        # control_pts = srv_resp.cp.data
        # print("Control Points", control_pts)

        # eval_pts_1 = srv_resp.ep1.data
        # eval_pts_2 = srv_resp.ep2.data
        coeff = srv_resp.coeffs.data
        # bf = srv_resp.bf.data

        # print("coeffs", coeff)

        # row_data = []
        # for i in coeff:
        #   row_data.append(i)
        
        # # # ep_limits = [eval_pts[0], eval_pts[-2]]
        # # # row_data.extend(ep_limits)  
        # writer1.writerow(row_data)

        # row_data = []
        # for i in control_pts:
        #   row_data.append(i)

        # writer2.writerow(row_data)

        # row_data = []
        # for i in bf:
        #   row_data.append(i)

        # writer3.writerow(row_data)

        # ep1 = list(eval_pts_1)
        # ep2 = list(eval_pts_2)

        


        # evaluation of line segment 1

        t = np.linspace(0, 1, 20)

        x = coeff[0]*t**2 + coeff[1]*t + coeff[2]
        y = coeff[3]*t**2 + coeff[4]*t + coeff[5]

        print(x)
        print(y)

        print(type(x), type(y))

        cv_img = np.zeros((300, 300, 3), dtype = "uint8")

        for i in range(len(x)):
          px = np.int(x[i])
          py = np.int(y[i])
          cv2.circle(cv_img,(px, py),5,(255,0,0),-1)
        
        # cp = list(control_pts)
        # kp = list(knot_pt)

#         cv_img = np.zeros((300, 300, 3), dtype = "uint8")
#         # cv_img_second = np.zeros((300, 300, 3), dtype = "uint8")

       
#         # a = coeff[0]
#         # b = coeff[1]
#         # # c = coeff[2]
#         # x = np.linspace(eval_pts_1[0],eval_pts_1[-2],10)
#         # # y = a*x**2 + b*x + c
#         # y = a*x + b
#         points = np.c_[x, y]

#         for j in range(len(points)):
#           x = np.int(points[j][0])
#           y = np.int(points[j][1]) 
#           cv2.circle(cv_img,(x, y),3,(255,255,255),-1)

#         # evaluation of line segment 1

#         t = np.linspace(0.5, 1, 10)

#         x2 = a2x*t**2 + b2x*t + c2x
#         y2 = a2y*t**2 + b2y*t + c2y

#         print(x2, y2)
#         k = 0
#         while k <=(len(ep2)-1):
#           x = int(ep2[k])
#           y = int(ep2[k+1])
#           cv2.circle(cv_img,(x, y),6,(0,0,255),-1)
#           k = k+2

#         # a = coeff[2]
#         # b = coeff[3]
#         # # c = coeff[5]
#         # x = np.linspace(eval_pts_2[0],eval_pts_2[-2],10)
#         # # y = a*x**2 + b*x + c
#         # y = a*x + b
#         points = np.c_[x2, y2]

#         for m in range(len(points)):
#           x = np.int(points[m][0])
#           y = np.int(points[m][1]) 
#           cv2.circle(cv_img,(x, y),3,(0,255,255),-1)
        

        

#         # # j = 0
#         # # while j <=(len(cp)-1):
#         # #   x = int(cp[j])
#         # #   y = int(cp[j+1])
#         # #   cv2.circle(cv_img,(x, y),3,(0,255,255),-1)
#         # #   j = j+2

#         # # k = 0
#         # # while k <=(len(kp)-1):
#         # #   x = int(kp[k])
#         # #   y = int(kp[k+1])
#         # #   cv2.circle(cv_img,(x, y),3,(255,0,255),-1)
#         # #   k = k+2

        fname1 = "first_seg"+ str(l) + ".png"
        cv2.imwrite(fname1, cv_img)
        cv2.imwrite("coeff_image.jpg", cv_img)

        # fname2 = "second_seg"+ str(l) + ".png"
        # cv2.imwrite(fname2, cv_img_second)
        l= l+1
# # 
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
    f1.close()
    f2.close()
    f3.close()

      
    
      
         

if __name__ == '__main__':
    main(sys.argv)
#!/usr/bin/env python3

import sys
import rospy
import cv2
import csv
import numpy as np
import math
from skimage.morphology import skeletonize
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from scipy.interpolate import splprep, splev, BSpline
import time
# from sympy import lambdify, bspline_basis_set
# from sympy.abc import u
from cv_bridge import CvBridge, CvBridgeError
from encoderless_vs.srv import cp_test, cp_testResponse, bin_img, bin_imgResponse

# Declaring cvbridge for ROS
bridge = CvBridge()
binary_image_output = None

# the following service callback definies the service that 
# takes in binary image input and gives control points output  
def control_points_service(data):  
  global bridge
  
  resp_bin = binary_image_output(1)
  binary_resp = resp_bin.bin
   
  # ros binary image as the service input
  # ros binary image as the service input
  ros_binary_image = binary_resp
  cv_binary = bridge.imgmsg_to_cv2(ros_binary_image, "mono8")  
  
  # Convert binary to skeleton
  binary = np.where(cv_binary > 0, 1, cv_binary)    
  skeleton = skeletonize(binary, method='lee')    
  skeleton = np.where(skeleton == 1, 255, skeleton)

 # Ordering pixels using NN-search
  pixels = np.argwhere(skeleton > 0)
  xinit = pixels[:, 1]
  yinit = pixels[:, 0]
  
  init_point = (149, 149)  # base pixel used as seed point for NN-search
  dist_mat = []
  for i in range(len(xinit)):
    dist = np.sqrt((xinit[i]-init_point[0])**2 + (yinit[i]-init_point[1])**2)
    dist_mat = np.append(dist_mat, dist)
    
  index = np.argsort(dist_mat)
  xinit, yinit = xinit[index], yinit[index]
  num_of_segments = rospy.get_param("3linkbot/shape_control/num_of_segments")
  k = rospy.get_param("3linkbot/shape_control/degree")
  point_jump = (len(xinit))/((num_of_segments+k)-1)
  x = np.array(xinit[0::math.floor(point_jump)])
  y = np.array(yinit[0::math.floor(point_jump)])

  if (x[-1] != xinit[-1]) or (y[-1] != yinit[-1]) :
      x = np.append(x, xinit[-1])
      y = np.append(y, yinit[-1])
      if len(x) > (num_of_segments+k):
        x = np.delete(x, -2)
        y = np.delete(y, -2) 

  # Spline fitting  
  tck, u_params = splprep([x, y], k = k, s = 0)   
  # Extracting control points
  cx = tck[1][0]  
  cx = cx.tolist()
  cy = tck[1][1]
  cy = cy.tolist() 
  knots = tck[0]

  # print("control_point", tck[1])

  # print("Parameters generated", u_params)
  # print("knots generated", tck[0])

  # List of control points  
  cp = []
  for i in range(len(cx)-1):
    # cp.append(cx[i+1]) 
    # cp.append(cy[i+1])
    cp.append(cx[i]) 
    cp.append(cy[i])
  
  cp_resp = Float64MultiArray()
  cp_resp.data = cp  

  k_len = math.floor(len(knots)/2) # knot midpoint index

  # Evaluate points between first and mid knot point
  new_params1 = np.linspace(0,knots[k_len],10)
  # print(knots[k_len])
  new_pts1 = splev(new_params1, tck)
  x = new_pts1[0]
  x = x.tolist()
  y = new_pts1[1]
  y = y.tolist()

  ep1 = []
  for i in range(len(x)):
    ep1.append(x[i]) 
    ep1.append(y[i])

  # print("Evaluated points", ep1)

  # Evaluate points between first and mid knot point
  new_params2 = np.linspace(knots[k_len],1,10)
  new_pts2 = splev(new_params2, tck)
  x = new_pts2[0]
  x = x.tolist()
  y = new_pts2[1]
  y = y.tolist()

  ep2 = []
  for i in range(len(x)):
    ep2.append(x[i]) 
    ep2.append(y[i])

  # coefficient evaluation of spline polynomials

  # evaluation pts in the first segment
  pol_params_1 = np.linspace(0, knots[k_len], 5)

  pol_pts1 = splev(pol_params_1, tck)  

  pol_x1 = pol_pts1[0]
  pol_y1 = pol_pts1[1]

  # print(pol_x1, pol_y1)
  # print("evaluation points in first segment", pol_x1, pol_y1)

  # solve for coefficients of quadratic function.
  # a = np.array([[pol_x1[1]**2, pol_x1[1]**1, pol_x1[1]**0], [pol_x1[2]**2, pol_x1[2]**1, pol_x1[2]**0], [pol_x1[3]**2, pol_x1[3]**1, pol_x1[3]**0]])
  # b = np.array([pol_y1[1], pol_y1[2], pol_y1[3]])

  # a = np.array([[pol_x1[1]**1, pol_x1[1]**0], [pol_x1[3]**1, pol_x1[3]**0]])
  # b = np.array([pol_y1[1], pol_y1[3]])
  # coeff1 = np.linalg.solve(a, b)

  a1x = np.array([[pol_params_1[1]**2, pol_params_1[1]**1, pol_params_1[1]**0], \
    [pol_params_1[2]**2, pol_params_1[2]**1, pol_params_1[2]**0], \
    [pol_params_1[3]**2, pol_params_1[3]**1, pol_params_1[3]**0]])
  b1x = np.array([pol_x1[1], pol_x1[2], pol_x1[3]])
  coeffx1 = np.linalg.solve(a1x, b1x)

  coeff_list = coeffx1.tolist()

  a1y = np.array([[pol_params_1[1]**2, pol_params_1[1]**1, pol_params_1[1]**0], \
    [pol_params_1[2]**2, pol_params_1[2]**1, pol_params_1[2]**0], \
      [pol_params_1[3]**2, pol_params_1[3]**1, pol_params_1[3]**0]])
  b1y = np.array([pol_y1[1], pol_y1[2], pol_y1[3]])
  coeffy1 = np.linalg.solve(a1y, b1y)
  
  coeff_list.extend(coeffy1)

  # coeff_list = coeff1.tolist()

  # coeff_list  = list(coeff1)

  # evaluation pts in second segment
  pol_params_2 = np.linspace(knots[k_len], 1, 5)

  # print(pol_params_2)

  pol_pts2 = splev(pol_params_2, tck)

  pol_x2 = pol_pts2[0]
  pol_y2 = pol_pts2[1]

  # print(pol_x2, pol_y2)

  # print("evaluation points in second segment", pol_x2, pol_y2)

  # solve for coefficients of quadratic function.
  # a = np.array([[pol_x2[1]**2, pol_x2[1]**1, pol_x2[1]**0], [pol_x2[2]**2, pol_x2[2]**1, pol_x2[2]**0], [pol_x2[3]**2, pol_x2[3]**1, pol_x2[3]**0]])
  # b = np.array([pol_y2[1], pol_y2[2], pol_y2[3]])
  # a = np.array([[pol_x2[1]**1, pol_x2[1]**0], [pol_x2[3]**1, pol_x2[3]**0]])
  # b = np.array([pol_y2[1], pol_y2[3]])

  a2x = np.array([[pol_params_2[1]**2, pol_params_2[1]**1, pol_params_2[1]**0], \
    [pol_params_2[2]**2, pol_params_2[2]**1, pol_params_2[2]**0], \
    [pol_params_2[3]**2, pol_params_2[3]**1, pol_params_2[3]**0]])
  b2x = np.array([pol_x2[1], pol_x2[2], pol_x2[3]])
  coeffx2 = np.linalg.solve(a2x, b2x)

  coeff_list.extend(coeffx2)

  a2y = np.array([[pol_params_2[1]**2, pol_params_2[1]**1, pol_params_2[1]**0], \
    [pol_params_2[2]**2, pol_params_2[2]**1, pol_params_2[2]**0], \
      [pol_params_2[3]**2, pol_params_2[3]**1, pol_params_2[3]**0]])
  b2y = np.array([pol_y2[1], pol_y2[2], pol_y2[3]])
  coeffy2 = np.linalg.solve(a2y, b2y)

  coeff_list.extend(coeffy2)

  # a2y = np.array([[pol_params_2[1]**1, pol_params_2[1]**0], [pol_params_2[3]**1, pol_params_2[3]**0]])
  # b2y = np.array([pol_y2[1], pol_y2[3]])
  # coeffy2 = np.linalg.solve(a2y, b2y)

  # coeff_list.extend(coeffy2)

  print("coefficients of the first segment", coeffx1, coeffy1)

  print("coefficients of the second segment", coeffx2, coeffy2)

  # coeff_list.extend(list(coeff2))

  # print(coeff_list)
  
  # print(coeff_list)

  # print(coeff_list)

  coeff_resp = Float64MultiArray()
  coeff_resp.data = coeff_list

  ep1_resp = Float64MultiArray()
  ep1_resp.data = ep1

  ep2_resp = Float64MultiArray()
  ep2_resp.data = ep2

  return cp_testResponse(cp_resp, ep1_resp, ep2_resp, coeff_resp)
  
  
def main(args):
  global binary_image_output
  # Initializing ROS
  rospy.init_node('control_service_node')

  # Declaring the control points service
  cp_service = rospy.Service("control_points_output", cp_test, control_points_service)

  rospy.wait_for_service("binary_image_output")

  # create a handle to call the service (The handle will send the request to the service)
  binary_image_output = rospy.ServiceProxy('binary_image_output', bin_img)

  # Initializing publishers
  # control_pts_pub = rospy.Publisher('vsbot/control_points', Float64MultiArray, queue_size= 1)
  # # basis_function_pub = rospy.Publisher('vsbot/basis_function', Float64MultiArray, queue_size= 1)
  # eval_pts_pub = rospy.Publisher('vsbot/evaluated_points', Float64MultiArray, queue_size= 1)
  # knot_pts_pub = rospy.Publisher('vsbot/knot_points', Float64MultiArray, queue_size= 1)

  # cpts = Float64MultiArray()
  # basis_func = Float64MultiArray()
  # eval_points = Float64MultiArray()
  # knot_points = Float64MultiArray()

  # rate = rospy.Rate(30)
  
    # Publish control points
    # cpts.data = cp
    # control_pts_pub.publish(cpts)

    # Publish basis func
    # basis_func.data = bf
    # basis_function_pub.publish(basis_func)

    # Publish evaluated data points for vis
    # eval_points.data = ep
    # eval_pts_pub.publish(eval_points)
    # ep.clear()

    # # Publish evaluated data points for vis
    # knot_points.data = knot_point
    # knot_pts_pub.publish(knot_points)
    # knot_point.clear()



  # waiting for binary service to be up
  

   
  rospy.spin()
  
if __name__ == '__main__':
    main(sys.argv)
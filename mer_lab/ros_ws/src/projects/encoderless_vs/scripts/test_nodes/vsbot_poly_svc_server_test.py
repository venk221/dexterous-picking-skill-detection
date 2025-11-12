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
  binary_resp = resp_bin.output
   
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
  num_of_segments = rospy.get_param("vsbot/shape_control/num_of_segments")
  k = rospy.get_param("vsbot/shape_control/degree")
  point_jump = (len(xinit))/((num_of_segments+k)-1)
  x = np.array(xinit[0::math.floor(point_jump)])
  y = np.array(yinit[0::math.floor(point_jump)])

  if (x[-1] != xinit[-1]) or (y[-1] != yinit[-1]) :
      x = np.append(x, xinit[-1])
      y = np.append(y, yinit[-1])
      if len(x) > (num_of_segments+k):
        x = np.delete(x, -2)
        y = np.delete(y, -2) 

  # polynomial fitting  
  tx = np.linspace(0, 1, 4)
  ty = np.linspace(0, 1, 4)
  t = np.linspace(0, 1, 4)

  print(x, tx, ty)

  coeff_x = np.polyfit(x, tx, 2)  

  ax = np.array([[t[1]**2, t[1]**1, t[1]**0], \
    [t[2]**2, t[2]**1, t[2]**0], \
    [t[3]**2, t[3]**1, t[3]**0]])
  bx = np.array([x[1], x[2], x[3]])

  coeff_new_x = np.linalg.solve(ax, bx)

  # print(coeff_x)
  print(coeff_new_x)


  # print(coeff_x)

  coeff_list = coeff_new_x.tolist()

  coeff_y = np.polyfit(y, ty, 2)

  ay = np.array([[t[1]**2, t[1]**1, t[1]**0], \
    [t[2]**2, t[2]**1, t[2]**0], \
    [t[3]**2, t[3]**1, t[3]**0]])
  by = np.array([y[1], y[2], y[3]])

  coeff_new_y = np.linalg.solve(ay, by)

  coeff_list.extend(coeff_new_y.tolist())

  print(coeff_list)

  # print(coeff_y)

  coeff_resp = Float64MultiArray()
  coeff_resp.data = coeff_list
  
  return cp_testResponse(coeff_resp)
  
  
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
  
  rospy.spin()
  
if __name__ == '__main__':
    main(sys.argv)
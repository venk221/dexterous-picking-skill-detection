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
from sympy import lambdify, bspline_basis_set
from sympy.abc import u
from cv_bridge import CvBridge, CvBridgeError
from encoderless_vs.srv import cp_goal, cp_goalResponse, bin_img, bin_imgResponse

# Declaring cvbridge for ROS
bridge = CvBridge()
binary_image_output = None

def control_points_service(data):
  # defining the service that takes in binary image input and gives control points output

  global bridge
  resp_bin = binary_image_output(1)
  binary_resp = resp_bin.output

  # ros binary image as the service input
  ros_binary_image = binary_resp

  # try: 
  cv_binary = bridge.imgmsg_to_cv2(ros_binary_image, "mono8")
  # except CvBridgeError as e:
  #     print(e)
  
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

  # Spline fitting and extracting tck list for evaluation points
  # and extracting control points 
  tck, u_params = splprep([x, y], k = k, s = 0)
  t = tck[0].tolist()
  cx = tck[1][0].tolist()
  cy = tck[1][1].tolist()
  k = tck[2]

  t_resp = t

  cx_resp = cx
  cy_resp = cy

  k_resp = k
  
  # List of control points

  cp = []
  for i in range(len(cx)-1):
    cp.append(cx[i+1]) 
    cp.append(cy[i+1])

  cp_resp = Float64MultiArray()
  cp_resp.data = cp

  return cp_goalResponse(cp_resp, t_resp, cx_resp, cy_resp, k_resp)
  
  
  
def main(args):
  global binary_image_output
  # Initializing ROS
  rospy.init_node('control_service_node')

  # Declaring the control points service
  cp_service = rospy.Service("control_points_output", cp_goal, control_points_service )

  # waiting for binary service to be up
  rospy.wait_for_service("binary_image_output")
  # create a handle to call the service (The handle will send the request to the service)
  binary_image_output = rospy.ServiceProxy('binary_image_output', bin_img)
  
  rospy.spin()
  
if __name__ == '__main__':
    main(sys.argv)

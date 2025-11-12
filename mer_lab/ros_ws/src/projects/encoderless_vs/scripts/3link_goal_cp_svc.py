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

  #  cp = [x1, y1, x2, y2, x3, y3...] based on number of features

  # ###### Computing theta, an additional feature to be added to cp ########
  # # theta is to be computed using first and last cp

  # # get first control point
  # x1 = cp[0]
  # y1 = cp[1]

  # # get last control point
  # x2 = cp[len(cp)-2]
  # y2 = cp[len(cp)-1]

  # base = abs(x2 - x1)
  # hy = math.sqrt(pow((x1 - x2),2) + pow((y1 - y2),2))

  # th = math.acos(base/hy)
  
  # ##### Appending this additional feature to cp ######
  # cp.append(th)

  cp_resp = Float64MultiArray()
  cp_resp.data = cp

  # Coefficient block
  # setting parameters
  # t0 = u_params[0]
  # t1 = u_params[1]
  # t2 = u_params[2]

  # # setting knot vectors
  # u0 = u1 = u2 = 0
  # u3 = u4 = u5 = 1

  # computing middle row of coeff matrix
  # n20t1 = 1
  # n11t1 = ((u3-t1)/(u3-u2)) * n20t1
  # n21t1 = ((t1-u2)/(u3-u2))*n20t1
  # n02t1 = ((u3-t1)/(u3-u1))* n11t1
  # n12t1 = ((t1-u1)/(u3-u1))*n11t1 + ((u4-t1)/(u4-u2))*n21t1 
  # n22t1 = ((t1-u2)/(u4-u2))*n21t1
  
  # coeffs = [n02t1, n12t1, n22t1]

  # Creating coefficients for pieces



  # BSpline basis element

  # basis = bspline_basis_set(tck[2], tck[0],  u)
  # np_basis = [lambdify(u, b, modules=['numpy']) for b in basis]
    
  # basis_funcn = []
  # for bi in np_basis:
  #   for u_val in u_params:
  #     basis_funcn = np.append(basis_funcn, bi(u_val))
      # if bi(u_val) == 0 or bi(u_val)== 1:
      #   basis_funcn = basis_funcn
      # else:
      #   basis_funcn = np.append(basis_funcn, bi(u_val))
  
  # coeffs = basis_funcn  

  # # print(coeffs)
  # # print(cp)

  # coeff_resp = Float64MultiArray()
  # coeff_resp.data = coeffs

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

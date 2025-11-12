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
from scipy.interpolate import splprep, splev
import time
from sympy import lambdify, bspline_basis_set
from sympy.abc import u
from cv_bridge import CvBridge, CvBridgeError
from encoderless_vs.srv import control_points, control_pointsResponse, bin_img, bin_imgResponse

# Declaring cvbridge for ROS
bridge = CvBridge()
binary_image_output = None

# the following service callback definies the service that 
# takes in binary image input and gives control points output  
def control_points_service(data):  
  global bridge
  
  bin_rqst_tstart = time.time()
  resp_bin = binary_image_output(1)
  bin_rqst_tstop = time.time()
  binary_resp = resp_bin.output
   
  # ros binary image as the service input
  ros_binary_image = binary_resp
  
  skel_tstart = time.time() 
  cv_binary = bridge.imgmsg_to_cv2(ros_binary_image, "mono8")  
  
  # Convert binary to skeleton
  binary = np.where(cv_binary > 0, 1, cv_binary)    
  skeleton = skeletonize(binary, method='lee')    
  skeleton = np.where(skeleton == 1, 255, skeleton)


  # print("Skeleton Length", len(skeleton))
  skel_tstop = time.time()

  # Ordering pixels using NN-search
  nn_tstart = time.time()
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
              
  # Spline fitting

  tck, u_params = splprep([x, y], k = k, s = 0)


  # Extracting control points

  cx = tck[1][0]  
  cx = cx.tolist()
  cy = tck[1][1]
  cy = cy.tolist()
  
  # List of control points
  # putting 1ist control points at the end as it is not changing and can be discarded
  cp = []
  # print(type(cp))
  for i in range(len(cx)-1):
    cp.append(cx[i+1]) 
    cp.append(cy[i+1])
    
  cp_resp = Float64MultiArray()
  cp_resp.data = cp  

  

  # service returns control points and coeffs
  return control_pointsResponse(cp_resp) #, skel_tstart, skel_tstop, nn_tstart, nn_tstop, spl_tstart, spl_tstop, bin_rqst_tstart, bin_rqst_tstop)  
  
def main(args):
  global binary_image_output
  # Initializing ROS
  rospy.init_node('control_service_node')

  # Declaring the control points service
  cp_service = rospy.Service("control_points_output", control_points, control_points_service )

  # waiting for binary service to be up
  rospy.wait_for_service("binary_image_output")

  # create a handle to call the service (The handle will send the request to the service)
  binary_image_output = rospy.ServiceProxy('binary_image_output', bin_img)

   
  rospy.spin()
  
if __name__ == '__main__':
    main(sys.argv)
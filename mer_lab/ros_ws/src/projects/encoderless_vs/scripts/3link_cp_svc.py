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
from cv_bridge import CvBridge, CvBridgeError
from encoderless_vs.srv import bin_img, bin_imgResponse, control_points, control_pointsResponse

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
  # ros binary image as the service input
  ros_binary_image = binary_resp
  cv_binary = bridge.imgmsg_to_cv2(ros_binary_image, "mono8")  

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


  # DownSampling points for fitting spline

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

  # List of control points  
  cp = []
  for i in range(len(cx)-1):
    cp.append(cx[i+1]) 
    cp.append(cy[i+1])
  
  cp_resp = Float64MultiArray()
  cp_resp.data = cp  

  return control_pointsResponse(cp_resp)
  
  
def main(args):
  global binary_image_output
  # Initializing ROS
  rospy.init_node('control_service_node')

  # Declaring the control points service
  cp_service = rospy.Service("control_points_output", control_points, control_points_service)

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
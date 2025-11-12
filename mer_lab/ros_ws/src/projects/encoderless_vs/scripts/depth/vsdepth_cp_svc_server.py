#!/usr/bin/env python3

import sys
import rospy
import cv2
import numpy as np
from skimage.morphology import skeletonize
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from scipy.interpolate import splprep, splev
import time
from cv_bridge import CvBridge, CvBridgeError
from encoderless_vs.srv import control_points, control_pointsResponse, bin_img, bin_imgResponse

# Declaring cvbridge for ROS
bridge = CvBridge()
binary_image_output = None
  
def control_points_service(data):
  # defining the service that takes in binary image input and gives control points output
  global bridge

  bin_rqst_tstart = time.time()
  resp_bin = binary_image_output(1)
  bin_rqst_tstop = time.time()
  binary_resp = resp_bin.output
   
  # ros binary image as the service input
  ros_binary_image = binary_resp
  # try: 
  skel_tstart = time.time() 
  cv_binary = bridge.imgmsg_to_cv2(ros_binary_image, "mono8")
  # except CvBridgeError as e:
  #     print(e)
  
  # Convert binary to skeleton
  binary = np.where(cv_binary > 0, 1, cv_binary)    
  skeleton = skeletonize(binary, method='lee')    
  skeleton = np.where(skeleton == 1, 255, skeleton)
  skel_tstop = time.time()

  cv2.imshow("Skeleton", skeleton)
  cv2.waitKey(1)

  # print("Time for skeletonization", skel_t)
  # Ordering pixels using NN-search
  nn_tstart = time.time()
  pixels = np.argwhere(skeleton > 0)
  # print("Skeleton Pixels", pixels)
  xinit = pixels[:, 1]
  yinit = pixels[:, 0]
  
  # tstart = time.time()
  init_point = (490, 220)  # base pixel used as seed point for NN-search
  dist_mat = []
  for i in range(len(xinit)):
    dist = np.sqrt((xinit[i]-init_point[0])**2 + (yinit[i]-init_point[1])**2)
    dist_mat = np.append(dist_mat, dist)
    
  index = np.argsort(dist_mat)
  xinit, yinit = xinit[index], yinit[index]
  point_jump = (len(xinit))/2
  x = np.array(xinit[0::round(point_jump)])
  y = np.array(yinit[0::round(point_jump)])
  
  if (x[-1] != xinit[-1]) or (y[-1] != yinit[-1]) :
      x = np.append(x, xinit[-1])
      y = np.append(y, yinit[-1])
      if ((x[-1] - x[-2] == 1) or (x[-1] - x[-2] == -1)) and ((y[-1] - y[-2] == 1) or (y[-1] - y[-2] == -1)):
        x = np.delete(x, -2)
        y = np.delete(y, -2)
  
  nn_tstop = time.time()
  # print("NN Search", tstart, tstop, nn_t)
  # 
      
  # Spline fitting
  spl_tstart = time.time()
  tck, u_params = splprep([x, y], k =2, s = 1)

  u = np.linspace(0,1,10)
  newPts = splev(u, tck)
  x = newPts[0]
  y = newPts[1]  
   
  points = np.c_[x,y]

  # #   # Create a blank image 
  blank_img = np.zeros((500, 500, 3), dtype = "uint8") 
    
  for i in range(len(points)):
    x = np.int(points[i][0])
    y = np.int(points[i][1])
    cv2.circle(blank_img,(x, y),2,(255,0,0),-1) 
  
  # cv2.imshow("Points", blank_img)
  # cv2.waitKey(1)
  # keeping the tck tuple for further use
  # tck = list(tck)
  # tck_resp
  # Extracting control points
  cx = tck[1][0]
  cx = cx.tolist()
  cy = tck[1][1]
  cy = cy.tolist()

  points = np.c_[cx,cy]

    # Create a blank image 
  # blank_img = np.zeros((300, 300, 3), dtype = "uint8") 
    
  for i in range(len(points)):
    x = np.int(points[i][0])
    y = np.int(points[i][1])
    cv2.circle(blank_img,(x, y),2,(0,0,255),-1) 
  
  cv2.imshow("Control Points", blank_img)
  cv2.waitKey(1)

  # List of control points
  # putting 1st control points at the end as it is not changing and can be discarded
  cp = [cx[1], cy[1], cx[2], cy[2], cx[0], cy[0]]
  cp_resp = Float64MultiArray()
  cp_resp.data = cp  
  spl_tstop = time.time()
  # print("splprep", tstart, tstop, spline_t)
  return control_pointsResponse(cp_resp, skel_tstart, skel_tstop, 
                                nn_tstart, nn_tstop, spl_tstart, spl_tstop, 
                                  bin_rqst_tstart, bin_rqst_tstop)
  
  
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

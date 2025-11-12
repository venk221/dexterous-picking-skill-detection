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
from cv_bridge import CvBridge, CvBridgeError
from encoderless_vision_dl.srv import franka_control_points, franka_control_pointsResponse, franka_bin_img, franka_bin_imgResponse
from astar import MyAstar

# Declaring cvbridge for ROS
bridge = CvBridge()
binary_image_service_response = None
ee_marker = None
markers = None
l = 0

# Constrain the skeleton to robot
def constrain_skeleton(skeleton):
    global ee_marker

    ee_marker = markers[0:2]
    base_marker = markers[2:4]
    base_corner_br = markers[6:8]      # bottom right
    base_corner_bl = markers[8:10]     # bottom left
    ee_corner_br = markers[10:12]      # bottom right
    ee_corner_bl = markers[12:14]      # bottom left
    ee_corner_tl = markers[14:16]      # top left
    ee_corner_tr = markers[16:18]      # top right
    # start_t = time.time()
    # Center of base marker bottom edge
    robot_base = [int((base_corner_bl[0] + base_corner_br[0])/2), int((base_corner_bl[1] + base_corner_br[1])/2)]

    # Skeleton pts
    skel_px = np.flip(np.argwhere(skeleton>0))

    # Find skeleton points closest to base and end-effector
    distance_from_base = (robot_base[0]-skel_px[:,0])**2 + (robot_base[1] - skel_px[:,1])**2
    distance_from_ee = (ee_marker[0]-skel_px[:,0])**2 + (ee_marker[1] - skel_px[:,1])**2

    # pts with min distance
    start = tuple(skel_px[np.argmin(distance_from_base)])
    end = tuple(skel_px[np.argmin(distance_from_ee)])
    # end_t = time.time()

    # print("constrain:", end_t - start_t)

    return start, end, robot_base, skel_px

# the following service callback definies the service that 
# takes in binary image input and gives control points output  
def franka_control_points_service(data):  
  global bridge, markers, l

  print("service called")
  resp_bin = binary_image_service_response(1)

  ros_binary_image = resp_bin.img
  markers = resp_bin.markers.data
  
  # ros binary image as the service input  
  cur_depth_img = bridge.imgmsg_to_cv2(ros_binary_image, "mono8")  
  
  # Convert binary to skeleton
  cur_depth_img = np.where(cur_depth_img == 255, 1, cur_depth_img)
  skeleton = skeletonize(cur_depth_img, method='lee')
  skeleton = np.where(skeleton >0, 255, skeleton)

  # cv2.imwrite("skeleton_goal_original.jpg", skeleton)
  # Block to find the end points
  start, goal, robot_base, skel_px = constrain_skeleton(skeleton)
    
  # Recontruct skeleton
  a_star = MyAstar(start, goal, skeleton.shape[0], skeleton.shape[1], skeleton)
  if(a_star.IsValid(start[0], start[1])):
      if(a_star.IsValid(goal[0], goal[1])):
          if(a_star.IsObstacle(start[0],start[1]) == False):
              if(a_star.IsObstacle(goal[0], goal[1]) == False):
                  (exploredStates, backtrackStates, distanceFromStartToGoal) = a_star.search()
                  if(distanceFromStartToGoal == float('inf')):
                      print("\nNo optimal path found.")
                  else:
                      pass
                      # print("\nOptimal path found. Distance is " + str(distanceFromStartToGoal))
              else:
                  print("The entered goal node is an obstacle ")
                  print("Please check README.md file for running astar.py file.")
          else:
              print("The entered start node is an obstacle ")
              print("Please check README.md file for running astar.py file.")
      else:
          print("The entered goal node outside the map ")
          print("Please check README.md file for running astar.py file.")
  else:
      print("The entered start node is outside the map ")
      print("Please check README.md file for running astar.py file.")

  num_of_segments = rospy.get_param("vsbot/shape_control/num_of_segments")
  k = rospy.get_param("vsbot/shape_control/degree")
  
  # Downsampling skeleton for spline fit
  n = k + num_of_segments - 1     # 1/downsample factor
  seg_size = int(len(backtrackStates)/n) - 1       
  downsampled_skeleton_x = []
  downsampled_skeleton_y = []
  for i in range(n+1):
      downsampled_skeleton_x.append(backtrackStates[i*seg_size][0])
      downsampled_skeleton_y.append(backtrackStates[i*seg_size][1])

  # Fitting the Spline
  tck, u_params = splprep([downsampled_skeleton_x, downsampled_skeleton_y], k = k, s = 0)

  # Evaluating the spline
  new_params = np.linspace(0,1,10)
  new_pts = splev(new_params, tck)
  x = new_pts[0].tolist()
  y = new_pts[1].tolist()  
  
  # Extracting the control points
  cx = tck[1][0]  
  cx = cx.tolist()
  cy = tck[1][1]
  cy = cy.tolist()

  cv_img = cv2.cvtColor(cur_depth_img, cv2.COLOR_GRAY2BGR)

  for pixel in backtrackStates:
        cv2.circle(cv_img, (pixel[0],pixel[1]), 3, (255,255,255), -1)

  cv2.imwrite("/home/jc-merlab/Pictures/Data/skel_test/skeleton_" + str(l) + ".jpg", cv_img)
  
  # Constraining spline by constraining first and last control points
  cx[0] = robot_base[0]
  cy[0] = robot_base[1]
  cx[-1] = ee_marker[0]
  cy[-1] = ee_marker[1]

  points = np.c_[x,y]
  ct_pts = np.c_[cx, cy]

#   for i in range(len(points)):
#       x = np.int(points[i][0])
#       y = np.int(points[i][1])
#       cv2.circle(cv_img,(x, y),2,(0,0,255),-1)


#   for i in range(len(ct_pts)):
#       x = np.int(ct_pts[i][0])
#       y = np.int(ct_pts[i][1])
#       cv2.circle(cv_img,(x, y),5,(0,255,255),-1)
  

  # List of control points
  # putting 1ist control points at the end as it is not changing and can be discarded
  cp = []
  for i in range(len(cx)-1):
    cp.append(cx[i+1]) 
    cp.append(cy[i+1])
  
  print("Control Points", cp)
    
  cp_resp = Float64MultiArray()
  cp_resp.data = cp  

  l = l+1

  return franka_control_pointsResponse(cp_resp) 
  
def main(args):
  global binary_image_service_response
  # Initializing ROS
  rospy.init_node('franka_control_service_node')

  # Declaring the control points service
  cp_service = rospy.Service("franka_control_service", franka_control_points, franka_control_points_service )

  # waiting for binary service to be up
  rospy.wait_for_service("binary_image_service_response")

  # create a handle to call the service (The handle will send the request to the service)
  binary_image_service_response = rospy.ServiceProxy('binary_image_service_response', franka_bin_img)

  rospy.spin()
  
if __name__ == '__main__':
    main(sys.argv)
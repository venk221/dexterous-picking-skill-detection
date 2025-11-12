#!/usr/bin/env python3

import sys
import rospy
import cv2
import csv
import numpy as np
import math
from skimage.morphology import skeletonize
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from scipy.interpolate import splprep, splev
import time
from cv_bridge import CvBridge, CvBridgeError
from encoderless_vs.srv import franka_control_points, franka_control_pointsResponse, dream_img, dream_imgResponse

# Declaring cvbridge for ROS
bridge = CvBridge()
dream_kp_service_response = None
m = 0
# the following service callback definies the service that 
# takes in binary image input and gives control points output  
def franka_control_points_service(data):  
  global bridge, m

  resp_bin = dream_kp_service_response(1)
  ros_binary_image = resp_bin.img
  key_points = resp_bin.kp.data

  print("Key points at the control node", key_points)
  # ros binary image as the service input  
  cv_binary = bridge.imgmsg_to_cv2(ros_binary_image, "bgr8")  
  
  # Convert binary to skeleton
  binary = np.where(cv_binary > 0, 1, cv_binary)
  skeleton = skeletonize(binary, method='lee')
  skeleton = cv2.cvtColor(skeleton, cv2.COLOR_BGR2GRAY)
  skeleton = np.where(skeleton == 1, 255, skeleton)
  cv2.imwrite("/home/merlab/Pictures/test_skel_new/orig_skel/o_skel"+str(m)+".jpg", skeleton)  

  # for i in range()

  # Block to find the end points
  # (rows, cols) = np.nonzero(skeleton)
  # end_coords = [] # storing the end points
  # for (r, c) in zip(rows, cols):
  #     (col_neigh, row_neigh) = np.meshgrid(
  #         np.array([c-1, c, c+1]), np.array([r-1, r, r+1]))
  #     col_neigh = col_neigh.astype('int')
  #     row_neigh = row_neigh.astype('int')
  #     pix_nbhood = skeleton[row_neigh, col_neigh].ravel() != 0
  #     if np.sum(pix_nbhood) < 3:
  #         end_coords.append((c, r))

  # pixels = np.argwhere(skeleton == 255)
  # pixels = np.flip(pixels) 

  # pix_x = pixels[:, 0]
  # pix_y = pixels[:, 1]
  # dist_near_base = np.sqrt((pix_x - base_center[0]) **
  #                          2 + (pix_y - base_center[1]) ** 2)
  # dist_near_ee = np.sqrt((pix_x - ee_center[0]) **
  #                        2 + (pix_y - ee_center[1]) ** 2)
  # dist_min_base = np.argmin(dist_near_base)
  # dist_min_ee = np.argmin(dist_near_ee)
  # pixels = pixels.tolist()
  # closest_base = pixels[dist_min_base]  
  # copy_pix = np.array(pixels)
  # init_point = closest_base
  
  # row_index, = np.where(np.all(copy_pix == (init_point), axis=1))
  # copy_pix = np.delete(copy_pix, row_index, axis=0)
  # ordered = init_point
  # while len(copy_pix) > 0:
  #   distances = np.sqrt((copy_pix[:, 0] - init_point[0]) ** 2 + (copy_pix[:, 1] - init_point[1]) ** 2)
  #   nearest_index = np.argmin(distances)
  #   init_point = copy_pix[nearest_index]  
  #   copy_pix = np.delete(copy_pix, nearest_index, axis=0)
  #   ordered = np.append(ordered, [init_point])
  # ordered = np.reshape(ordered, (-1, 2))
  # pixels = ordered

  # pix_x = pixels[:, 0]
  # pix_y = pixels[:, 1]
  # dist_near_base = np.sqrt((pix_x - base_center[0]) **
  #                          2 + (pix_y - base_center[1]) ** 2)
  # dist_near_ee = np.sqrt((pix_x - ee_center[0]) **
  #                        2 + (pix_y - ee_center[1]) ** 2)
  # dist_min_base = np.argmin(dist_near_base)
  # dist_min_ee = np.argmin(dist_near_ee)
  # pixels = pixels.tolist()
  # closest_base = pixels[dist_min_base]   # pixel closest to base center
  # closest_ee = pixels[dist_min_ee]

  # index_base = pixels.index([closest_base[0], closest_base[1]])
  # index_end = pixels.index([closest_ee[0], closest_ee[1]])

  # new_pixels = []
  # for index, value in enumerate(pixels[index_base:]):
  #   new_pixels.append(value)
  #   if index == index_end:
  #     break
  
  # new_pixels = np.array(new_pixels)

  
  # # in the following blosck we are finding the white pixels on the skeleton closest to the markers
  # robot_base = end_coords[-1]
  # pixels = np.argwhere(skeleton == 255)
  # pixels = np.flip(pixels)
  # pix_x = pixels[:, 0]
  # pix_y = pixels[:, 1]
  # dist_near_base = np.sqrt((pix_x - base_center[0]) **
  #                          2 + (pix_y - base_center[1]) ** 2)
  # dist_near_ee = np.sqrt((pix_x - ee_center[0]) **
  #                        2 + (pix_y - ee_center[1]) ** 2)
  # dist_min_base = np.argmin(dist_near_base)
  # dist_min_ee = np.argmin(dist_near_ee)
  # pixels = pixels.tolist()
  # closest_base = pixels[dist_min_base]   # pixel closest to base center
  # closest_ee = pixels[dist_min_ee]       # pixel closest to ee center

  # # block to remove the part of the skeleton from base marker to the base of robot
  # cut_base_index = pixels.index([robot_base[0], robot_base[1]])
  # base_index = pixels.index([closest_base[0], closest_base[1]])
  # points_removed_base = pixels[cut_base_index:base_index-1]

  # for i in range(len(points_removed_base)-1):
  #       skeleton[points_removed_base[i][1], points_removed_base[i][0]] = 0
  
  # # Order the remaining white pixels for further skeleton processing
  # pixels = np.argwhere(skeleton == 255)
  # pixels = np.flip(pixels)
  # copy_pix = pixels
  # init_point = pixels[0]  
  # row_index, = np.where(np.all(copy_pix == (init_point), axis=1))
  # copy_pix = np.delete(copy_pix, row_index, axis=0)
  # ordered = init_point
  # while len(copy_pix) > 0:
  #     distances = np.sqrt(
  #         (copy_pix[:, 0] - init_point[0]) ** 2 + (copy_pix[:, 1] - init_point[1]) ** 2)
  #     nearest_index = np.argmin(distances)
  #     init_point = copy_pix[nearest_index]  
  #     copy_pix = np.delete(copy_pix, nearest_index, axis=0)
  #     ordered = np.append(ordered, [init_point])
  # ordered = np.reshape(ordered, (-1, 2))
  # pixels = ordered.tolist()

  # # Remove the part of skeleton from ee marker to end of the robot
  # start_index = pixels.index([init_point[0], init_point[1]])
  # end_index = pixels.index([closest_ee[0], closest_ee[1]])

  # # points_considered = pixels[end_index:start_index]
  # points_removed_ee = pixels[end_index+1:]

  # for i in range(len(points_removed_ee)):
  #   skeleton[points_removed_ee[i][1], points_removed_ee[i][0]] = 0

  # In this block we look for the considered body of the skeleton and check for braches and remove them
  # (rows, cols) = np.nonzero(skeleton)
  # end_coords = []
  # intersection = []
  # for (r, c) in zip(rows, cols):
  #     (col_neigh, row_neigh) = np.meshgrid(
  #         np.array([c-1, c, c+1]), np.array([r-1, r, r+1]))
  #     col_neigh = col_neigh.astype('int')
  #     row_neigh = row_neigh.astype('int')
  #     pix_nbhood = skeleton[row_neigh, col_neigh].ravel() != 0
  #     if np.sum(pix_nbhood) < 3:
  #         end_coords.append((c, r))
  #     # the list of co-ordinates for intersection points
  #     elif np.sum(pix_nbhood) > 3:
  #         intersection.append((c, r))

  # pixels = np.argwhere(skeleton == 255)
  # pixels = np.flip(pixels).tolist()
  # if intersection:
  #   for i in range(len(intersection)):
  #     intersection_index = pixels.index([intersection[i][0], intersection[i][1]])      
  #     for j in range(len(end_coords)):
  #       if j != 0 and j != -1 and j :
  #         end_index = pixels.index([end_coords[i][0], end_coords[i][1]])
  #         if end_index > intersection_index:
  #           intersection_seg = pixels[intersection_index:end_index]
  #         else:
  #           intersection_seg = pixels[end_index:intersection_index]
  
  # for i in range(len(intersection_seg)):
  #   skeleton[intersection_seg[i][1], intersection_seg[i][0]] = 0
  
  # reordering the points after constraining the skeleton
  # new_pix = np.argwhere(skeleton == 255)
  # new_pix = np.flip(new_pix)
  # copy_pix = new_pix
  # init_point = new_pix[0]
  # row_index, = np.where(np.all(copy_pix == (init_point), axis=1))
  # copy_pix = np.delete(copy_pix, row_index, axis=0)
  # ordered = init_point
  # while len(copy_pix) > 0:
  #     distances = np.sqrt(
  #         (copy_pix[:, 0] - init_point[0]) ** 2 + (copy_pix[:, 1] - init_point[1]) ** 2)
  #     nearest_index = np.argmin(distances)
  #     init_point = copy_pix[nearest_index] 
  #     copy_pix = np.delete(copy_pix, nearest_index, axis=0)
  #     ordered = np.append(ordered, [init_point])
  
  # ordered = np.reshape(ordered, (-1, 2))
  # new_pix = ordered

  # xinit = new_pixels[:,0]
  # yinit = new_pixels[:,1]

  # # Downsampling the points in the skeleton to fit a curve
  # num_of_segments = rospy.get_param("vsbot/shape_control/num_of_segments")
  # k = rospy.get_param("vsbot/shape_control/degree")
  # point_jump = (len(xinit))/((num_of_segments+k)-1)
  # x = np.array(xinit[0::math.floor(point_jump)])
  # y = np.array(yinit[0::math.floor(point_jump)])
  # if (x[-1] != xinit[-1]) or (y[-1] != yinit[-1]) :
  #   x = np.append(x, xinit[-1])
  #   y = np.append(y, yinit[-1])
  #   if len(x) > (num_of_segments+k):
  #     x = np.delete(x, -2)
  #     y = np.delete(y, -2)
  # x[0] = base_center[0]
  # y[0] = base_center[1]  
  # x[-1] = ee_center[0]
  # y[-1] = ee_center[1]
  
  # # Fitting the Spline
  # tck, u_params = splprep([x, y], k = k, s = 0)

  # # Evaluating the spline
  # new_params = np.linspace(0,1,10)
  # new_pts = splev(new_params, tck)
  # x = new_pts[0].tolist()
  # y = new_pts[1].tolist()
  
  # # Extracting the control points
  # cx = tck[1][0]  
  # cx = cx.tolist()
  # cy = tck[1][1]
  # cy = cy.tolist()
  
  # # List of control points
  # # putting 1ist control points at the end as it is not changing and can be discarded
  # cp = []
  # for i in range(len(cx)-1):
  #   cp.append(cx[i+1]) 
  #   cp.append(cy[i+1])
    
  cp_resp = Float32MultiArray()
  cp_resp.data = cp  

  return franka_control_pointsResponse(cp_resp) 
  
def main(args):
  global dream_kp_service_response
  # Initializing ROS
  rospy.init_node('franka_control_service_node')

  # Declaring the control points service
  cp_service = rospy.Service("franka_control_service", franka_control_points, franka_control_points_service )

  # waiting for binary service to be up
  rospy.wait_for_service("dream_kp_service_response")

  # create a handle to call the service (The handle will send the request to the service)
  dream_kp_service_response = rospy.ServiceProxy('dream_kp_service_response', dream_img)

  rospy.spin()
  
if __name__ == '__main__':
    main(sys.argv)
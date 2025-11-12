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
from encoderless_vs.srv import franka_control_points, franka_control_pointsResponse, franka_bin_img, franka_bin_imgResponse

# Declaring cvbridge for ROS
bridge = CvBridge()
binary_image_service_response = None
m = 0
# the following service callback definies the service that 
# takes in binary image input and gives control points output  
def franka_control_points_service(data):  
  global bridge, m

  resp_bin = binary_image_service_response(1)
  ros_binary_image = resp_bin.img
  base_center = resp_bin.base.data
  ee_center = resp_bin.ee.data

  print("Base center in skel", base_center)
  print("ee center in skel", ee_center)

  # ros binary image as the service input  
  cv_binary = bridge.imgmsg_to_cv2(ros_binary_image, "bgr8")  
  
  # Convert binary to skeleton
  binary = np.where(cv_binary == 255, 1, cv_binary)
  skeleton = skeletonize(binary, method='lee')
  skeleton = cv2.cvtColor(skeleton, cv2.COLOR_BGR2GRAY)
  skeleton = np.where(skeleton == 1, 255, skeleton)
  cv2.imwrite("/home/merlab/Pictures/test_skel_new/orig_skel/o_skel"+str(m)+".jpg", skeleton)  
  
  # Block to find the end points
  (rows, cols) = np.nonzero(skeleton)
  end_coords = [] # storing the end points
  intersection = []
  for (r, c) in zip(rows, cols):
      (col_neigh, row_neigh) = np.meshgrid(
          np.array([c-1, c, c+1]), np.array([r-1, r, r+1]))
      col_neigh
      col_neigh = col_neigh.astype('int')
      row_neigh = row_neigh.astype('int')
      # print(col_neigh)
      # print(row_neigh)
      pix_nbhood = skeleton[row_neigh, col_neigh].ravel() != 0
      # print(pix_nbhood)
      if np.sum(pix_nbhood) < 3:
          end_coords.append((c, r))
      elif np.sum(pix_nbhood) > 3:
          intersection.append((c,r))

  print("End Coordinates frame:"+str(m), end_coords)
  print("Intersection frame:"+str(m), intersection)

  pix = np.argwhere(skeleton == 255)
  pix = np.flip(pix)
  print(pix)
  skeleton = cv2.cvtColor(skeleton, cv2.COLOR_GRAY2BGR)
  cv2.circle(skeleton,(int(base_center[0]), int(base_center[1])),5,(0,255,0),-1)
  cv2.circle(skeleton,(int(ee_center[0]), int(ee_center[1])),5,(0,0,255),-1)
  cv2.circle(skeleton,(np.int(pix[-1][0]), np.int(pix[-1][1])),5,(255,0,0),-1)

  # cv2.imwrite("/home/merlab/Pictures/test_skel_new/orig_skel/o_skel"+str(m)+".jpg", skeleton)  
  
  skeleton = cv2.cvtColor(skeleton, cv2.COLOR_BGR2GRAY)  
  
  # skeleton = cv2.cvtColor(skeleton, cv2.COLOR_BGR2GRAY)
  
  # in the following blosck we are finding the white pixels on the skeleton closest to the markers
  robot_base = end_coords[-1]  
  pixels = np.argwhere(skeleton == 255)
  pixels = np.flip(pixels) 
  
  pix_x = pixels[:, 0]
  pix_y = pixels[:, 1]
  dist_near_base = np.sqrt((pix_x - base_center[0]) **
                           2 + (pix_y - base_center[1]) ** 2)
  dist_near_ee = np.sqrt((pix_x - ee_center[0]) **
                         2 + (pix_y - ee_center[1]) ** 2)
  dist_min_base = np.argmin(dist_near_base)
  dist_min_ee = np.argmin(dist_near_ee)
  pixels = pixels.tolist()
  closest_base = pixels[dist_min_base]  
  copy_pix = np.array(pixels)
  init_point = closest_base
  row_index, = np.where(np.all(copy_pix == (init_point), axis=1))
  copy_pix = np.delete(copy_pix, row_index, axis=0)
  ordered = init_point
  while len(copy_pix) > 0:
  # while init_point.all != exit_point.all:
      # print("Init_point first",init_point)
      distances = np.sqrt((copy_pix[:, 0] - init_point[0]) ** 2 + (copy_pix[:, 1] - init_point[1]) ** 2)
      # if len(distances) == 0:
      #   break
      nearest_index = np.argmin(distances)
      init_point = copy_pix[nearest_index]  
      # print("init_point updated", init_point)
      copy_pix = np.delete(copy_pix, nearest_index, axis=0)
      ordered = np.append(ordered, [init_point])
  ordered = np.reshape(ordered, (-1, 2))
  pixels = ordered
  
  # start_index = pixels.index([init_point[0], init_point[1]])
  # end_index = pixels.index([closest_ee[0], closest_ee[1]])

  pix_x = pixels[:, 0]
  pix_y = pixels[:, 1]
  dist_near_base = np.sqrt((pix_x - base_center[0]) **
                           2 + (pix_y - base_center[1]) ** 2)
  dist_near_ee = np.sqrt((pix_x - ee_center[0]) **
                         2 + (pix_y - ee_center[1]) ** 2)
  dist_min_base = np.argmin(dist_near_base)
  dist_min_ee = np.argmin(dist_near_ee)
  pixels = pixels.tolist()
  closest_base = pixels[dist_min_base]   # pixel closest to base center
  closest_ee = pixels[dist_min_ee]       # pixel closest to ee center

  # robot_base_index = pixels.index([robot_base[0], robot_base[1]])
  
  # skeleton = cv2.cvtColor(skeleton, cv2.COLOR_GRAY2BGR)
  # cv2.circle(skeleton,(int(closest_base[0]), int(closest_base[1])),5,(255,255,0),-1)
  # cv2.circle(skeleton,(int(closest_ee[0]), int(closest_ee[1])),5,(0,255,255),-1)
  
  # cv2.imwrite("/home/merlab/Pictures/test_skel_new/pruned_skel/p_skel"+str(m)+".jpg", skeleton)

  # skeleton = cv2.cvtColor(skeleton, cv2.COLOR_BGR2GRAY)

  # init_point = closest_base
  exit_point = closest_ee

  print("Closest Base", closest_base)
  print("Closest EE", closest_ee)
  index_base = pixels.index([closest_base[0], closest_base[1]])
  index_end = pixels.index([closest_ee[0], closest_ee[1]])
  print("Index_end", index_end)
  print("Index_base", index_base)
  # slice skeleton from end to base

  new_pixels = []
  for index, value in enumerate(pixels[index_base:]):
    new_pixels.append(value)
    if index == index_end:
      break 

  skeleton = cv2.cvtColor(skeleton, cv2.COLOR_GRAY2BGR)

  for i in range(len(new_pixels)):
    x = int(new_pixels[i][0])
    y = int(new_pixels[i][1])
    cv2.circle(skeleton,(x, y),3,(255,0,255),-1)

  cv2.circle(skeleton,(int(closest_base[0]), int(closest_base[1])),6,(0,255,0),-1)

  
  cv2.imwrite("/home/merlab/Pictures/test_skel_new/final_skel/f_skel"+str(m)+".jpg", skeleton)

  skeleton = cv2.cvtColor(skeleton, cv2.COLOR_BGR2GRAY)
  
  new_pixels = np.array(new_pixels)

  xinit = new_pixels[:,0]
  yinit = new_pixels[:,1]

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
  x[0] = base_center[0]
  y[0] = base_center[1]  
  x[-1] = ee_center[0]
  y[-1] = ee_center[1]
  
  # Fitting the Spline
  tck, u_params = splprep([x, y], k = k, s = 0)

  # Evaluating the spline
  new_params = np.linspace(0,1,10)
  new_pts = splev(new_params, tck)
  x = new_pts[0].tolist()
  y = new_pts[1].tolist()

  print(x)
  print(y)

  skeleton = cv2.cvtColor(skeleton, cv2.COLOR_GRAY2BGR)

  for i in range(len(x)):
    x_new = int(x[i])
    y_new = int(y[i])
    cv2.circle(skeleton,(x_new, y_new),5,(255,0,0),-1)

  cv2.imwrite("/home/merlab/Pictures/test_skel_new/eval_skel/eval_skel"+str(m)+".jpg", skeleton)
  
  # Extracting the control points
  cx = tck[1][0]  
  cx = cx.tolist()
  cy = tck[1][1]
  cy = cy.tolist()
  
  # List of control points
  # putting 1ist control points at the end as it is not changing and can be discarded
  cp = []
  for i in range(len(cx)-1):
    cp.append(cx[i+1]) 
    cp.append(cy[i+1])
    
  cp_resp = Float32MultiArray()
  cp_resp.data = cp  


  # # print(len(pixels))
  # # new_pixels = []
  # # for index, value in enumerate(pixels[index_base:]):
  # #   new_pixels.append(value)
  # #   if index == index_end:
  # #     break

  # # new_pixels.sort()
  # # new_index_end = new_pixels.index([closest_ee[0], closest_ee[1]])

  # # print("New Index end", new_pixels[new_index_end])
  # # print("New Pixels", new_pixels[-1])

  # # # updated_pixels = []

  # # # for index, value in enumerate(new_pixels):
  # # #   updated_pixels.append(value)
  # # #   if index == new_index_end:
  # # #     break
 
  # # # print(len(new_pixels))

  # # skeleton = cv2.cvtColor(skeleton, cv2.COLOR_GRAY2BGR)

  # # for i in range(len(new_pixels)):
  # #   x = int(new_pixels[i][0])
  # #   y = int(new_pixels[i][1])
  # #   cv2.circle(skeleton,(x, y),0,(255,255,255),-1)
  
  # # cv2.circle(skeleton,(new_pixels[0][0], new_pixels[0][1]),5,(0,0,255),-1)
  # # cv2.circle(skeleton,(new_pixels[-1][0], new_pixels[-1][1]),5,(255,0,0),-1)
  # # cv2.circle(skeleton,(new_pixels[new_index_end][0], new_pixels[new_index_end][1]),5,(0,255,0),-1)
  


  
  

  # # robot_base_index = pixels.index([robot_base[0], robot_base[1]])
  # # print("index base", index_base)
  # # print("robot_base", robot_base_index)
  # # points_removed_base = pixels[robot_base_index:index_base-1]

  # # for i in range(len(points_removed_base)):
  # #   skeleton[points_removed_base[i][1], points_removed_base[i][0]] = 0
  
  
  # # cv2.imwrite("/home/merlab/Pictures/test_skel_new/pruned_skel/p_skel"+str(m)+".jpg", skeleton)
  

  # # index_ee = pixels.index([closest_ee[0], closest_ee[1]]) 
 
  
  # # print(init_point.all == exit_point.all)
  # # points_included =  pixels[index_base:index_ee] 

  # skeleton = cv2.cvtColor(skeleton, cv2.COLOR_BGR2GRAY)

  # # cv2.circle(skeleton, (int(init_point[0]), int(
  # #           init_point[1])), 5, [0, 255, 0], -1) 
  # # cv2.circle(skeleton, (int(exit_point[0]), int(
  # #           exit_point[1])), 5, [0, 255, 0], -1) 
  # # cv2.imwrite("/home/merlab/Pictures/test_skel_new/pruned_skel/p_skel"+str(m)+".jpg", skeleton) 
 

  # # cv2.circle(skeleton, (int(closest_base[0]), int(
  # #       closest_base[1])), 5, [255, 0, 0], -1)
  # # cv2.circle(skeleton, (int(closest_ee[0]), int(
  # #           closest_ee[1])), 5, [0, 0, 255], -1) 
  # # cv2.circle(skeleton, (int(base_center[0]), int(
  # #       base_center[1])), 5, [255, 255, 0], -1)
  # # cv2.circle(skeleton, (int(ee_center[0]), int(
  # #           ee_center[1])), 5, [0, 255, 0], -1) 
  
  
  # pixels = np.argwhere(skeleton == 255)
  # pixels = np.flip(pixels) 
  # copy_pix = pixels
  # init_point = pixels[0]
  # row_index, = np.where(np.all(copy_pix == (init_point), axis=1))
  # copy_pix = np.delete(copy_pix, row_index, axis=0)
  # # print(copy_pix)
  # ordered = init_point
  # while len(copy_pix) > 0:
  # # while init_point.all != exit_point.all:
  #     # print("Init_point first",init_point)
  #     distances = np.sqrt((copy_pix[:, 0] - init_point[0]) ** 2 + (copy_pix[:, 1] - init_point[1]) ** 2)
  #     # if len(distances) == 0:
  #     #   break
  #     nearest_index = np.argmin(distances)
  #     init_point = copy_pix[nearest_index]  
  #     # print("init_point updated", init_point)
  #     copy_pix = np.delete(copy_pix, nearest_index, axis=0)
  #     ordered = np.append(ordered, [init_point])
  # ordered = np.reshape(ordered, (-1, 2))
  # pixels = ordered.tolist()
  
  # start_index = pixels.index([init_point[0], init_point[1]])
  # end_index = pixels.index([closest_ee[0], closest_ee[1]])

  # # start_index = pixels.index([closest_base[0], closest_base[1]])
  # # print("start index frame" + str(m), start_index)
  # # end_index = pixels.index([exit_point[0], exit_point[1]])
  # # print("end index frame" + str(m), end_index)
  # # print("Ordered Pixels", pixels)
  # pixels = pixels[start_index:end_index]
  # # print("Sliced Pixels", pixels)

  # for i in range(len(pixels)):
  #   x = int(pixels[i][0])
  #   y = int(pixels[i][1])
  #   cv2.circle(skeleton,(x, y),5,(0,255,255),-1)

  
  # # cv2.imwrite("/home/merlab/Pictures/test_skel_new/final_skel/f_skel"+str(m)+".jpg", skeleton) 

  # # print("pixels", len(pixels))
  
  # # points_included =  pixels[index_base:index_ee] 

  # # for i in range(len(points_included)):
  # #   skeleton[points_included[i][1], points_included[i][0]] = (0, 0, 255)

  

  
  # pixels = np.array(pixels)
  # xinit = pixels[:,0]
  # yinit = pixels[:,1]

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

  # for i in range(len(cx)):
  #       x = np.int(cx[i])
  #       y = np.int(cy[i])
  #       cv2.circle(cv_binary,(x, y),5,(0,255,255),-1)
  
  # cv2.imwrite("/home/merlab/Pictures/test_skel_new/control_points_on_depth/cp"+str(m)+".jpg", cv_binary) 
  
  # # List of control points
  # # putting 1ist control points at the end as it is not changing and can be discarded
  # cp = []
  # for i in range(len(cx)-1):
  #   cp.append(cx[i+1]) 
  #   cp.append(cy[i+1])
 
  m +=1
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
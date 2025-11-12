#!/usr/bin/env python3
from os import remove
import numpy as np
import sys
from scipy.interpolate import splprep, splev
import rospy
import cv2
import time
from rospy.topics import Subscriber
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from skimage.morphology import skeletonize, thin, remove_small_objects
from skimage.segmentation import clear_border
# from encoderless_vs.srv import bin_img, bin_imgResponse

def binary_image_service():

  # depth_img = cv2.imread('/home/jani/.ros/depth1.jpg')

  depth_img = cv2.imread('/home/janch-ros/Documents/video_images/frame0009.jpg')

  depth_img = cv2.cvtColor(depth_img, cv2.COLOR_BGR2GRAY)

  cv2.imshow("Depth Window", depth_img)
  cv2.waitKey(200)

  depth_img[0:300, 0:300] = 0
  depth_img[0:80, 230:230+370] = 0
  depth_img[0:480, 550:550+300] = 0
  depth_img[240:240+240, 0:848] = 0

  depth_img = np.where(depth_img < 1, 215, depth_img)

  # r = cv2.selectROI(depth_img)
  # print(r[0], r[1], r[2], r[3])
  # depth_roi = depth_img[r[1]:r[1]+r[3], r[0]:r[0]+r[2]]
  # print(np.max(depth_roi))

  # depth_roi = depth_img[100:100+150, 270: 270+300]
  
  # depth_img = np.where(depth_img < 1, 30, depth_img)
  # depth_img = np.where(depth_img == 55, 0, depth_img)

  cv2.imshow("Depth Window New", depth_img)
  cv2.waitKey(200)  

  
  depth_img = np.where(depth_img <=43, 255, depth_img)
  depth_img = np.where(depth_img != 255, 0, depth_img)

  cv_binary = depth_img

  kernel = np.ones((5,5),np.uint8)
  depth_img = cv2.GaussianBlur(depth_img, (5,5), 0)
  depth_img = cv2.erode(depth_img, kernel, iterations = 5)
  ret3,cv_binary = cv2.threshold(depth_img,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
  cv_binary = cv2.morphologyEx(cv_binary, cv2.MORPH_OPEN, kernel)
  # cv_binary = cv2.bitwise_not(cv_binary)

  # depth_img = np.where(depth_img <= 10, 255, depth_img)
  # depth_img = np.where(depth_img != 255, 0, depth_img)

  # kernel = np.ones((5, 5), np.uint8)
  # cv_binary = cv2.morphologyEx(cv_binary, cv2.MORPH_CLOSE, kernel)
  # # cv_binary = cv2.erode(cv_binary,kernel,iterations = 3)
  # cv_binary = cv2.morphologyEx(cv_binary, cv2.MORPH_OPEN, kernel)

  cv2.imshow("CV Binary", cv_binary)
  cv2.waitKey(200)

  
  binary = np.where(cv_binary > 0, 1, cv_binary)    
  skeleton = skeletonize(binary, method='lee')    
  skeleton = np.where(skeleton == 1, 255, skeleton)

  new_skel = cv2.cvtColor(skeleton,cv2.COLOR_GRAY2RGB)

  pixels = np.argwhere(skeleton == 255)

  xinit = pixels[:, 1]
  yinit = pixels[:, 0] 
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
  
  points = np.c_[x, y]
  # new_img = np.zeros((480, 848, 3), dtype = "uint8")

  # for i in range(len(points)):
  #   x = np.int(points[i][0])
  #   y = np.int(points[i][1])
  #   cv2.circle(new_img,(x, y),2,(255,0,0),-1) 
  
  tck, u_params = splprep([x, y], k =2, s = 1)

  u = np.linspace(0,1,10)
  newPts = splev(u, tck)
  x = newPts[0]
  y = newPts[1]  
   
  points = np.c_[x,y]

  for i in range(len(points)):
    x = np.int(points[i][0])
    y = np.int(points[i][1])
    cv2.circle(new_skel,(x, y),3,(0,0,255),-1)

  # final_img = cv2.addWeighted(skeleton, 0.5, new_img, 0.5, 0)
  
  cv2.imshow("eval Points", new_skel)
  cv2.waitKey(10000)   

def main(args):  
  # Initialize ROS
  rospy.init_node('image_segmentation')  
  binary_image_service()

  rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
    


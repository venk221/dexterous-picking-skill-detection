#!/usr/bin/env python3
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
from skimage.morphology import skeletonize, thin
from encoderless_vs.srv import bin_img, bin_imgResponse

# Define CvBridge for ROS
# bridge = CvBridge()
# # image to be used in binary service
# current_ros_image = None
# # flag to control the control point service request
# cp_flag = False

def binary_image_service():
  # global bridge

  depth_img = cv2.imread('/home/jani/.ros/depth5.jpg')

  depth_img = cv2.cvtColor(depth_img, cv2.COLOR_BGR2GRAY)

  # print(np.max(depth_img))  

  # Convert ROS image to cv image
  # try:
  # current_cv_image = bridge.imgmsg_to_cv2(current_ros_image, 'passthrough')    
  # depth_img = np.array(current_cv_image, dtype=np.float32)
  # cv2.normalize(depth_img, depth_img, 0, 1, cv2.NORM_MINMAX)

  # depth_img = abs(depth_img - 1)

  cv2.imshow("Depth Window", depth_img)
  cv2.waitKey(5000)

  # r = cv2.selectROI(depth_img)
  # print(r[0], r[1], r[2], r[3])

  # depth_img[0:500, 0:300] = 0
  # depth_img[250: 250+230, 300:300+320] = 0 
  # depth_img[0:480, 500:500+350] = 0
  # depth_img[0:80, 200:850] = 0
  # depth_img[80:80+400, 570:570+300] = 0
  # depth_img[270:270+200, 250:250+360] = 0  

  depth_img[0:300, 0:300] = 0
  depth_img[0:80, 230:230+370] = 0
  depth_img[0:480, 550:550+300] = 0
  depth_img[240:240+240, 0:848] = 0


  # depth_roi = depth_img[100:100+150, 270: 270+300]
  
  depth_img = np.where(depth_img < 10, 170, depth_img)

  cv2.imshow("Depth Window New", depth_img)
  cv2.waitKey(2000)

  # ret, depth_img = cv2.threshold(depth_img, 20, 170, cv2.THRESH_BINARY)

  # r = cv2.selectROI(depth_img)
  # print(r[0], r[1], r[2], r[3])

  # imgCrop = depth_img[int(r[1]):int(r[1]+r[3]), int(r[0]):int(r[0]+r[2])]

  # print(np.max(imgCrop))
  # print(np.min(imgCrop))



  # kernel = np.ones((5,5),np.uint8)
  # # depth_img = cv2.medianBlur(depth_img, 5)
  # depth_img = cv2.GaussianBlur(depth_img, (5,5), 0)
  # cv2.adaptiveThreshold(depth_img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,21,4)

  # depth_img = np.where(depth_img == 0, 55, depth_img)

  # cv2.imshow("Depth Window New", depth_img)
  # cv2.waitKey(5000)

  # depth_img = np.where(depth_img < 100, 255, depth_img)

  # cv2.imshow("Depth Window 2", depth_img)
  # cv2.waitKey(5000)
  

  kernel = np.ones((5,5),np.uint8)
  depth_img = cv2.erode(depth_img, kernel, iterations = 5)
  ret3,cv_binary = cv2.threshold(depth_img,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
  cv_binary = cv2.bitwise_not(cv_binary)

  # cv_binary = cv2.bitwise_not(cv_binary)
  # cv_binary = cv2.erode(cv_binary,kernel,iterations = 5)

  # r = cv2.selectROI(cv_binary)
  # print(r[0], r[1], r[2], r[3])

  # cv_binary = thin(cv_binary)

  cv2.imshow("CV Binary Old", cv_binary)
  cv2.waitKey(2000)

  # for index, pixels in enumerate(cv_binary):
  #   # print("Index", index, "Pixels", pixels)
  #   # if index <=500 and index >=270:
  #   if index >=270 or index <=90:
  #     cv_binary[index] = np.where(cv_binary[index]==255, 0, cv_binary[index])
  #   if index < 270:
  #     for i in range(len(cv_binary[index])):
  #       # print(cv_binary[index][i])
  #       if i <= 280 or i >= 550:
  #         cv_binary[index][i] = 0 

  # cv_binary = cv2.erode(cv_binary,kernel,iterations = 1)
  

  # cv2.imshow("CV Binary New", cv_binary)
  # cv2.waitKey(5000)

  
  binary = np.where(cv_binary > 0, 1, cv_binary)    
  skeleton = skeletonize(binary, method='lee')    
  skeleton = np.where(skeleton == 1, 255, skeleton)
  print(skeleton)

  # cv2.imshow("skeleton", skeleton)
  # cv2.waitKey(5000)

  pixels = np.argwhere(skeleton == 255)
  print(pixels)
  x= pixels[-1][1]
  y = pixels[-1][0]

  img = cv2.circle(skeleton, (x, y), radius=10, color=(255, 255, 255), thickness=1)

  print(x, y)

  cv2.imshow("skeleton", img)
  cv2.waitKey(2000)

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
  new_img = np.zeros((1000, 1000, 3), dtype = "uint8")

  print([x, y])

  # for i in range(len(points)):
  #   x = np.int(points[i][0])
  #   y = np.int(points[i][1])
  #   cv2.circle(new_img,(x, y),2,(255,0,0),-1) 
  
  cv2.imshow("Data Points", new_img)
  cv2.waitKey(2000)
 
  tck, u_params = splprep([x, y], k =2, s = 1)

  u = np.linspace(0,1,10)
  newPts = splev(u, tck)
  x = newPts[0]
  y = newPts[1]  
   
  points = np.c_[x,y]

  for i in range(len(points)):
    x = np.int(points[i][0])
    y = np.int(points[i][1])
    cv2.circle(new_img,(x, y),2,(0,0,255),-1)
  
  cv2.imshow("eval Points", new_img)
  cv2.waitKey(5000)

  


  
  # cv_binary = cv_binary.tolist()
  # textfile = open("/home/jani/Documents/a_file.txt", "w")
  # for element in cv_binary:
  #   # print(element)
  #   element = ' '.join(map(str, element))
  #   # print(element)
  #   textfile.write(element + "\n")
  # textfile.close()

  # cv_binary = cv_binary*255
  # cv_binary = np.where(cv_binary>200, 255, cv_binary)
  # cv_binary = np.where(cv_binary<200, 0, cv_binary)



  # cv_binary = np.array(cv_binary, dtype=np.uint8)  

  # print(cv_binary)

  # print(cv_binary.shape)
  # cv_binary = cv2.bitwise_not(cv_binary)
  # ret3,cv_binary = cv2.threshold(cv_binary,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
  # cv2.normalize(cv_binary, cv_binary, 0, 1, cv2.NORM_MINMAX)
  # cv_binary = np.array(cv_binary, dtype=np.float32)  

  # binary_image = bridge.cv2_to_imgmsg(cv_binary, "mono8")
  
  # return bin_imgResponse(binary_image)

# def image_callback(ros_image):
#   global current_ros_image, cp_flag
#   current_ros_image = ros_image
#   if current_ros_image is not None:    
#     cp_flag = True
#   else:
#     print("No Image Found")

def main(args):  
  # Initialize ROS
  rospy.init_node('image_segmentation')  
  binary_image_service()

  # Declare subcscribers
  # cam sub
  # image_sub = rospy.Subscriber("/camera/depth/image_rect_raw",Image,image_callback,queue_size = 1)

  # service declaration to receive the binary image
  # bin_img_service = rospy.Service("binary_image_output", bin_img, binary_image_service )
  
  # publisher to publish flag to start control points svc
  # flag_pub = rospy.Publisher("/vsbot/control_flag", Bool, queue_size = 1)

  # while not rospy.is_shutdown():
    # flag_pub.publish(cp_flag)

  rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
    


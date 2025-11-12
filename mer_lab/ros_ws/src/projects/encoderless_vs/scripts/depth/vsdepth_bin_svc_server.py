#!/usr/bin/env python3
import numpy as np
import sys
import rospy
import cv2
import time
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from encoderless_vs.srv import bin_img, bin_imgResponse

# Define CvBridge for ROS
bridge = CvBridge()
# image to be used in binary service
current_ros_image = None
# flag to control the control point service request
cp_flag = False

def binary_image_service(msg):
  global bridge

  # Convert ROS image to cv image
  # try:
  current_cv_image = bridge.imgmsg_to_cv2(current_ros_image, '32FC1')    
  depth_img = np.array(current_cv_image, dtype=np.float32)
  depth_img = np.array(depth_img, dtype=np.uint8)
  cv2.normalize(depth_img, depth_img, 0, 1, cv2.NORM_MINMAX)

  depth_img = abs(depth_img - 1)

  cv2.imshow("Depth Window", depth_img)
  cv2.waitKey(20)

  kernel = np.ones((5,5),np.uint8)
  gray_img = cv2.morphologyEx(depth_img, cv2.MORPH_CLOSE, kernel)
  cv_binary = cv2.erode(gray_img,kernel,iterations = 1)

  cv_binary = cv_binary*255
  cv_binary = np.where(cv_binary>200, 255, cv_binary)
  cv_binary = np.where(cv_binary<200, 0, cv_binary)

  cv2.imshow("CV Binary", cv_binary)
  cv2.waitKey(20)

  cv_binary = np.array(cv_binary, dtype=np.uint8)  

  # cv_binary = cv_binary.tolist()
  # textfile = open("a_file.txt", "w")
  # for element in cv_binary:
  #   print(element)
  #   element = ' '.join(map(str, element))
  #   print(element)
  #   textfile.write(element + "\n")
  # textfile.close()

  # print(cv_binary)

  # print(cv_binary.shape)
  # cv_binary = cv2.bitwise_not(cv_binary)
  # ret3,cv_binary = cv2.threshold(cv_binary,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
  # cv2.normalize(cv_binary, cv_binary, 0, 1, cv2.NORM_MINMAX)
  # cv_binary = np.array(cv_binary, dtype=np.float32)  

  binary_image = bridge.cv2_to_imgmsg(cv_binary, "mono8")
  
  return bin_imgResponse(binary_image)

def image_callback(ros_image):
  global current_ros_image, cp_flag
  current_ros_image = ros_image
  if current_ros_image is not None:    
    cp_flag = True
  else:
    print("No Image Found")

def main(args):  
  # Initialize ROS
  rospy.init_node('image_segmentation')  
  # Declare subcscribers
  # cam sub
  image_sub = rospy.Subscriber("/camera/depth/image_rect_raw",Image,image_callback,queue_size = 1)

  # service declaration to receive the binary image
  bin_img_service = rospy.Service("binary_image_output", bin_img, binary_image_service )
  
  # publisher to publish flag to start control points svc
  flag_pub = rospy.Publisher("/vsbot/control_flag", Bool, queue_size = 1)

  while not rospy.is_shutdown():
    flag_pub.publish(cp_flag)

  rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
    


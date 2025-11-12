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

def binary_image_service(msg):
  global bridge

  current_cv_image = bridge.imgmsg_to_cv2(current_ros_image, "bgr8")  

  cv2.imwrite("/home/janch-ros/Documents/pics/robot1.jpg", current_cv_image)

  # gray_img=cv2.cvtColor(current_cv_image,cv2.COLOR_BGR2GRAY)
  
  # cv2.imwrite("/home/janch-ros/Documents/pics/gray1.jpg", gray_img)

  # r = cv2.selectROI(depth_img)
  # print(r[0], r[1], r[2], r[3])

  # depth_roi = depth_img[r[1]:r[1]+r[3], r[0]:r[0]+r[2]]

# print(np.max(depth_roi), np.min(depth_roi))

  # blur = cv2.GaussianBlur(gray_img,(5,5),0)
  # ret3,cv_binary = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

  # thresh, cv_binary = cv2.threshold(blur, 127, 255, cv2.THRESH_BINARY)



  # cv_binary = cv2.bitwise_not(cv_binary)

  # cv2.imshow("CV Depth Image", cv_binary)
  # cv2.waitKey(200)

  hsvimg = cv2.cvtColor(current_cv_image, cv2.COLOR_BGR2HSV)
  # Define image color bounds 
  orange_lower = np.array([10, 100, 20], np.uint8) 
  orange_upper = np.array([25, 255, 255], np.uint8) 
  # orange_lower = np.array([5, 50, 50], np.uint8) 
  # orange_upper = np.array([15, 255, 255], np.uint8) 
  white_lower = np.array([0, 0, 200], np.uint8)
  white_upper = np.array([145, 60, 255], np.uint8) 
  blue_lower = np.array([78,158,124])
  blue_upper = np.array([138,255,255])
  # blue_lower = np.array([110,158,124])
  # blue_upper = np.array([130,255,255])

  # Binarizing individual colors & combining
  orange_mask = cv2.inRange(hsvimg, orange_lower, orange_upper)
  blue_mask = cv2.inRange(hsvimg, blue_lower, blue_upper)
  white_mask = cv2.inRange(hsvimg, white_lower, white_upper)
		
  cv_binary = orange_mask + blue_mask + white_mask

  cv2.imwrite("/home/janch-ros/Documents/pics/binary1.jpg", cv_binary)

  binary_image = bridge.cv2_to_imgmsg(cv_binary, "mono8")
  return bin_imgResponse(binary_image)  

def read_saved_image():
  global current_ros_image
  img_read_tstart = time.time()

  # read a saved image from file as a cv image
  input_image = cv2.imread('/home/janch-ros/frame0003.jpg')  

  # convert the cv_image to ROS image
  current_ros_image = bridge.cv2_to_imgmsg(input_image, "bgr8")
  print(type(current_ros_image))
  

def main(args):  
  # Initialize ROS
  rospy.init_node('image_segmentation')  

  # call the function to read the image from the file
  read_saved_image()

  # service declaration to receive the binary image
  bin_img_service = rospy.Service("binary_image_output", bin_img, binary_image_service )

  rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
    


#!/usr/bin/env python3

import numpy as np
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Define CvBridge for ROS
bridge = CvBridge()
# image to be published
binary_image = None

def image_callback(ros_image):
    global bridge, current_image, binary_image
    # Convert ROS image to cv image
    # try:
    current_cv_image = bridge.imgmsg_to_cv2(ros_image, 'bgr8')

    cv2.imwrite("/home/janch-ros/Documents/pics/robot.jpg", current_cv_image)

    # cv2.imshow("Robot", current_cv_image)
    # cv2.waitKey(20)
    gray_img=cv2.cvtColor(current_cv_image,cv2.COLOR_BGR2GRAY)
  

    blur = cv2.GaussianBlur(gray_img,(5,5),0)
    ret3,cv_binary = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

    new_cv = cv2.cvtColor(cv_binary,cv2.COLOR_GRAY2BGR)

    cv2.imwrite("/home/janch-ros/Documents/pics/binary.jpg", cv_binary)

    dst = cv2.addWeighted(current_cv_image,0.5,new_cv,0.7,0)

    cv2.imwrite("/home/janch-ros/Documents/pics/overlay.jpg", dst)

    
    binary_image = bridge.cv2_to_imgmsg(cv_binary, "mono8")
    # depth_img = np.array(current_cv_image, dtype=np.float32)
    # cv2.normalize(depth_img, depth_img, 0, 1, cv2.NORM_MINMAX)

  #   depth_img = abs(depth_img - 1)
  # # depth_img = depth_img*255
  #   cv2.imshow("Depth Window", depth_img)
  #   cv2.waitKey(200)

    # kernel = np.ones((5,5),np.uint8)
    # gray_img = cv2.morphologyEx(depth_img, cv2.MORPH_CLOSE, kernel)
    # cv_binary = cv2.erode(gray_img,kernel,iterations = 1)

    # cv_binary = cv_binary*255

    # cv_binary = np.where(cv_binary>200, 255, cv_binary)
    # cv_binary = np.where(cv_binary<200, 0, cv_binary)

      
    # cv_binary = np.array(cv_binary, dtype=np.uint8)
    # print("Test the image sub", len(cv_image_capture))
    # # except CvBridgeError as e:
    #   # print(e)
 
    # # converting to hsv
    # hsvimg = cv2.cvtColor(cv_image_capture, cv2.COLOR_BGR2HSV)

    # # Define image color bounds 
    # orange_lower = np.array([10, 100, 20], np.uint8) 
    # orange_upper = np.array([25, 255, 255], np.uint8) 
    # white_lower = np.array([0, 0, 200], np.uint8)
    # white_upper = np.array([145, 60, 255], np.uint8) 
    # blue_lower = np.array([78,158,124])
    # blue_upper = np.array([138,255,255])
	  
    # # Binarizing individual colors & combining
    # orange_mask = cv2.inRange(hsvimg, orange_lower, orange_upper)
    # blue_mask = cv2.inRange(hsvimg, blue_lower, blue_upper)
    # white_mask = cv2.inRange(hsvimg, white_lower, white_upper)
	  	
    # cv_binary = orange_mask + blue_mask + white_mask

    # # Convert cv image to ROS image
    # # try:
    binary_image = bridge.cv2_to_imgmsg(cv_binary, "mono8")
    # # except CvBridgeError as e:
    #   # print(e)    

    # current_image = ros_image
       

def main(args):
  # Initialize ROS
  rospy.init_node('image_segmentation')
  
  # Declare subcscribers
  # cam sub
  image_sub = rospy.Subscriber("/vsbot/camera1/image_raw",Image,image_callback,queue_size =1)
       
  # Declare Publishers
  # binary img pub
  binary_img_pub = rospy.Publisher("/vsbot/binary_image",Image,queue_size=1)
  
  # publishing latest binary image at 30Hz
  r = 30 #rospy.get_param('vsbot_depth/estimation/rate')
  rate = rospy.Rate(r)
  while not rospy.is_shutdown():
    # Start publishing after first callback
    if binary_image != None:
      binary_img_pub.publish(binary_image)
    # Sleeps and refreshes subscriber
    rate.sleep()

if __name__ == '__main__':
    main(sys.argv)
    


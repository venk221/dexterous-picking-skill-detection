#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from encoderless_vs.srv import cp_goal, cp_goalResponse
from scipy.interpolate import splev
from cv_bridge import CvBridge, CvBridgeError

def main():
    # initialize the node
    rospy.init_node('goal_image_publish_node')
    bridge = CvBridge()
    
    # cv_goal_img = cv2.imread("franka_published_goal_image.jpg")
    cv_goal_img = cv2.imread("dream_published_goal_image.jpg")
    
    published_image = bridge.cv2_to_imgmsg(cv_goal_img, "bgr8")

    
    # Declaring the publisher for image
    goal_image_pub = rospy.Publisher('franka/goal_image', Image, queue_size= 1)
    
    # assigning a rate at which the image will be published
    r = rospy.get_param('vsbot/estimation/rate')
    rate = rospy.Rate(r)
  
    while not rospy.is_shutdown():
      # Publish goal image at the loop rate
      goal_image_pub.publish(published_image)
      rate.sleep()  
         

if __name__ == '__main__':
    main()

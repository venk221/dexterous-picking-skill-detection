#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from scipy.interpolate import splev
from cv_bridge import CvBridge, CvBridgeError

def main():
    # initialize the node
    rospy.init_node('goal_image_publish_node')
    bridge = CvBridge()
    cv_goal_img = cv2.imread("dl_published_goal_image.jpg")
    # cv_goal_img_1 = cv2.imread("/home/merlab/.ros/franka_published_goal_image_pick.jpg")
    # cv_goal_img_2 = cv2.imread("/home/merlab/.ros/franka_published_goal_image_im.jpg")
    # cv_goal_img_3 = cv2.imread("/home/merlab/.ros/franka_published_goal_image_place.jpg")
    # cv_goal_img_4 = cv2.imread("/home/merlab/.ros/franka_published_goal_image_final.jpg")

    # cv_overlay_1 = cv2.addWeighted(cv_goal_img_1, 1.0, cv_goal_img_2, 1.0, 0)
    # cv_overlay_2 = cv2.addWeighted(cv_goal_img_3, 1.0, cv_goal_img_4, 1.0, 0)
    # cv_overlay = cv2.addWeighted(cv_overlay_1, 1.0, cv_overlay_2, 1.0, 0)

    # cv2.imwrite("/home/merlab/Pictures/cv_overlay_1.jpg", cv_overlay_1)
    # cv2.imwrite("/home/merlab/Pictures/cv_overlay_2.jpg", cv_overlay_2)
    # cv2.imwrite("/home/merlab/.ros/cv_overlay.jpg", cv_overlay)

    published_image = bridge.cv2_to_imgmsg(cv_goal_img, "bgr8")

    
    # Declaring the publisher for image
    goal_image_pub = rospy.Publisher('franka/dl_goal_image', Image, queue_size= 1)
    
    # assigning a rate at which the image will be published
    r = rospy.get_param('vsbot/estimation/rate')
    rate = rospy.Rate(r)
  
    while not rospy.is_shutdown():
      # Publish goal image at the loop rate
      goal_image_pub.publish(published_image)
      rate.sleep()  
         

if __name__ == '__main__':
    main()

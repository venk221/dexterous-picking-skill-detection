#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
bridge= CvBridge()

feature_pub = rospy.Publisher("two_link_vs/features", Float64MultiArray, queue_size=1)

def feature_detect(img):
    global bridge

    cur_img = bridge.imgmsg_to_cv2(img, "bgr8")
    hsvimg = cv2.cvtColor(cur_img, cv2.COLOR_BGR2HSV)

    blue_lower = np.array([94, 80, 2], np.uint8) 
    blue_upper = np.array([120, 255, 255], np.uint8) 
    blue_mask = cv2.inRange(hsvimg, blue_lower, blue_upper)
    
    kernel = np.ones((5, 5), "uint8")
    blue_mask_contour = cv2.dilate(blue_mask, kernel)

    # white_lower = np.array([0, 0, 200], np.uint8)
    # white_upper = np.array([145, 60, 255], np.uint8)   
    # white_mask = cv2.inRange(hsvimg, white_lower, white_upper)

    # # Blue segment converted to binary
    # mask1 = blue_mask + white_mask

    # # Points on blue marker contour
    # pixels = np.argwhere(mask1 > 0)

    # x1 = pixels[0][1]
    # x2 = pixels[-1][1]

    # y1 = pixels[0][0]
    # y2 = pixels[-1][0]

    # #calculating the center by averaging the pixels
    # cX = int((x1+x2)/2)
    # cY = int((y1+y2)/2)
    
    contours, hierarchy = cv2.findContours(blue_mask_contour, 
	                                        cv2.RETR_TREE, 
	                                        cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        M = cv2.moments(contour)
        cX = int(M["m10"]/M["m00"])
        cY = int(M["m01"]/M["m00"])
    
    center = Float64MultiArray()
    center.data = [cX, cY]

    feature_pub.publish(center)
    
def main():
    rospy.init_node('marker_track')

    image_sub = rospy.Subscriber("two_link/camera1/image_raw", Image, feature_detect, queue_size=1)
    
    rospy.spin()

if __name__ == "__main__":
    main()
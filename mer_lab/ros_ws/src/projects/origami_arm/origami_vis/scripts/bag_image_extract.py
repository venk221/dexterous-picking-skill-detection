#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco
from std_msgs.msg import Float64MultiArray
import sys
import math

i = 0
cv_img = None
bridge = CvBridge()

def writeImg(msg):
    global i, cv_img

    cv_img = bridge.imgmsg_to_cv2(msg, 'bgr8')
    cv2.imwrite(str(i)+".jpeg", cv_img)

    i = i+1



def main():
    #ROS stuff
    rospy.init_node('write_raw_imgs')

    # Subscribe to topic(s)
    raw_img_sub = rospy.Subscriber("raw_image", Image, writeImg, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    main()
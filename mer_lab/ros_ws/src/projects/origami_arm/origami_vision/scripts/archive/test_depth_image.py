#!/usr/bin/env python3

from logging import shutdown
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from skimage.morphology import skeletonize
from scipy.interpolate import splprep, splev
import math

bridge = CvBridge()
cur_depth_img = None

def getDepthImage(msg):
    global bridge, cur_depth_img
    cur_depth_img = bridge.imgmsg_to_cv2(msg, "8UC1")

def main():
    rospy.init_node('depth_image_writer')

    depth_img_sub = rospy.Subscriber("camera/aligned_depth_to_color/image_raw", Image, getDepthImage, queue_size = 1)
    
    itr = 0
    rospy.sleep(5)
    rate = rospy.Rate(15)
    while not rospy.is_shutdown():
        if cur_depth_img is not None:
            cv2.imwrite(str(itr)+".jpg",cur_depth_img )
            print(itr)
            itr += 1
        else:
            print("no image")
        rate.sleep()
    rospy.spin()
if __name__ == "__main__":
    main()

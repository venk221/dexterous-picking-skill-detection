#!/usr/bin/env python3

import rospy
# import numpy as np
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco
from std_msgs.msg import Float32MultiArray
import sys

bridge = CvBridge()
ros_img = None
ee_center = []
base_center = []
base_index = None
ee_index = None
marker_base_flag = False
marker_ee_flag = False
ee_corners_list = []
base_corners_list = []

# Subscriber callback
def marker_pose(img_msg):
    global bridge, ros_img

    cv_img = bridge.imgmsg_to_cv2(img_msg, 'bgr8')
    
    if cv_img is not None:
        ros_img = bridge.cv2_to_imgmsg(cv_img, 'bgr8')


def main(args):
    # Initialize ROS
    rospy.init_node('marker_detect')

    # Listening to the start flag

    # Subscribers
    image_rgb_sub = rospy.Subscriber("/camera/color/image_raw", Image, marker_pose, queue_size=1)
    # image_depth_sub = rospy.Subscriber("/camera/color/image_raw", Image, marker_pose, queue_size=1)

    # Publishers
    rgb_pub = rospy.Publisher("published/rgb/image", Image, queue_size=1)
    

    # Rate Loop to publish
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        # Publish image
        if ros_img is not None:
            # rospy.sleep(5)
            rgb_pub.publish(ros_img)

        rate.sleep()

    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)

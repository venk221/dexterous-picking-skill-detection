#!/usr/bin/env python3

# This node saves depth images to file

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool

bridge = CvBridge()
it = 0

def imgWriter(img_msg):
    global bridge, it

    # Convert to cv image
    img = bridge.imgmsg_to_cv2(img_msg, "8UC1")
    
    # Write image
    filename = str(it) + ".png"
    cv2.imwrite(filename,img)

    # Increment iterator for image name
    it = it+1


def main():
    # Initialize ROS
    rospy.init_node('depth_image_writer')
    print("Initialized depth image writer")
    
    # Wait for the camera
    rospy.sleep(20)
    
    # Subscribers
    depth_img_sub = rospy.Subscriber("camera/aligned_depth_to_color/image_raw", Image, imgWriter, queue_size = 1)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == "__main__":
    main()
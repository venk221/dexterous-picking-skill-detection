#!/usr/bin/env python3

from logging import shutdown
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()


def main():
    rospy.init_node('ref_pub')
    ref_publisher = rospy.Publisher("origami_vs/ref_img", Image, queue_size=1)
    
    # read rate
    rate = rospy.get_param("origami_skeleton_vs/control_rate")
    r = rospy.Rate(rate)
    # read image
    ref_img = cv2.imread("ref_img.jpg")
    
    # convert img to msg
    ref_img = bridge.cv2_to_imgmsg(ref_img, "bgr8")
    
    # publish image
    while not rospy.is_shutdown():
        ref_publisher.publish(ref_img)
        r.sleep()
    rospy.spin()


if __name__ == "__main__":
    main()
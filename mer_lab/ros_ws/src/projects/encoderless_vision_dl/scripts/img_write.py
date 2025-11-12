#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

bridge = CvBridge()
status = -2
itr = 0

def statusCallback(msg):
    global status
    status = msg.data

def imgSave(msg):
    global itr
    # print("status right now",status)
    if status > 0:
        # Write imgs to file
        try:
            cv_img = bridge.imgmsg_to_cv2(msg,"bgr8")
        except CvBridgeError as e:
            print(e)
        filename = "raw/" + str(itr) + ".png"
        cv2.imwrite(filename, cv_img)
        itr += 1


def main(args):
    
    # Initialize ROS
    rospy.init_node('img_writer')

    # Initialize subscribers
    img_sub = rospy.Subscriber("camera/color/image_raw", Image, imgSave, queue_size = 1)
    status_sub = rospy.Subscriber("vsbot/status", Int32, statusCallback, queue_size = 1)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == "__main__":
    main(sys.argv)


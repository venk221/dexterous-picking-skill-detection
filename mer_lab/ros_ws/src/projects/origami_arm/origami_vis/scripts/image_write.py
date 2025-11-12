#!/usr/bin/env python3
import roslib
import sys
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
bridge = CvBridge()

# This node writes images to file
itr = 0
def imgSave(msg):
    global itr
    print("img callback")
    # Write imgs to file
    try:
        cv_img = bridge.imgmsg_to_cv2(msg,"bgr8")
    except CvBridgeError as e:
        print(e)
    filename = str(itr) + ".jpeg"
    cv2.imwrite(filename, cv_img)
    itr += 1

def main():
    # Initialize ROS
    rospy.init_node('depth_writer')
    rospy.sleep(10)
    # Initialize subscribers
    img_sub = rospy.Subscriber("camera/color/image_raw", Image, imgSave, queue_size = 1)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main()
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaver:
    def __init__(self):
        rospy.init_node('image_saver', anonymous=True)

        self.bridge = CvBridge()

        # Set up subscribers for two image topics
        self.image_subscriber1 = rospy.Subscriber('grasp_detection/visualization', Image, self.image_callback1)
        self.image_subscriber2 = rospy.Subscriber('grasp_generation_results', Image, self.image_callback2)
        self.image_subscriber3= rospy.Subscriber('zed_camera/depth_image', Image, self.image_callback3)
        self.image_subscriber4 = rospy.Subscriber('zed_camera/image', Image, self.image_callback4)
        self.basePath = "/home/merlab/Documents/Graph scenes/scene"
        if not os.path.exists(self.basePath):
        # Create the directory
            os.makedirs(self.basePath)
        


    def image_callback1(self, data):
        self.save_image(data, self.basePath + "/grasp_detection_image.jpg")

    def image_callback2(self, data):
        self.save_image(data, self.basePath + "/grasp_gen_image.jpg")
        self.save_image(self.depth, self.basePath + "/depth_image.jpg")
        self.save_image(self.rgb, self.basePath + "/rgb_image.jpg")

    def image_callback3(self, data):
        self.depth = data

    def image_callback4(self, data):
        self.rgb = data


    def save_image(self, data, filename):
        if not os.path.exists(self.basePath):
        # Create the directory
            os.makedirs(self.basePath)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            rospy.logerr(e)
            return

        # Save the image
        cv2.imwrite(filename, cv_image)
        rospy.loginfo(f"Saved image: {filename}")

if __name__ == '__main__':
    image_saver = ImageSaver()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

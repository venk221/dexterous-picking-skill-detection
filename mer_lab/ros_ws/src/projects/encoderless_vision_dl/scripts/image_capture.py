#!/usr/bin/env python3.8

import cv2
import numpy as numpy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError
import sys
import rospy
import os
from os.path import expanduser
import glob

# Define CvBridge for ROS
home = expanduser("~")
bridge = CvBridge()
i = 0
size = None
status = -2

# latest changes

file_path = home + "/Pictures/" + "Data/"

rgb_path = file_path + 'video_images/rgb_01/'
dep_path = file_path + 'video_images/dep/'

# path = "/home/fearless/Pictures/video_images"s


def statusCallback(msg):
    global status
    status = msg.data


def depth_callback(dep):
    
    fname = dep_path + "image_" + str(i) + ".jpg"
    cv_img = bridge.imgmsg_to_cv2(dep, 'mono8')
  
    # cv2.imwrite(path+str(i)+'.jpg', cv_img)
    cv2.imwrite(fname, cv_img)
    i = i+1

def image_callback(img):
    global bridge, i, home, size, status

    # if status == -2:
    # print("is image call back getting called")
    # convert ros images to numpy/cv images
    fname = rgb_path + "/image_" + str(i) + ".jpg"
    cv_img = bridge.imgmsg_to_cv2(img, 'bgr8')

    # print(cv_img)
    # print(fname)
    # cv2.imwrite(path+str(i)+'.jpg', cv_img)
    cv2.imwrite(fname, cv_img)
    i = i+1

    # elif status == 1:
    #     # stitch video frames
    #     img_array = []
    #     file_list = []
    #     for file in glob.glob("*.jpg"):
    #         file_tup = file.partition('.')
    #         file_list.append(int(file_tup[0]))
    #     file_list.sort()
    #     for i in file_list:
    #         img = cv2.imread(str(i)+".jpg")
    #         cv2.imwrite(file_path+"test_image.jpg", img)
    #         height, width, layers = img.shape
    #         print(height, width)
    #         size = (width, height)
    #         img_array.append(img)

    #     # for filename in sorted([e for e in path.iterdir() if e.is_file() and str(e).endswith(".jpg")]):
    #     # for filename in sorted(os.listdir(path)):
    #     #     print(filename)
    #     #     img = cv2.imread(str(filename))
    #     #     img_array.append(img)
    #     print(size)
    #     out = cv2.VideoWriter("exp_vid.avi",cv2.VideoWriter_fourcc(*'XVID'), 10, size)
    #     for i in range(len(img_array)):
    #         out.write(img_array[i])
    #     out.release()
    #     print("Video saved")


def main(args):
  # Initialize ROS
    rospy.init_node('capture_images_for_video')
    # Declare subcscribers
    # subscriber for rgb images
    image_rgb_sub = rospy.Subscriber("/camera/color/image_raw", Image, image_callback, queue_size=1)
    # image_dep_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, depth_callback, queue_size=1)

    # status subscriber
    # status_sub = rospy.Subscriber("vsbot/status", Int32, statusCallback, queue_size = 1)
    # while not rospy.is_shutdown():
    #     print("keep capturing")

    # publisher to publish flag to start control points svc
    # flag_pub = rospy.Publisher("/franka/control_flag", Bool, queue_size = 1)

    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)

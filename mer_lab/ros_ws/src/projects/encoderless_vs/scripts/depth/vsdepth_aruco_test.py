#!/usr/bin/env python3
import numpy as np
import sys
import matplotlib.pyplot as plt
import rospy
import cv2
import cv2.aruco as aruco
import pyrealsense2 as rs
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
current_ros_image = None
m = 0
j = 0
k = 0
# creating global variable for base and ee center x and y co-ordinates
# to be used in depth callback for alignment test
base_center_x = None
base_center_y = None
ee_center_x = None
ee_center_y = None
id_list = []
frame = None
ros_img = None


def rgb_image_callback(ros_image):
    global bridge, m, base_center_x, base_center_y, ee_center_x, ee_center_y, id_list, frame, ros_img
    current_ros_image = ros_image
    if current_ros_image is not None:
        print(True)
    else:
        print("No Image Found")

    print("current m", m)
    color_img = bridge.imgmsg_to_cv2(current_ros_image, 'bgr8')
    print(color_img.shape)

    # color_img = cv2.resize(color_img, (640, 480), interpolation = cv2.INTER_AREA)

    print(color_img.shape)

    gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

    arucoParameters = aruco.DetectorParameters_create()

    corners, ids, rejectedImgPoints = aruco.detectMarkers(
        gray, aruco_dict, parameters=arucoParameters)

    # cv2.imwrite('/home/jani/Documents/pics/aruco'+str(i)+'.jpg', frame)

    # ids = ids.tolist()

    id_list.clear()
    for i in ids:
        print(type(i))
        if int(i) == 32:
            id_list.append(int(i))
        elif int(i) == 34:
            id_list.append(int(i))

    # Aruco ID for each marker
    base_id = 34
    ee_id = 32
    # Extracting ids to list

    # id_list = [int(ids[0]), int(ids[1])]

    # Checking index of marker in the list
    base_index = id_list.index(base_id)
    ee_index = id_list.index(ee_id)
    # Separating the corner pixel co-ordinates for each marker
    base_corners_list = corners[base_index].reshape(
        4, 2)   # [[x1, y1],[x2, y2],[x3, y3],[x4, y4]]
    ee_corners_list = corners[ee_index].reshape(4, 2)
    # Averaging base corner co-ordinates to obtain marker center
    base_center_x = (base_corners_list[0][0] + base_corners_list[1]
                     [0] + base_corners_list[2][0] + base_corners_list[3][0])/4
    base_center_y = (base_corners_list[0][1] + base_corners_list[1]
                     [1] + base_corners_list[2][1] + base_corners_list[3][1])/4

    # computing the center for the base marker to restrict skeletonization
    base_center = [base_center_x, base_center_y]
    # Averaging ee corner co-ordinates to obtain marker center
    ee_center_x = (ee_corners_list[0][0] + ee_corners_list[1]
                   [0] + ee_corners_list[2][0] + ee_corners_list[3][0])/4
    ee_center_y = (ee_corners_list[0][1] + ee_corners_list[1]
                   [1] + ee_corners_list[2][1] + ee_corners_list[3][1])/4

    # computing the center for the ee marker to restrict skeletonization
    ee_center = [ee_center_x, ee_center_y]

    print("Base Center in RGB", int(base_center_x), int(base_center_y))
    print("EE Center in RGB", int(ee_center_x), int(ee_center_y))

    # drawing a rectangle around the marker using aruco libraries
    frame = aruco.drawDetectedMarkers(color_img, corners)

    # cv2.imwrite('/home/jani/Documents/pics/aruco'+str(i)+'.jpg', frame)

    # marking the centers in the images
    cv2.circle(frame, (int(base_center_x), int(
        base_center_y)), 6, [0, 255, 255], -1)
    cv2.circle(frame, (int(ee_center_x), int(
        ee_center_y)), 6, [255, 255, 0], -1)

    # cv2.imwrite(str(i)+'.tiff', frame)
    cv2.imwrite('/home/jani/Documents/pics1/cv-rgb'+str(m)+'.jpg', frame)

    m = m+1

    print("next m", m)

    # ros_img = bridge.cv2_to_imgmsg(frame, 'bgr8')


def depth_image_callback(ros_image):
    global j, k
    depth_img = bridge.imgmsg_to_cv2(ros_image, '8UC1')
    depth_img = cv2.cvtColor(depth_img, cv2.COLOR_GRAY2BGR)

    # depth_img = cv2.resize(depth_img, (640, 480), interpolation = cv2.INTER_AREA)
    print(depth_img.shape)

    print("Base Center in Depth", int(base_center_x), int(base_center_y))
    print("EE Center in Depth", int(ee_center_x), int(ee_center_y))

    cv2.circle(depth_img, (int(base_center_x), int(base_center_y)), 3, [0, 0, 255], -1)
    cv2.circle(depth_img, (int(ee_center_x), int(ee_center_y)), 3, [255, 0, 0], -1)

    cv2.imwrite('/home/jani/Documents/pics1/cv-depth'+str(j)+'.jpg', depth_img)

    # cv_img = cv2.addWeighted(depth_img, 0.5, frame, 0.5, 0)

    # j = j+1

    cv2.imwrite('/home/jani/Documents/pics2/aligned'+str(k)+'.jpg', depth_img)

    # k = k+1

    ros_img = bridge.cv2_to_imgmsg(depth_img, 'bgr8')


def main(args):
    # Initialize ROS
    rospy.init_node('depth_rgb_alignment_node')

    rgb_sub = rospy.Subscriber(
        "/camera/color/image_raw", Image, rgb_image_callback)
    depth_sub = rospy.Subscriber(
        "/camera/aligned_depth_to_color/image_raw", Image, depth_image_callback)
    image_pub = rospy.Publisher("current_ros_image", Image, queue_size=1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if ros_img:
            image_pub.publish(ros_img)

    rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)

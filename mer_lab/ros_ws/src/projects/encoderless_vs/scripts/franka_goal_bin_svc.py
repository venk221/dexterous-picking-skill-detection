#!/usr/bin/env python3
# license removed for brevity
from email.mime import base
import numpy as np
import sys
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt
import rospy
import cv2
import cv2.aruco as aruco
import time
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray, Int32
# import pyrealsense2 as rs
from cv_bridge import CvBridge, CvBridgeError
from encoderless_vs.srv import franka_bin_img, franka_bin_imgResponse, franka_cp_goal, franka_cp_goalResponse

# Declaring cvBridge for cv to ros conversion and vice versa
bridge = CvBridge()
ros_depth_img = None
franka_control_service = None
cv_depth_img = None

# markers
ee_center = []
base_center = []

status_pub = rospy.Publisher("/vsbot/status", Int32, queue_size=1)
# def update_goal_pose():
#   global franka_control_service

#   rospy.sleep(10)
#   print("Waiting to move franka to goal position")
#   franka_control_service = rospy.ServiceProxy('franka_control_service', franka_cp_goal)
#   goal_srv_resp = franka_control_service(1)
#   control_points = goal_srv_resp.cp.data
#   print("Control_points", control_points)

def franka_binary_image_service(msg):
  global bridge, cv_depth_img
  # Convert ROS image to cv image
  cv_depth_img = bridge.imgmsg_to_cv2(ros_depth_img, "8UC1")

  dmin = rospy.get_param('vsbot/depth_baseline/dmin')
  dmax = rospy.get_param('vsbot/depth_baseline/dmax')

  cv2.imwrite("depth_img_pre.jpg", cv_depth_img)
#   cv_depth_img = cv2.cvtColor(cv_depth_img, cv2.COLOR_GRAY2BGR)

#   r = cv2.selectROI(cv_depth_img)
#   cv_depth_img = cv_depth_img[r[1]:r[1]+r[3], r[0]:r[0]+r[2]]
#   print(np.min(cv_depth_img))
#   print(np.max(cv_depth_img))
#   print(r[0], r[1], r[2], r[3])

#   cv_img = np.where((cv_depth_img < dmin) | (cv_depth_img > dmax))
#   print(cv_img)
  cv_depth_img = np.where((cv_depth_img < dmin) | (cv_depth_img > dmax), 0, cv_depth_img)
  cv_depth_img[np.int64(base_center[1]):480, 0:640] = 0

  cv2.imwrite("depth_img_mid.jpg", cv_depth_img)

#   cv_depth_img = cv2.cvtColor(cv_depth_img, cv2.COLOR_GRAY2BGR)
  k1 = rospy.get_param('vsbot/depth_baseline/kernel1')
  k2 = rospy.get_param('vsbot/depth_baseline/kernel2')
  kernel = np.ones((k1, k2), np.uint8)
  cv_depth_img = cv2.GaussianBlur(cv_depth_img, (5, 5), 0)
  cv_depth_img = cv2.dilate(cv_depth_img, kernel, iterations=3)
  cv_depth_img = cv2.erode(cv_depth_img, kernel, iterations=1)
  cv_depth_img = cv2.morphologyEx(cv_depth_img, cv2.MORPH_OPEN, kernel)

  cv_depth_img = np.where(cv_depth_img > 0, 255, cv_depth_img)

  cv_depth_img = cv2.cvtColor(cv_depth_img, cv2.COLOR_GRAY2BGR)


  ee_resp = Float32MultiArray()
  ee_resp.data = ee_center

  base_resp = Float32MultiArray()
  base_resp.data = base_center

  cv2.imwrite("depth_img_post.jpg", cv_depth_img)
  
  binary_image = bridge.cv2_to_imgmsg(cv_depth_img, "bgr8")
  
  return franka_bin_imgResponse(binary_image, base_resp, ee_resp)

def marker_callback(img_msg):
    global bridge, cv_img, ee_center, base_center
    status_msg = Int32()
    status_msg.data = 1
    status_pub.publish(status_msg)
    cv_img = bridge.imgmsg_to_cv2(img_msg, 'bgr8')

    cv2.imwrite("initial_img.jpg", cv_img)

    gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    arucoParameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(
        gray, aruco_dict, parameters=arucoParameters)
    
    id_list = []
    id_list.clear()
    for i in ids:
        if int(i) == 32:
            id_list.append(int(i))
        elif int(i) == 34:
            id_list.append(int(i))
    # Aruco ID for each marker
    base_id = 34
    ee_id = 32
    # Check if both markers are found
    marker_base_flag = False
    try:
        base_index = id_list.index(base_id)
        marker_base_flag = True
    except:
        marker_base_flag = False
    
    marker_ee_flag = False
    try:
        ee_index = id_list.index(ee_id)
        marker_ee_flag = True
    except:
        marker_ee_flag = False

    # Separating the corner pixel co-ordinates for each marker
    if (marker_base_flag) and (marker_ee_flag):
        ee_corners_list = corners[ee_index].reshape(4, 2)
        base_corners_list = corners[base_index].reshape(4, 2)
        
        # Averaging base corner co-ordinates to obtain marker center
        base_center_x = (base_corners_list[0][0] + base_corners_list[1]
                         [0] + base_corners_list[2][0] + base_corners_list[3][0])/4
        base_center_y = (base_corners_list[0][1] + base_corners_list[1]
                         [1] + base_corners_list[2][1] + base_corners_list[3][1])/4
        base_center = [base_center_x, base_center_y]
    
        # Averaging ee corner co-ordinates to obtain marker center
        ee_center_x = (ee_corners_list[0][0] + ee_corners_list[1]
                       [0] + ee_corners_list[2][0] + ee_corners_list[3][0])/4
        ee_center_y = (ee_corners_list[0][1] + ee_corners_list[1]
                       [1] + ee_corners_list[2][1] + ee_corners_list[3][1])/4
        ee_center = [ee_center_x, ee_center_y]


def depth_callback(ros_img):
    global bridge, ros_depth_img
    
    if ros_img is not None:
        flag = True
        ros_depth_img = ros_img


def main():
    # Initialize the node
    rospy.init_node('franka_goal_feature_node')
    
    # subscriber for rgb image to detect markers
    image_rgb_sub = rospy.Subscriber("/camera/color/image_raw", Image, marker_callback, queue_size=1)

    # subscriber for depth image for binarization
    image_depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, depth_callback, queue_size=1)

    # service declaration to receive the binary image
    bin_img_service = rospy.Service("binary_image_service_response", franka_bin_img, franka_binary_image_service )

    # wait for control points service to be up
    # rospy.wait_for_service('franka_control_service')   
    
    
    # try:
    #     update_goal_pose() # publishes joint positions to position controller
    # except rospy.ROSInterruptException:
    #     pass

    rospy.spin()


if __name__ == '__main__':
    main()
    

#!/usr/bin/env python3

import numpy as np
import sys
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt
import rospy
import cv2
import cv2.aruco as aruco
import time
import math
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64MultiArray, Int64
# import pyrealsense2 as rs
from cv_bridge import CvBridge, CvBridgeError
from encoderless_vision_dl.srv import franka_bin_img, franka_bin_imgResponse

# Define CvBridge for ROS
bridge = CvBridge()
# flag to control the control point service request
control_flag = False

# #global variable to publish the flag for a new frame
# flag_pub = None

# markers
markers = []
th_e = None
th_b = None

base_index = None
ee_index = None

down = 4                # Image downsampling factor before skeletonization

ee_corners_list = []
base_corners_list = []
ee_marker = []      # list for storing ee co-ordinates
base_marker = []    # list for storing base co-ordinates
base_corner_br = []
base_corner_bl = []
ee_corner_tl = []
ee_corner_tr = []
ee_corner_br = []
ee_corner_bl = []
cur_img = None
roi = None
cur_depth_img = None

# depth image
ros_depth_img = None
cv_img = None

# the following callback function receives a request from the control points service 
# and responds with a binary image and the marker centers
def franka_binary_image_service(msg):
    global bridge, cur_depth_img
  # Convert ROS image to cv image
    cur_depth_img = bridge.imgmsg_to_cv2(ros_depth_img, "8UC1")
    cur_depth_img = cv2.pyrDown(cv2.pyrDown(cur_depth_img))

    # Binarize image and segment robot
    dmin = rospy.get_param('vsbot/depth_baseline/dmin')
    dmax = rospy.get_param('vsbot/depth_baseline/dmax')
   
    # dmin = 2
    # dmax = 5

    # Segment depth image based on depth intensity
    cur_depth_img = np.where((cur_depth_img<dmin) | (cur_depth_img>dmax), 0, cur_depth_img)
    cur_depth_img = np.where(cur_depth_img>0,255,cur_depth_img)
    
    # Segment robot mount and other annoyances
    # for x_i in range(cur_depth_img.shape[1]):
    #     for y_i in range(cur_depth_img.shape[0]):
    #         if y_i < base_marker[1]:
    #         # if y_i < max(base_corner_bl[1], base_corner_br[1]):
    #             cur_depth_img[y_i, x_i] = 0
    #         elif y_i > max(ee_corner_bl[1], ee_corner_br[1]):
    #             cur_depth_img[y_i, x_i] = 0 

    kernel_dilate = np.ones((5, 5), np.uint8)
    kernel_erode = np.ones((5,5), np.uint8)

    cur_depth_img = cv2.dilate(cur_depth_img, kernel_dilate, iterations = 3)    
    cur_depth_img = cv2.erode(cur_depth_img, kernel_erode, iterations = 1)      
    cur_depth_img = cv2.GaussianBlur(cur_depth_img, (9,9), 0)  
#   cur_depth_img = cv2.cvtColor(cv_depth_img, cv2.COLOR_GRAY2BGR)

#   r = cv2.selectROI(cv_depth_img)
#   cv_depth_img = cv_depth_img[r[1]:r[1]+r[3], r[0]:r[0]+r[2]]
#   print(np.min(cv_depth_img))
#   print(np.max(cv_depth_img))
#   print(r[0], r[1], r[2], r[3])
    # binary_image = bridge.cv2_to_imgmsg(cur_depth_img, "bgr8")
    binary_image = bridge.cv2_to_imgmsg(cur_depth_img, "mono8")
    marker_resp = Float64MultiArray()
    marker_resp.data = markers
  
    return franka_bin_imgResponse(binary_image, marker_resp)

def marker_callback(img_msg):
    global bridge, cv_img, markers, ee_center, base_center
    # status_msg = Int64()
    # status_msg.data = 1
    # status_pub.publish(status_msg)
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
        base_center = [base_center_x/down, base_center_y/down]
    
        # Averaging ee corner co-ordinates to obtain marker center
        ee_center_x = (ee_corners_list[0][0] + ee_corners_list[1]
                       [0] + ee_corners_list[2][0] + ee_corners_list[3][0])/4
        ee_center_y = (ee_corners_list[0][1] + ee_corners_list[1]
                       [1] + ee_corners_list[2][1] + ee_corners_list[3][1])/4
        ee_center = [ee_center_x/down, ee_center_y/down]    

        th_e = math.atan2((ee_corners_list[2][1]/down - ee_corners_list[3][1]/down),(ee_corners_list[2][0]/down - ee_corners_list[3][0]/down))
        th_b = 0

        markers = [ee_center[0],ee_center[1], base_center[0],base_center[1], th_e, th_b,
                            base_corners_list[2][0]/down, base_corners_list[2][1]/down, base_corners_list[3][0]/down,
                            base_corners_list[3][1]/down, ee_corners_list[2][0]/down, ee_corners_list[2][1]/down,
                            ee_corners_list[3][0]/down, ee_corners_list[3][1]/down, ee_corners_list[0][0]/down,
                            ee_corners_list[0][1]/down, ee_corners_list[1][0]/down, ee_corners_list[1][1]/down]

def depth_callback(ros_img):
    global bridge, ros_depth_img, control_flag
    ros_depth_img = ros_img
    if cv_img is not None and ros_depth_img is not None:
      control_flag = True

    flag_pub.publish(control_flag)
        

def main(args):  
  # Initialize ROS
  global flag_pub
  rospy.init_node('franka_image_segmentation')  
  # Declare subcscribers
  # subscriber for rgb image to detect markers  
  image_rgb_sub = rospy.Subscriber("/camera/color/image_raw", Image, marker_callback, queue_size=1)

  # subscriber for depth image for binarization
  image_depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, depth_callback, queue_size=1)

  # service declaration to receive the binary image
  bin_img_service = rospy.Service("binary_image_service_response", franka_bin_img, franka_binary_image_service)
  
  # publisher to publish flag to start control points svc
  flag_pub = rospy.Publisher("/franka/control_flag", Bool, queue_size = 1)
    

  rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
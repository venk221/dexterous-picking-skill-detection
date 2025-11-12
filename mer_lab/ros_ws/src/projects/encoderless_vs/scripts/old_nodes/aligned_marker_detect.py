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
    global bridge, ros_img, ee_center, base_center, base_index, ee_index, marker_base_flag,\
         marker_ee_flag, ee_corners_list, base_corners_list
    cv_img = bridge.imgmsg_to_cv2(img_msg, 'bgr8')

    gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY) 
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50) 

    arucoParameters = aruco.DetectorParameters_create() 

    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=arucoParameters)

    # print("Id generated", ids) #Remove
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
    try:    
        base_index = id_list.index(base_id)
        marker_base_flag = True
    except:
        marker_base_flag = False
    try:
        ee_index = id_list.index(ee_id)
        marker_ee_flag = True
    except:
        marker_ee_flag = False

    # if not marker_flag:
    #     print("MARKER NOT FOUND")
    # Separating the corner pixel co-ordinates for each marker
    if (marker_base_flag) and (marker_ee_flag):
        ee_corners_list = corners[ee_index].reshape(4,2)
        base_corners_list = corners[base_index].reshape(4,2)
    
    
    # print("ee corner list", ee_corners_list) #Remove

    # Averaging base corner co-ordinates to obtain marker center
    base_center_x = (base_corners_list[0][0] + base_corners_list[1][0] + base_corners_list[2][0] + base_corners_list[3][0])/4
    base_center_y = (base_corners_list[0][1] + base_corners_list[1][1] + base_corners_list[2][1] + base_corners_list[3][1])/4
    
    base_center = [base_center_x, base_center_y]

    # print(base_center)
    
    # Averaging ee corner co-ordinates to obtain marker center
    ee_center_x = (ee_corners_list[0][0] + ee_corners_list[1][0] + ee_corners_list[2][0] + ee_corners_list[3][0])/4
    ee_center_y = (ee_corners_list[0][1] + ee_corners_list[1][1] + ee_corners_list[2][1] + ee_corners_list[3][1])/4
    
    ee_center = [ee_center_x, ee_center_y]

    # print(ee_center)

    # Compute ee_center_co-ordinate w.r.t base marker
    # ee_center = [ee_center_x - base_center_x, ee_center_y - base_center_y]
    # Draw a box on detected markers
    cv_img = aruco.drawDetectedMarkers(cv_img, corners)
    # Draw the centers
    # cv2.circle(cv_img, (int(base_center_x), int(base_center_y)), 4, [0, 255, 255], -1)
    cv2.circle(cv_img, (int(ee_center_x), int(ee_center_y)), 4, [255, 255, 0], -1)
    cv2.circle(cv_img, (int(base_center_x), int(base_center_y)), 4, [255, 255, 0], -1)
    # Convert back to ros img to publish

    # cv2.imshow("Image", cv_img)
    # cv2.waitKey(10)

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
    marker_detect_pub = rospy.Publisher("marker/Image", Image, queue_size=1)
    base_pose_pub = rospy.Publisher("base/Pose", Float32MultiArray, queue_size=1)
    ee_pose_pub = rospy.Publisher("ee/Pose", Float32MultiArray, queue_size=1)

    # Rate Loop to publish
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():

        # Convert ee pose to rosmsg
        ee_marker = Float32MultiArray()
        ee_marker.data = ee_center

        # Convert base pose to rosmsg
        base_marker = Float32MultiArray()
        base_marker.data = base_center

        # Publish marker pose
        if ee_center != []:
            ee_pose_pub.publish(ee_marker)
        
        # Publish marker pose
        if base_center != []:
            base_pose_pub.publish(base_marker)

        # Publish image
        if ros_img is not None:
            marker_detect_pub.publish(ros_img)

        rate.sleep()

    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)

#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco
from std_msgs.msg import Float64MultiArray
import sys
import math

bridge = CvBridge()
ros_img = None
marker_flag = False

ee_center = []
base_center = []

th_e = None
th_b = None

base_index = None
ee_index = None

down = 1                # Image downsampling factor before skeletonization

ee_corners_list = []
base_corners_list = []

def marker_pose(img_msg):
    global bridge, ros_img, ee_center, base_center, base_index, ee_index, marker_flag, ee_corners_list, base_corners_list, th_e, th_b
    cv_img = bridge.imgmsg_to_cv2(img_msg, 'bgr8')
    # cv_img = cv2.pyrDown(cv2.pyrDown(cv_img))
    # cv_img = cv2.pyrDown(cv_img)
    # print (cv_img.shape)
    gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50) 

    arucoParameters = aruco.DetectorParameters_create()

    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=arucoParameters)

    # Aruco ID for each marker
    base_id = 34
    ee_id = 32
    
    # print("Id generated", ids) #Remove
    id_list = []
    
    id_list.clear()
    for i in ids:
        if int(i) == ee_id:
            id_list.append(int(i))
        elif int(i) == base_id:
            id_list.append(int(i))
        

    
    # Check if both markers are found
    try:    
        base_index = id_list.index(base_id)
        ee_index = id_list.index(ee_id)
        marker_flag = True
    except:
        marker_flag = False

    # try:
    #     ee_index = id_list.index(ee_id)
    #     marker_flag = True
    # except:
    #     marker_flag = False

    if marker_flag:
        ee_corners_list = corners[ee_index].reshape(4,2)
        base_corners_list = corners[base_index].reshape(4,2)

    # Averaging ee corner co-ordinates to obtain marker center
    ee_center_x = (ee_corners_list[0][0] + ee_corners_list[1][0] + ee_corners_list[2][0] + ee_corners_list[3][0])/4
    ee_center_y = (ee_corners_list[0][1] + ee_corners_list[1][1] + ee_corners_list[2][1] + ee_corners_list[3][1])/4
    
    ee_center = [ee_center_x/down, ee_center_y/down]

    base_center_x = (base_corners_list[0][0] + base_corners_list[1][0] + base_corners_list[2][0] + base_corners_list[3][0])/4
    base_center_y = (base_corners_list[0][1] + base_corners_list[1][1] + base_corners_list[2][1] + base_corners_list[3][1])/4

    base_center = [base_center_x/down, base_center_y/down]

    th_e = math.atan2((ee_corners_list[2][1]/down - ee_corners_list[3][1]/down),(ee_corners_list[2][0]/down - ee_corners_list[3][0]/down))
    th_b = 0

    # Draw a box on detected markers
    cv_img = aruco.drawDetectedMarkers(cv_img, corners)
    
    # Draw the centers
    cv2.circle(cv_img, (int(base_center_x), int(base_center_y)), 5, [255, 133, 26], -1)
    cv2.circle(cv_img, (int(ee_center_x), int(ee_center_y)), 5, [89, 17, 212], -1)

    # Printing corners
    # cv2.circle(cv_img, (int(base_corners_list[0][0]), int(base_corners_list[0][1])), 4, [0,0,255], -1)
    # cv2.circle(cv_img, (int(base_corners_list[1][0]), int(base_corners_list[1][1])), 4, [0,255,0], -1)
    # cv2.circle(cv_img, (int(base_corners_list[2][0]), int(base_corners_list[2][1])), 4, [255,0,0], -1)

    # Convert back to ros img to publish
    ros_img = bridge.cv2_to_imgmsg(cv_img, 'bgr8')


def main(args):
    # Initialize ROS node
    rospy.init_node('detect_markers')

    # Subscribers
    image_sub = rospy.Subscriber("camera/color/image_raw", Image, marker_pose, queue_size=1)

    # Publishers
    marker_detect_pub = rospy.Publisher("origami_vs/aruco/result", Image, queue_size=1)
    marker_pose_pub = rospy.Publisher("origami_vs/aruco/pose", Float64MultiArray, queue_size=1)
    
    # Read parameters
    rate = rospy.get_param("origami_vs/control_rate")
    r = rospy.Rate(rate)
    
    while not rospy.is_shutdown():
        
        # Convert pose to rosmsg & publish
        if ee_center and base_center != []:
            markers = Float64MultiArray()
            markers.data = [ee_center[0],ee_center[1], base_center[0],base_center[1], th_e, th_b,
                            base_corners_list[2][0]/down, base_corners_list[2][1]/down, base_corners_list[3][0]/down,
                            base_corners_list[3][1]/down, ee_corners_list[2][0]/down, ee_corners_list[2][1]/down,
                            ee_corners_list[3][0]/down, ee_corners_list[3][1]/down, ee_corners_list[0][0]/down,
                            ee_corners_list[0][1]/down, ee_corners_list[1][0]/down, ee_corners_list[1][1]/down]
            
            # print("Marker detect: " + str(markers.data))
            marker_pose_pub.publish(markers)

        # Publish image
        if ros_img:
            marker_detect_pub.publish(ros_img)

        r.sleep()

    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)

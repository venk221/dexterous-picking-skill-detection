#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco
from std_msgs.msg import Float64MultiArray
import sys

bridge = CvBridge()
ros_img = None
marker_flag = False

ee_center = []
base_center = []

base_index = None
ee_index = None

ee_corners_list = []
base_corners_list = []

def marker_pose(img_msg):
    global bridge, ros_img, ee_center, base_center, base_index, ee_index, marker_flag, ee_corners_list, base_corners_list
    cv_img = bridge.imgmsg_to_cv2(img_msg, 'bgr8')

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
    
    ee_center = [ee_center_x, ee_center_y]

    base_center_x = (base_corners_list[0][0] + base_corners_list[1][0] + base_corners_list[2][0] + base_corners_list[3][0])/4
    base_center_y = (base_corners_list[0][1] + base_corners_list[1][1] + base_corners_list[2][1] + base_corners_list[3][1])/4

    base_center = [base_center_x, base_center_y]

    # print(ee_center)
    # print(base_center)

    # Draw a box on detected markers
    cv_img = aruco.drawDetectedMarkers(cv_img, corners)
    
    # Draw the centers
    cv2.circle(cv_img, (int(base_center_x), int(base_center_y)), 4, [0, 255, 255], -1)
    cv2.circle(cv_img, (int(ee_center_x), int(ee_center_y)), 4, [255, 255, 0], -1)

    # Printing corners
    # cv2.circle(cv_img, (int(ee_corners_list[0][0]), int(ee_corners_list[0][1])), 4, [0,0,255], -1)
    # cv2.circle(cv_img, (int(ee_corners_list[1][0]), int(ee_corners_list[1][1])), 4, [0,255,0], -1)
    # cv2.circle(cv_img, (int(ee_corners_list[2][0]), int(ee_corners_list[2][1])), 4, [255,0,0], -1)

    # Convert back to ros img to publish
    ros_img = bridge.cv2_to_imgmsg(cv_img, 'bgr8')


def main(args):
    # Initialize ROS node
    rospy.init_node('detect markers')

    # Subscribers
    image_sub = rospy.Subscriber("camera/color/image_raw", Image, marker_pose, queue_size=1)

    # Publishers
    marker_detect_pub = rospy.Publisher("origami_vs/aruco/result", Image, queue_size=1)
    marker_pose_pub = rospy.Publisher("origami_vs/aruco/pose", Float64MultiArray, queue_size=1)
    
    # Read parameters
    rate = rospy.get_param("origami_vs/control_rate")
    r = rospy.Rate(rate)
    
    while not rospy.is_shutdown():
        
        # Convert end-effector pose to rosmsg
        markers = Float64MultiArray()
        markers.data = [ee_center, base_center]

        # Publish marker pose
        if ee_center and base_center != []:
            marker_pose_pub.publish(markers)

        # Publish image
        if ros_img:
            marker_detect_pub.publish(ros_img)

        r.sleep()

    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)

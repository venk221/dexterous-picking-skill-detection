#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco
from std_msgs.msg import Float32MultiArray, Bool
import sys
# from const_vel_kalman_filter import KalmanFilter
# from simple_kalman_filter import KalmanFilter
from extended_kalman_filter import ExtendedKalmanFilter
from missing_feature_kalman_filter import KalmanFilter

bridge = CvBridge()
ros_img = None
ee_center = []
marker_flag = None
list_measured = []
i = 0
m = 0

cx_list = []
cy_list = []

#KalmanFilter(dt, x_std_meas, y_std_meas)
# kf = KalmanFilter(0.1, 1, 1, 1, 0.1,0.1)

# kf = KalmanFilter(0.2, 1, 1, 1, 0.1, 0.1)
# kf = ExtendedKalmanFilter(1.0, 0.5)
kf = KalmanFilter(1, 10, 10, 7, 7)

def kalman_estimate(ee_center, img):
    global i, kf, marker_flag, list_measured, cx_list, cy_list

    # # Detect feature points
    cx = ee_center[0]
    cy = ee_center[1]
    # theta = np.arctan(-cy/cx)   

    cx_list = np.append(cx, cx_list)
    cy_list = np.append(cy, cy_list)

    dx = cx_list[0]-cx_list[1]
    dy = cy_list[0]-cy_list[1]

    vx = dx
    vy = dy

    measured = np.array([[np.float32(cx)], [np.float32(cy)], [np.float32(vx)], [np.float32(vy)]])
    # meas_img = np.array([[np.float32(cx)], [np.float32(cy)], [np.float32(theta)]])

    print("Measured", measured)

    # Predict    
    predicted = kf.predict()

    # pred_img = np.array([predicted[0][0], predicted[1][0], pred_theta])
    # Update
    print("Marker Flag", marker_flag)
    if marker_flag:
        updated = np.asarray(kf.update(measured))
        print("Updated when True", updated)
        list_measured = np.append(updated, list_measured) 
        # if len(list_measured)<60
        #     list_measured = list_measured
        # else:
        #     list_measured = list_measured[0:60]        
    elif not marker_flag:
        # rospy.sleep(2)
        new_measured = np.array([[np.float32(list_measured[0])], [np.float32(list_measured[1])], [np.float32(list_measured[2])], [np.float32(list_measured[3])]])
        updated = np.asarray(kf.update(new_measured))
        list_measured = np.append(updated, list_measured)
        # if len(list_measured)<60
        #     list_measured = list_measured
        # else:
        #     list_measured = list_measured[0:60] 
        print("Updated when False", updated)

    # upd_img = np.array([updated[0][0], updated[1][0], upd_theta])

    print("predicted", predicted[0:2])
    
    pred_x, pred_y = int(predicted[0][0]), int(predicted[1][0])   
    upd_x, upd_y = int(updated[0]), int(updated[1])
    # print("Updated", updated)

    font = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = 0.4
    org1 = (10, 20)
    org2 = (10,80)
    org3 = (10, 100)
    org4 = (10,140)
    org5 = (10, 160)


    cv2.putText(img, str(measured), org1, font, fontScale, (0, 255, 0), 1, cv2.LINE_AA)
    cv2.circle(img, (int(cx), int(cy)), 5, (0, 255, 0), -1)
    cv2.putText(img, str(predicted[0:4]), org2, font, fontScale, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.putText(img, str(predicted[4:]), org3, font, fontScale, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.circle(img, (pred_x, pred_y), 10, (0, 0, 0), 4)
    cv2.putText(img, str(updated[0:4]), org4, font, fontScale, (0, 0, 255), 1, cv2.LINE_AA)
    cv2.putText(img, str(updated[4:]), org5, font, fontScale, (0, 0, 255), 1, cv2.LINE_AA)
    cv2.circle(img, (upd_x, upd_y), 5, (0, 0, 255), 4)

    print("next frame"+str(i))
    
    cv2.imwrite("/home/merlab/Pictures/kalman_images/"+str(i)+'.jpg', img)

    i=i+1

    # if i > 50:
    #     marker_flag = False


# Subscriber callback
def marker_pose(img_msg):    
    global bridge, ros_img, ee_center, base_index, ee_index, marker_flag, ee_corners_list, m, ee_obs, marker_flag
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
    base_id = 32
    ee_id = 34
    
    # Check if both markers are found
    # try:    
    #     base_index = id_list.index(base_id)
    #     marker_flag = True
    # except:
    #     marker_flag = False
    try:
        ee_index = id_list.index(ee_id)
        marker_flag = True
    except:
        marker_flag = False

    # if not marker_flag:
    #     print("MARKER NOT FOUND")
    # Separating the corner pixel co-ordinates for each marker
    if marker_flag:
        # base_corners_list = corners[base_index].reshape(4,2)   # [[x1, y1],[x2, y2],[x3, y3],[x4, y4]]
        ee_corners_list = corners[ee_index].reshape(4,2)
    else:
        print("Marker Flag", marker_flag)
    
    
    # print("ee corner list", ee_corners_list) #Remove

    # Averaging base corner co-ordinates to obtain marker center
    # base_center_x = (base_corners_list[0][0] + base_corners_list[1][0] + base_corners_list[2][0] + base_corners_list[3][0])/4
    # base_center_y = (base_corners_list[0][1] + base_corners_list[1][1] + base_corners_list[2][1] + base_corners_list[3][1])/4
    
    # base_center = [base_center_x, base_center_y]
    
    # Averaging ee corner co-ordinates to obtain marker center
    ee_center_x = (ee_corners_list[0][0] + ee_corners_list[1][0] + ee_corners_list[2][0] + ee_corners_list[3][0])/4
    ee_center_y = (ee_corners_list[0][1] + ee_corners_list[1][1] + ee_corners_list[2][1] + ee_corners_list[3][1])/4
    
    ee_center = [ee_center_x, ee_center_y]

    # print("marker center", ee_center)

    # print("marker center", ee_center)
    # Compute ee_center_co-ordinate w.r.t base marker
    # ee_center = [ee_center_x - base_center_x, ee_center_y - base_center_y]
    # Draw a box on detected markers
    img = aruco.drawDetectedMarkers(cv_img, corners)
    # Draw the centers
    # cv2.circle(cv_img, (int(base_center_x), int(base_center_y)), 4, [0, 255, 255], -1)
    # cv2.circle(cv_img, (int(ee_center_x), int(ee_center_y)), 4, [255, 255, 0], -1)

    # cv2.imwrite("marked_img"+str(m)+".jpg", img)    

    # cv2.imshow("marked image", img)
    # cv2.waitKey(1)

    # Convert back to ros img to publish
    ros_img = bridge.cv2_to_imgmsg(cv_img, 'bgr8')

    m = m+1


    kalman_estimate(ee_center, cv_img)


def main(args):
    # Initialize ROS
    rospy.init_node('feature_detection')

    # Listening to the start flag

    # Subscribers
    image_sub = rospy.Subscriber("/camera/color/image_raw", Image, marker_pose, queue_size=1)

    marker_flag_pub = rospy.Publisher("/feature/Flag", Bool, queue_size=1)

    # Rate Loop to publish
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        marker_flag_pub.publish(marker_flag)
        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)

#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from encoderless_vs.srv import franka_cp_vis, franka_cp_visResponse
from scipy.interpolate import splev
from cv_bridge import CvBridge, CvBridgeError


def main():
    # initialize the node
    rospy.init_node('franka_overlay_vis_node') 
    
    rospy.sleep(15)
    # to convert cv to ros
    bridge = CvBridge()
    # wait for control points service to be up
    rospy.wait_for_service('franka_control_service')

    # create a handle to call the service
    franka_control_service = rospy.ServiceProxy('franka_control_service', franka_cp_vis)    
    # service request is sent to receive tck
    srv_resp = franka_control_service(1)
    ros_rgb_img = srv_resp.rgb

    cv_rgb_img = bridge.imgmsg_to_cv2(ros_rgb_img, "bgr8")

    cv2.imwrite("franka_robot.jpg", cv_rgb_img)

    # cv_skel_img = bridge.imgmsg_to_cv2(ros_skel_img, "bgr8")

    # skel_on_franka = cv2.addWeighted(cv_skel_img, 0.3, cv_rgb_img, 1,0)

    # cv2.imwrite("skeleton_on_franka.jpg", skel_on_franka)

    control_points = srv_resp.cp.data
    print("Control Points", control_points)
    t = np.array(srv_resp.t)
    cx = np.array(srv_resp.cx)
    cy = np.array(srv_resp.cy)
    k = srv_resp.k

    print(t, cx, cy, k)

    tck = [t, [cx, cy], k]    

    # generating new parameters to be evaluated
    params = np.linspace(0, 1, 200)

    # new points after evaluation
    newPts =  np.array(splev(params, tck))
    x = newPts[0]
    y = newPts[1]


    points = np.c_[x,y]

    ct_pts = np.c_[cx, cy]

    # Create a blank image with black background
    # cv_img = np.zeros((720, 1280, 3), dtype = "uint8") 
    
    for i in range(len(points)):
      x = np.int(points[i][0])
      y = np.int(points[i][1])
      cv2.circle(cv_rgb_img,(x, y),3,(0,0,255),-1)
    
    for i in range(len(ct_pts)):
      x = np.int(ct_pts[i][0])
      y = np.int(ct_pts[i][1])
      cv2.circle(cv_rgb_img,(x, y),5,(0,255,255),-1)

    
    # save the goal curve image in .ros file
    cv2.imwrite("franka_published_goal_image.jpg", cv_rgb_img)

    rospy.spin()
         

if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from encoderless_vs.srv import cp_goal, cp_goalResponse
from scipy.interpolate import splev
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def main():
    # initialize the node
    rospy.init_node('goal_vis_node')
    rospy.sleep(6)
    
    # to convert cv to ros
    bridge = CvBridge()

    # wait for control points service to be up
    rospy.wait_for_service('control_points_output')

    # create a handle to call the service
    control_points_output = rospy.ServiceProxy('control_points_output', cp_goal)

    # service request is sent to receive tck
    srv_resp = control_points_output(1)
    t = np.array(srv_resp.t)
    cx = np.array(srv_resp.cx)
    cy = np.array(srv_resp.cy)
    k = srv_resp.k

    tck = [t, [cx, cy], k]    

    # generating new parameters to be evaluated
    params = np.linspace(0, 1, 20)

    # new points after evaluation
    newPts =  np.array(splev(params, tck))
    x = newPts[0]
    y = newPts[1]


    points = np.c_[x,y]

    ct_pts = np.c_[cx, cy]

    # Create a blank image with black background
    cv_img = np.zeros((300, 300, 3), dtype = "uint8") 
    
    for i in range(len(points)):
      x = np.int(points[i][0])
      y = np.int(points[i][1])
      cv2.circle(cv_img,(x, y),2,(0,0,255),-1)
    
    for i in range(len(ct_pts)):
      x = np.int(ct_pts[i][0])
      y = np.int(ct_pts[i][1])
      cv2.circle(cv_img,(x, y),3,(0,255,255),-1)

    
    # save the goal curve image in .ros file
    cv2.imwrite("published_goal_image.jpg", cv_img)

    rospy.spin()
         

if __name__ == '__main__':
    main()

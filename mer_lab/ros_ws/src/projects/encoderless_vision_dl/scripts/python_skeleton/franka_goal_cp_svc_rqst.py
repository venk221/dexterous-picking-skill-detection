#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from encoderless_vision_dl.srv import franka_cp_goal, franka_cp_goalResponse
from scipy.interpolate import splev
from cv_bridge import CvBridge, CvBridgeError


def main():
    # initialize the node
    rospy.init_node('franka_goal_vis_node') 
    
    rospy.sleep(20)
    # to convert cv to ros
    bridge = CvBridge()
    # wait for control points service to be up
    rospy.wait_for_service('franka_control_service')

    # create a handle to call the service
    franka_control_service = rospy.ServiceProxy('franka_control_service', franka_cp_goal)    
    # service request is sent to receive tck
    srv_resp = franka_control_service(1)
    # control_points = srv_resp.cp.data
    
    # Print control points to terminal
    # print("Control Points", control_points)

    ## Ouput a YAML file with these parameters
    yaml_file = open("franka_features.yaml","w")
    s = ""
    s += "shape_controller:\n"
    s += "  goal_features: [" + (','.join(map(str,srv_resp.cp.data))) +"]\n"
    
    yaml_file.write(s)
    yaml_file.close()
    
    
    t = np.array(srv_resp.t)
    cx = np.array(srv_resp.cx)
    cy = np.array(srv_resp.cy)
    k = srv_resp.k

    # print(t, cx, cy, k)

    tck = [t, [cx, cy], k]    

    # generating new parameters to be evaluated
    params = np.linspace(0, 1, 20)

    # new points after evaluation
    newPts =  np.array(splev(params, tck))
    x = newPts[0]
    y = newPts[1]


    points = np.c_[x,y]

    ct_pts = np.c_[cx, cy]

    # print("Control Points", ct_pts)

    # Create a blank image with black background
    cv_img = np.zeros((120, 160, 3), dtype = "uint8") 
    # cv_img.fill(255)
    
    for i in range(len(points)):
      x = np.int(points[i][0])
      y = np.int(points[i][1])
      cv2.circle(cv_img,(x, y),2,(0,0,255),-1)

    x1 = np.int(ct_pts[0][0])
    y1 = np.int(ct_pts[0][1])

    x2 = np.int(ct_pts[1][0])
    y2 = np.int(ct_pts[1][1])

    x3 = np.int(ct_pts[2][0])
    y3 = np.int(ct_pts[2][1])

    x4 = np.int(ct_pts[3][0])
    y4 = np.int(ct_pts[3][1])

    # print("Individual Control Points", x1,y1, x2,y2, x3,y3, x4,y4)
    
    # cv2.circle(cv_img, (x1, y1), 5, (255,0,0), 5)

    # cv2.circle(cv_img, (x2, y2), 5, (0,255,0), 5)

    # cv2.circle(cv_img, (x3, y3), 5, (255,255,0), 5)

    # cv2.circle(cv_img, (x4, y4), 5, (0,0,0), 5)    
  
    
    for i in range(len(ct_pts)):
      x = np.int(ct_pts[i][0])
      y = np.int(ct_pts[i][1])
      cv2.circle(cv_img,(x, y),5,(0,255,255),-1)

    # Upsampling in the final image
    # cv_img = cv2.pyrUp(cv2.pyrUp(cv_img))
    # save the goal curve image in .ros file
    cv2.imwrite("franka_published_goal_image.jpg", cv_img)

    rospy.signal_shutdown("Wrote features to file")
         

if __name__ == '__main__':
    main()
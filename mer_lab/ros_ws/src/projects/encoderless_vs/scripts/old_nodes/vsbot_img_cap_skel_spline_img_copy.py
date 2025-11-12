#!/usr/bin/env python3
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import numpy as np
from skimage.morphology import skeletonize
from std_msgs.msg import String, Float64MultiArray, Float64
from sensor_msgs.msg import Image
from scipy.interpolate import splprep, splev
import time
from cv_bridge import CvBridge, CvBridgeError
from sympy import lambdify, bspline_basis_set
from sympy.abc import u

bridge = CvBridge()
skel_pub = rospy.Publisher('skel_viewer', Image, queue_size = 1)
pub_cp_x = rospy.Publisher('control_points_x', Float64MultiArray, queue_size= 1)
pub_cp_y = rospy.Publisher('control_points_y', Float64MultiArray, queue_size= 1)
pub_basis_function = rospy.Publisher('basis_function', Float64MultiArray, queue_size= 1)

def image_callback(ros_image):      
    global bridge, captured_image, fig, ax, bgd, dataPts, splinePts
    rate = rospy.Rate(60)
    #capture image in gazebo
    try:
        cv_image_capture = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)	   
    
    #converting it into masked image
    
    tstart = time.time()
    hsvimg = cv2.cvtColor(cv_image_capture, cv2.COLOR_BGR2HSV)
	  	  
    orange_lower = np.array([10, 100, 20], np.uint8) 
    orange_upper = np.array([25, 255, 255], np.uint8) 
      
    white_lower = np.array([0, 0, 200], np.uint8)
    white_upper = np.array([145, 60, 255], np.uint8) 
	  
    orange_mask = cv2.inRange(hsvimg, orange_lower, orange_upper)
    white_mask = cv2.inRange(hsvimg, white_lower, white_upper)
	    
    kernel = np.ones((5, 5), "uint8")
	
    cv_mask = orange_mask + white_mask
    masked_image = bridge.cv2_to_imgmsg(cv_mask, "mono8")
    #Convert binary to skeleton
    binary = np.where(cv_mask > 0, 1, cv_mask)  
    skeleton = skeletonize(binary, method='lee')
    skeleton = np.where(skeleton == 1, 255, skeleton)       
  
    tstop = time.time()
    timer = tstop - tstart
    print("Binary and Skeleton Timing", timer)
    # End Convert binary to skeleton
    
    #Start of Ordering pixels of skeleton using nearest neigbor search
    tstart = time.time()
    pixels = np.argwhere(skeleton > 0)
    xinit = pixels[:, 1]
    yinit = pixels[:, 0]
    
    init_point = (149, 149)
    dist_mat = []
    for i in range(len(xinit)):
      dist = np.sqrt((xinit[i]-init_point[0])**2 + (yinit[i]-init_point[1])**2)
      dist_mat = np.append(dist_mat, dist)
      
    index = np.argsort(dist_mat)
    xinit, yinit = xinit[index], yinit[index]
    # print("xinit after ordering: ", xinit)
    # print("yinit after ordering: ", yinit)
    point_jump = (len(xinit))/2
    # print(int(point_jump))
    x = np.array(xinit[0::round(point_jump)])
    # print("The jumped X is: ", x)
    y = np.array(yinit[0::round(point_jump)])
    # print(print("The jumped y is: ", y))
    if (x[-1] != xinit[-1]) or (y[-1] != yinit[-1]) :
        x = np.append(x, xinit[-1])
        y = np.append(y, yinit[-1])
        # print("XY after 1st if", [x,y])
        if ((x[-1] - x[-2] == 1) or (x[-1] - x[-2] == -1)) and ((y[-1] - y[-2] == 1) or (y[-1] - y[-2] == -1)):
          x = np.delete(x, -2)
          y = np.delete(y, -2) 
          # print("XY after 2nd if", [x,y])
    tstop = time.time()
    timer = tstop - tstart
    print("Time to order the points using NN Search", timer)           
    #End of Ordering pixels of skeleton using nearest neigbor search
    #Start of using splprep and splev
    tstart = time.time()
    tck, u_ticks = splprep([x, y], k =2, s = 1)
    print(u_ticks)
    print(tck[1])

    control_points_x = Float64MultiArray()
    control_points_y = Float64MultiArray()
    control_points_x.data = tck[1][0]
    control_points_y.data = tck[1][1]
    
    pub_cp_x.publish(control_points_x)
    pub_cp_y.publish(control_points_y)

    newPts = splev(u_ticks, tck)

    print("New Points: ", newPts)

    x = newPts[0].astype(int)
    y = newPts[1].astype(int)

    basis = bspline_basis_set(tck[2], tck[0],  u)
    np_basis = [lambdify(u, b, modules=['numpy']) for b in basis]

    basis_funcn = []
    for bi in np_basis:
      for u_val in u_ticks:
        if bi(u_val) == 0 or bi(u_val)== 1:
          basis_funcn = basis_funcn
        else:
          basis_funcn = np.append(basis_funcn, bi(u_val))

    
    basis_function = Float64MultiArray()
    basis_function.data = basis_funcn

    pub_basis_function.publish(basis_function)

    tstop = time.time()
    timer = tstop-tstart
    print("Time to evaluate the Spline: ", timer)

    # Display the spline
    points = np.c_[x, y]

    points = ([tuple(i) for i in points])

    print("points: ", points)

    blank_img = np.zeros((300, 300, 3), dtype = "uint8") 
 

    red = [0, 0, 255]
    
    cv2.polylines(blank_img, [np.array(points)], isClosed = False, color = red, thickness = 3, 
              lineType = cv2.LINE_AA)  

    cv2.imshow("Spline_image", blank_img)
    cv2.waitKey(1)
      
    
    try:
        ros_skeleton = bridge.cv2_to_imgmsg(skeleton, "mono8")
    except CvBridgeError as e:
        print(e)    
    if not rospy.is_shutdown():          
          skel_pub.publish(ros_skeleton)
    
    
def main(args):
  rospy.init_node('image_skel_spline_trial')
  image_sub = rospy.Subscriber("image_publisher",Image,image_callback,  queue_size = 1)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

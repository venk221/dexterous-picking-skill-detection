#!/usr/bin/env python3

# ROS deps
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
# Image processing
import cv2
from cv_bridge import CvBridge, CvBridgeError
# Math
from numpy import pi
import numpy as np
import math
# Curve fitting and processing libs
from skimage.morphology import skeletonize
from astar import MyAstar
from scipy.interpolate import splprep, splev
from pyclothoids import Clothoid

import time

# -----------------------------------------------------------
# global marker objects
ee_marker = []      # list for storing ee co-ordinates
base_marker = []    # list for storing base co-ordinates
base_corner_br = []
base_corner_bl = []
ee_corner_tl = []
ee_corner_tr = []
ee_corner_br = []
ee_corner_bl = []

# Counter for writing unique image IDs
count = 0

# global image objects
cur_img = None
img = None
cur_depth_img = None
depth_img = None
ref_image = None

# Curve input parameters
k = 0
num_of_segments = 0
curve_type = ""

# Global publishers
curve_pub = rospy.Publisher('origami_vs/curve_image', Image, queue_size=1)
feature_pub = rospy.Publisher('origami_vs/feature_pub', Float64MultiArray, queue_size=1)

# Global CV bridge object
bridge = CvBridge() 

# --------------------------------------------------------------------------------------
# Get markers (curve start and end)
def getMarkers(msg):
    global ee_marker, base_marker, base_corner_br, base_corner_bl
    global ee_corner_br, ee_corner_bl, ee_corner_tl, ee_corner_tr
    
    ee_marker = msg.data[0:2]
    base_marker = msg.data[2:4]
    base_corner_br = msg.data[6:8]      # bottom right
    base_corner_bl = msg.data[8:10]     # bottom left
    ee_corner_br = msg.data[10:12]      # bottom right
    ee_corner_bl = msg.data[12:14]      # bottom left
    ee_corner_tl = msg.data[14:16]      # top left
    ee_corner_tr = msg.data[16:18]      # top right


# Get color image & downsample
def getImage(msg):
    global img, cur_img, bridge
    cur_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    # cur_img = cv2.pyrDown(cv2.pyrDown(img))


# Get depth image & downsample
def getDepthImage(msg):
    global bridge, cur_depth_img, depth_img
    cur_depth_img = bridge.imgmsg_to_cv2(msg, "8UC1")
    # cur_depth_img = cv2.pyrDown(cv2.pyrDown(depth_img))
    # skeletonization()


# Get reference image
def getRefImage(msg):
    global ref_img, bridge
    ref_img = bridge.imgmsg_to_cv2(msg, "bgr8")


# Skeletonize method
# def skeletonization():
    # global bridge, it, cur_depth_img, cur_img, curve_type
    # # roi(cur_img, cur_depth_img)
    # # Binarize image and segment robot
    # dmin = 0
    # dmax = 3

    # cur_depth_img = np.where((cur_depth_img<dmin) | (cur_depth_img>dmax), 0, cur_depth_img)
    
    # # Segment robot mount and other annoyances
    # for x_i in range(cur_depth_img.shape[1]):
    #     for y_i in range(cur_depth_img.shape[0]):
    #         if y_i < base_marker[1]:
    #         # if y_i < max(base_corner_bl[1], base_corner_br[1]):
    #             cur_depth_img[y_i, x_i] = 0
    #         elif y_i > max(ee_corner_bl[1], ee_corner_br[1]):
    #             cur_depth_img[y_i, x_i] = 0 

    # kernel_dilate = np.ones((3, 3), np.uint8)
    # kernel_erode = np.ones((7,7), np.uint8)

    # cur_depth_img = cv2.dilate(cur_depth_img, kernel_dilate, iterations = 1)
    # cur_depth_img = cv2.erode(cur_depth_img, kernel_erode, iterations = 1)
    # cur_depth_img = cv2.GaussianBlur(cur_depth_img, (9,9), 0)

    # cur_depth_img = np.where(cur_depth_img == 255, 1, cur_depth_img)
    # skeleton = skeletonize(cur_depth_img, method='lee')
    # skeleton = np.where(skeleton >0, 255, skeleton)

    # start, goal, robot_base, skel_px = constrain_skeleton(skeleton)

    # # Recontruct skeleton
    # a_star = MyAstar(start, goal, skeleton.shape[0], skeleton.shape[1], skeleton)
    # if(a_star.IsValid(start[0], start[1])):
    #     if(a_star.IsValid(goal[0], goal[1])):
    #         if(a_star.IsObstacle(start[0],start[1]) == False):
    #             if(a_star.IsObstacle(goal[0], goal[1]) == False):
    #                 (exploredStates, backtrackStates, distanceFromStartToGoal) = a_star.search()

    #                 if(distanceFromStartToGoal == float('inf')):
    #                     print("\nNo optimal path found.")
    #                 else:
    #                     pass
    #                     # print("\nOptimal path found. Distance is " + str(distanceFromStartToGoal))
    #             else:
    #                 print("The entered goal node is an obstacle ")
    #                 print("Please check README.md file for running astar.py file.")
    #         else:
    #             print("The entered start node is an obstacle ")
    #             print("Please check README.md file for running astar.py file.")
    #     else:
    #         print("The entered goal node outside the map ")
    #         print("Please check README.md file for running astar.py file.")
    # else:
    #     print("The entered start node is outside the map ")
    #     print("Please check README.md file for running astar.py file.")

    # Choose curve to fit
    if curve_type == 'spline':
        pass
        # fit_spline(robot_base, backtrackStates)
    elif curve_type == 'clothoid':
        fit_clothoid()


# Constrain the skeleton
def constrain_skeleton(skeleton):
    robot_base = [int((base_corner_bl[0] + base_corner_br[0])/2), int((base_corner_bl[1] + base_corner_br[1])/2)]

    # Skeleton pts
    skel_px = np.flip(np.argwhere(skeleton>0))

    # Find skeleton points closest to base and end-effector
    distance_from_base = (robot_base[0]-skel_px[:,0])**2 + (robot_base[1] - skel_px[:,1])**2
    distance_from_ee = (ee_marker[0]-skel_px[:,0])**2 + (ee_marker[1] - skel_px[:,1])**2

    # pts with min distance
    start = tuple(skel_px[np.argmin(distance_from_base)])
    end = tuple(skel_px[np.argmin(distance_from_ee)])

    return start, end, robot_base, skel_px


# Spline fit method
def fit_spline(robot_base, reconstructed_skeleton):
    # Downsampling skeleton for spline fit
    n = k + num_of_segments - 1     # 1/downsample factor
    seg_size = int(len(reconstructed_skeleton)/n) - 1       
    downsampled_skeleton_x = []
    downsampled_skeleton_y = []
    for i in range(n+1):
        downsampled_skeleton_x.append(reconstructed_skeleton[i*seg_size][0])
        downsampled_skeleton_y.append(reconstructed_skeleton[i*seg_size][1])

    # Fitting the Spline
    tck, u_params = splprep([downsampled_skeleton_x, downsampled_skeleton_y], k = k, s = 0)

    # Evaluating the spline
    new_params = np.linspace(0,1,25)
    new_pts = splev(new_params, tck)
    spline_x = new_pts[0].tolist()
    spline_y = new_pts[1].tolist()
  
    # Extracting the control points
    cx = tck[1][0]  
    cx = cx.tolist()
    cy = tck[1][1]
    cy = cy.tolist()
  
    # Constraining spline by constraining first and last control points
    cx[0] = robot_base[0]
    cy[0] = robot_base[1]

    cx[-1] = ee_marker[0]
    cy[-1] = ee_marker[1]

    # List of control points
    cp = []
    for i in range(len(cx)-1):
      cp.append(cx[i+1]) 
      cp.append(cy[i+1])
    
    # Drawing spline
    for i in range(len(spline_x)):
        cv2.circle(cur_img, (int(spline_x[i]), int(spline_y[i])), 3, (89,17,212), -1)

    # Drawing skeleton
    for pixel in reconstructed_skeleton:
        cv2.circle(cur_img, (pixel[0],pixel[1]), 1, (255,255,255), -1)

    # Drawing control points
    for i in range(len(cx)):
        cv2.circle(cur_img, (int(cx[i]), int(cy[i])), 5, (255, 133, 26), -1)

    # # overlaying ref
    # if ref_img is not None:
    #     cur_img = cv2.addWeighted(cur_img,1.0,ref_img, 0.5, 0)

    # Publish control points
    feature_msg = Float64MultiArray()
    feature_msg.data = cp
    feature_pub.publish(feature_msg)

    # Publish skeleton
    skel_msg = bridge.cv2_to_imgmsg(cur_img, "bgr8")
    curve_pub.publish(skel_msg)


# Clothoid fit method
def fit_clothoid():
    global count, cur_img

    # Start point from base marker
    x0 = int(base_corner_bl[0] + base_corner_br[0])/2
    y0 = int(base_corner_bl[1] + base_corner_br[1])/2

    # End point from end-effector marker
    xe = int(ee_corner_tl[0] + ee_corner_tr[0])/2
    ye = int(ee_corner_tl[1] + ee_corner_tr[1])/2

    # Taking 2 corners of end-effector marker for tangent calculation
    x1 = ee_corner_tr[0]
    y1 = ee_corner_tr[1]
    x2 = ee_corner_br[0]
    y2 = ee_corner_br[1]

    # Compute the tangents
    t0 = pi/2
    t1 = math.atan2(y2-y1, x2-x1)
    
    # G1 fit for clothoid
    cur_curve = Clothoid.G1Hermite(x0, y0, t0, xe, ye, t1, tol=1e-10)
    
    # Extract clothoid params (features)
    # dk = cur_curve.dk
    l = cur_curve.length
    k0 = cur_curve.KappaStart
    # k1 = cur_curve.KappaEnd

    # Visualization
    X,Y = cur_curve.SampleXY(50)

    # Overlay reference
    # if ref_img is not None:
    cur_img = cv2.addWeighted(cur_img,1.0,ref_img, 0.5, 0)

    # Draw curve points on downsampled image
    for i in range(len(X)):
        cv2.circle(cur_img, (int(X[i]), int(Y[i])), 3, (89,17,212), -1)

    # Draw input points
    cv2.circle(cur_img, (int(x0), int(y0)), 3, (255,255,255),-1)
    cv2.circle(cur_img, (int(xe), int(ye)), 3, (255,255,255), -1)
    
    # Publish clothoid image
    # curve_image = cv2.pyrUp(cur_img)
    # cv2.imwrite((str(count)+".png"), cur_img)
    curve_image = bridge.cv2_to_imgmsg(cur_img, "bgr8")
    curve_pub.publish(curve_image)
    # count += 1

    # Publish features
    feature_msg = Float64MultiArray()
    feature_msg.data = [l, k0, xe, ye]
    feature_pub.publish(feature_msg)

def main():
    global curve_type, k, num_of_segments, ref_img
    rospy.init_node('curve_fit')

    # Read parameters from param server
    control_rate = rospy.get_param("origami_skeleton_vs/control_rate")
    curve_type = rospy.get_param("origami_skeleton_vs/curve_type")
    num_of_segments = rospy.get_param("origami_skeleton_vs/num_of_segments")
    k = rospy.get_param("origami_skeleton_vs/degree")
    ref_img = cv2.imread("ref_img.jpg")

    r = rospy.Rate(control_rate)

    # Wait for the camera
    rospy.sleep(15)

    # Subscribers
    depth_img_sub = rospy.Subscriber("camera/aligned_depth_to_color/image_raw", Image, getDepthImage, queue_size = 1)
    marker_sub = rospy.Subscriber("origami_vs/aruco/pose", Float64MultiArray, getMarkers, queue_size=1)
    img_sub = rospy.Subscriber("camera/color/image_raw", Image, getImage, queue_size=1)
    # ref_img_sub = rospy.Subscriber("origami_vs/ref_img", Image, getRefImage, queue_size=1)
    
    while not rospy.is_shutdown():

        if cur_img is not None:
            fit_clothoid()
        r.sleep()
    
    # rospy.spin()


if __name__ == "__main__":
    main()
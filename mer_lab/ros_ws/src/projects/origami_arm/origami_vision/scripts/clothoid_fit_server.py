#!/usr/bin/env python3

# ROS deps
import rospy
from std_msgs.msg import Float64MultiArray

# Math
from numpy import pi
import numpy as np
import math

# Curve fitting and processing libs
from pyclothoids import Clothoid

# Services
from origami_vision.srv import curve_msg, curve_msgResponse
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
feature_type = ""

# flag to set base marker
base_marker_flag = False

# number of features
num_features = 0
num_points = 0

# Publishers
feature_pub = rospy.Publisher('origami_vs/feature_pub', Float64MultiArray, queue_size=1)
clothoid_pub = rospy.Publisher('origami_vs/curve_points', Float64MultiArray, queue_size=1)

# aruco marker callback
def getMarkers(msg):
    global ee_marker, base_marker, base_corner_br, base_corner_bl
    global ee_corner_br, ee_corner_bl, ee_corner_tl, ee_corner_tr
    global base_marker_flag
    
    ee_marker = msg.data[0:2]
    if not base_marker_flag:
        base_marker = msg.data[2:4]
        base_corner_br = msg.data[6:8]      # bottom right
        base_corner_bl = msg.data[8:10]     # bottom left
        base_marker_flag = True
    ee_corner_br = msg.data[10:12]      # bottom right
    ee_corner_bl = msg.data[12:14]      # bottom left
    ee_corner_tl = msg.data[14:16]      # top left
    ee_corner_tr = msg.data[16:18]      # top right

# clothoid fit service callback
def fitClothoid(msg):
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

    ################## old features using curve parameters ######################
    # Extract clothoid params (features)
    # l = cur_curve.length
    # k0 = 10000*cur_curve.KappaStart
    # # k0 = cur_curve.KappaStart
    # k1 = 10000*cur_curve.KappaEnd
    # dk = 100000*cur_curve.dk

    # curve_features.data = [l,k0,k1,xe,ye]
    # curve_features.data = [k0,k1,xe,ye]
    # curve_features.data = [dk,k0,k1,xe,ye]
    # curve_features.data = [l,k1,xe,ye]
    # curve_features.data = [l,k0,k1,dk,x0,y0,xe,ye]  # all features
    # curve_features.data = [l,k0,base_marker[0],base_marker[1]]
    # curve_features.data = [xe,ye]
    # print("base marker: ", x0,", ",y0)
    # print("base_marker: ", base_marker )
    ################  old features using curve parameters ##########################

    # Compute curve parameters
    l = cur_curve.length
    k0 = cur_curve.KappaStart
    k1 = cur_curve.KappaEnd
    dk = cur_curve.dk

    # Sample arc length
    if feature_type == "kappa":
        s = np.linspace(0, l, num_points+1)
    else:
        s = np.linspace(0, l, num_features+1)
    
    s = s[1:]   # removing first point

    clothoid_pts = []
    clothoid_pts.clear()

    kappa = []
    kappa.clear()

    curve_features = Float64MultiArray()

    # Compute features and fill response
    if feature_type == "kappa":
        # kappa.append(l)
        for s_i in s:    
            kappa.append(cur_curve.X(s_i))
            kappa.append(cur_curve.Y(s_i))
            kappa.append(10000*(k0 + s_i*dk))
        curve_features.data = kappa
    
    elif feature_type == "points":
        for s_i in s:
            clothoid_pts.append(cur_curve.X(s_i))
            clothoid_pts.append(cur_curve.Y(s_i))
        curve_features.data = clothoid_pts
    
    elif feature_type == "curve_parameters":
        curve_parameters = [l, 10000*k0, 10000*k1, xe, ye]
        curve_features.data = curve_parameters

    # Visualization data
    X,Y = cur_curve.SampleXY(50)
    curve_points = Float64MultiArray()
    curve_points.data.clear()
    for i in range(len(X)):
        curve_points.data.append(X[i])
        curve_points.data.append(Y[i])
    
    # Log data
    clothoid_pub.publish(curve_points)
    feature_pub.publish(curve_features)

    return curve_msgResponse(curve_features)

def main():
    global num_features, feature_type, num_points
    rospy.init_node('clothoid_fit')
    
    # Subscribers
    marker_sub = rospy.Subscriber("origami_vs/aruco/pose", Float64MultiArray, getMarkers, queue_size=1)

    # Read params
    num_features = int((rospy.get_param("origami_skeleton_vs/no_of_features"))/2)
    feature_type = rospy.get_param("origami_skeleton_vs/feature_type")
    num_points = rospy.get_param("origami_skeleton_vs/no_of_points")

    # Service server
    clothoid_server = rospy.Service('clothoid_fit_srv', curve_msg, fitClothoid)


    print("Initialized clothoids")

    rospy.spin()


if __name__ == '__main__':
    main()



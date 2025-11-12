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
from pyclothoids import Clothoid

# Global CV bridge object
bridge = CvBridge() 

# global image object(s)
cur_img = None
ref_img = None

# curve objects
X = []
Y = []

# feature objects
x = []
y = []

num_features = 0
feature_type = ""

color = [(26,255,26),(146,0,75),(10,194,255),(0,97,230)]

# Global publisher object
curve_pub = rospy.Publisher("origami_vs/curve_image", Image, queue_size=1)

def getCurve(msg):
    global X,Y
    X.clear()
    Y.clear()
    for i in range(50):
        X.append(msg.data[i*2])
        Y.append(msg.data[(i*2)+1])

def getFeatures(msg):
    global x, y
    x.clear()
    y.clear()
    for i in range(num_features):
        x.append(msg.data[i*2])
        y.append(msg.data[(i*2)+1])



def drawCurve(msg):
    global cur_img, bridge, ref_img
    cur_img = bridge.imgmsg_to_cv2(msg, "bgr8")

    overlayed_img = cv2.addWeighted(cur_img,1.0,ref_img, 0.5, 0)
    
    # draw curve
    for i in range(len(X)):
        cv2.circle(overlayed_img, (int(X[i]), int(Y[i])), 3, (255,133,26), -1)
    
    # draw features
    if feature_type == "points":
        for i in range(len(x)):
            cv2.circle(overlayed_img, (int(x[i]),int(y[i])), 7, color[i], -1)    
    
    imgmsg = bridge.cv2_to_imgmsg(overlayed_img, "bgr8")
    curve_pub.publish(imgmsg)


def main():
    global cur_img,bridge, ref_img, num_features, feature_type

    rospy.init_node('curve_visualizer')

    # Read params
    control_rate = rospy.get_param("origami_skeleton_vs/control_rate")
    num_features = int((rospy.get_param("origami_skeleton_vs/no_of_features"))/2)
    feature_type = rospy.get_param("origami_skeleton_vs/feature_type")

    ref_img = cv2.imread("ref_img.jpg")

    r = rospy.Rate(control_rate)

    # Subscribers
    curve_sub = rospy.Subscriber("origami_vs/curve_points", Float64MultiArray, getCurve, queue_size=1)
    img_sub = rospy.Subscriber("camera/color/image_raw", Image, drawCurve, queue_size=1)
    # marker_sub = rospy.Subscriber("origami_vs/aruco/result", Image, drawCurve, queue_size=1)
    feature_sub = rospy.Subscriber("origami_vs/feature_pub", Float64MultiArray, getFeatures, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    main()

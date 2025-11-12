#!/usr/bin/env python3
import cv2
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from numpy import pi
import numpy as np
from pyclothoids import Clothoid

bridge = CvBridge()
cur_img = None

goal_position = []
target_vec = 0

# Publishers
feature_pub = rospy.Publisher('origami_vs/feature_pub', Float64MultiArray, queue_size=1)
clothoid_pub = rospy.Publisher('origami_vs/curve_points', Float64MultiArray, queue_size=1)

# global marker objects
ee_marker = []      # list for storing ee co-ordinates
base_marker = []    # list for storing base co-ordinates
base_corner_br = []
base_corner_bl = []
ee_corner_tl = []
ee_corner_tr = []
ee_corner_br = []
ee_corner_bl = []

# number of features & type
num_features = 0
num_points = 0
feature_type = ""

# flag to set base marker
base_marker_flag = False

def getImage(msg):
    global cur_img
    cur_img = bridge.imgmsg_to_cv2(msg, "bgr8")

def click_event(event, x, y, flags, params):
    global goal_position, target_vec
    if event == cv2.EVENT_LBUTTONDOWN:
        goal_position.clear()
        goal_position.append(x)
        goal_position.append(y)
        print(goal_position)
        target_vec = float(input("Enter a target vector in rads: "))
        print ("received: ", target_vec)
        
        # Fit clothoid
        fitClothoid(target_vec)

def getMarker(msg):
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

def fitClothoid(t1):
    # Start point from base marker
    x0 = int(base_corner_bl[0] + base_corner_br[0])/2
    y0 = int(base_corner_bl[1] + base_corner_br[1])/2

    # End point from image click
    xe = goal_position[0]
    ye = goal_position[1]

    # tangents
    t0 = pi/2
    # we already have t1 from user

    # G1 fit for clothoid
    cur_curve = Clothoid.G1Hermite(x0, y0, t0, xe, ye, t1, tol=1e-10)

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
        print(kappa)
    elif feature_type == "points":
        for s_i in s:
            clothoid_pts.append(cur_curve.X(s_i))
            clothoid_pts.append(cur_curve.Y(s_i))
        curve_features.data = clothoid_pts
    
    elif feature_type == "curve_parameters":
        curve_parameters = [l, 10000*k0, 10000*k1, xe, ye]
        curve_features.data = curve_parameters
    
    cur_features = curve_features.data
    # write features to yaml
    print("Writing to yaml")
    yaml_file = open("origami_features.yaml","w")
    
    stext = ""
    stext += "origami_skeleton_vs:\n"
    stext += "  goal_features: [" + (','.join(map(str,cur_features))) + "]\n"

    yaml_file.write(stext)
    yaml_file.close()

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

    # Write curve to file
    ref_img = np.zeros([480,640,3],dtype=np.uint8)
    ref_img.fill(75)

    # draw ref curve
    for i in range(50):
        cv2.circle(ref_img, (int(curve_points.data[i*2]), int(curve_points.data[i*2+1])), 4, (146,0,75), -1)
    
    # draw features
    for s_i in s:
        cv2.circle(ref_img,(int(cur_curve.X(s_i)),int(cur_curve.Y(s_i))), 7,(26,255,26),-1)

    cv2.imwrite("ref_img.jpg", ref_img)    

def main():
    rospy.init_node('reference_clothoid')
    global num_features, feature_type, num_points
    # Read params
    num_features = int((rospy.get_param("origami_skeleton_vs/no_of_features"))/2)
    feature_type = rospy.get_param("origami_skeleton_vs/feature_type")
    num_points = rospy.get_param("origami_skeleton_vs/no_of_points")

    # Subscribers
    img_sub = rospy.Subscriber("camera/color/image_raw", Image, getImage, queue_size=1)
    marker_sub = rospy.Subscriber("origami_vs/aruco/pose",Float64MultiArray, getMarker, queue_size=1)
    while not rospy.is_shutdown():
        # Get goal position
        if cur_img is not None:
            cv2.imshow('current_image', cur_img)
            cv2.setMouseCallback('current_image', click_event)
            cv2.waitKey(1)
        
    rospy.spin()

if __name__ == "__main__":
    main()
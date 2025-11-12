#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

from origami_vision.srv import curve_msg

bridge = CvBridge() # bridge object

cur_img = None
ref_raw = None
X = []
Y = []

color = [(26,255,26),(146,0,75),(10,194,255),(0,97,230)]

def drawCurve(msg):
    global cur_img, bridge, ref_raw
    cur_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    ref_raw = bridge.imgmsg_to_cv2(msg, "bgr8")

    # draw curve
    for i in range(len(X)):
        cv2.circle(cur_img, (int(X[i]), int(Y[i])), 3, (89,17,212), -1)

def getCurve(msg):
    global X,Y
    X.clear()
    Y.clear()
    for i in range(50):
        X.append(msg.data[i*2])
        Y.append(msg.data[(i*2)+1])

def main():
    global cur_img
    rospy.init_node('reference_gen')

    # Subscribers
    img_sub = rospy.Subscriber("camera/color/image_raw", Image, drawCurve, queue_size=1)
    curve_sub = rospy.Subscriber("origami_vs/curve_points", Float64MultiArray, getCurve, queue_size=1)
    r = rospy.Rate(10)

    # Service client
    rospy.wait_for_service("clothoid_fit_srv")
    fit_clothoid = rospy.ServiceProxy('clothoid_fit_srv', curve_msg)

    print("waiting for features")
    rospy.sleep(20)
    count = 0
    while not rospy.is_shutdown():
        if cur_img is not None:
            response = fit_clothoid()
            cur_features = response.curve_features.data
            
            # write features to yaml
            print("Writing to yaml")
            yaml_file = open("origami_features.yaml","w")
            
            s = ""
            s += "origami_skeleton_vs:\n"
            s += "  goal_features: [" + (','.join(map(str,cur_features))) + "]\n"

            yaml_file.write(s)
            yaml_file.close()
            
            # draw features
            for i in range(int(len(response.curve_features.data)/2)):
                cv2.circle(cur_img, (int(response.curve_features.data[i*2]), int(response.curve_features.data[i*2+1])), 7, color[i],-1)
            
            # write goal image
            cv2.imwrite("ref_img.jpg", cur_img)
            cv2.imwrite("ref_raw.jpg", ref_raw)
            count = count + 1

            if count >3:
                rospy.signal_shutdown("Reference generated")
            
            r.sleep()
    # rospy.spin()


if __name__ == "__main__":
    main()
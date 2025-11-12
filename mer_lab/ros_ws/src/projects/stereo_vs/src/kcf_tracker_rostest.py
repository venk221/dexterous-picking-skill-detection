#!/usr/bin/env python3

# imports required for opencv KCF
import cv2
import sys

# imports required for ROS bridge
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
import numpy as np
import math

import threading

# set up necessary global objects:
bridge = CvBridge()
# tracker = cv2.TrackerKCF_create()  # KCF object tracker
tracker = cv2.TrackerCSRT_create()
multiTracker = cv2.MultiTracker_create()  # create the multitracker for KCF
# create the right image multitracker for KCF
rightMultiTracker = cv2.MultiTracker_create()
state_running = 0

# set up the feature tracking messages:
lhandx = Float64()
lhandy = Float64()
rhandx = Float64()
rhandy = Float64()
ltargetx = Float64()
ltargety = Float64()
rtargetx = Float64()
rtargety = Float64()
# KCF-specific feature tracking messages:
lfingerx = Float64()
lfingery = Float64()
rfingerx = Float64()
rfingery = Float64()
lthumbx = Float64()
lthumby = Float64()
rthumbx = Float64()
rthumby = Float64()


# set up the error messages:
errorLeft = Pose()
errorRight = Pose()
lemMsg = Float64()
remMsg = Float64()

IMGENC = "bgr8"  # mono8, mono16, bgr8

# KCF tracker to produce the bounding box in each Image


def kcf_tracking(frame, isLeft):
    global colors, rcolors
    # tracker was initialized with x_bbox_init, thus local bbox is ok
    # (select trackers based on isLeft)
    if isLeft:
        ok, bboxes = multiTracker.update(frame)
    else:
        ok, bboxes = rightMultiTracker.update(frame)

    # draw bounding box for debugging
    # if ok:
    cX0 = 0
    cY0 = 0
    cXt = 0
    cYt = 0
    cXh = 0
    cYh = 0
    for i, newbox in enumerate(bboxes):
        p1 = (int(newbox[0]), int(newbox[1]))
        p2 = (int(newbox[0] + newbox[2]), int(newbox[1] + newbox[3]))
        cX = int((p1[0] + p2[0])/2)
        cY = int((p1[1] + p2[1])/2)
        if i == 0:
            cX0 = cX
            cY0 = cY
            if isLeft:
                lfingerx.data = cX
                lfingery.data = cY
            else:
                rfingerx.data = cX
                rfingery.data = cY
        if i == 1:
            cXh = int((cX0+cX)/2)
            cYh = int((cY0+cY)/2)
            if isLeft:
                lthumbx.data = cX
                lthumby.data = cY
            else:
                rthumbx.data = cX
                rthumby.data = cY
        if i == 2:
            cXt = cX
            cYt = cY
        if isLeft:
            cv2.circle(frame, (cX, cY), 5, colors[i], -1)
            cv2.rectangle(frame, p1, p2, colors[i], 2, 1)
            if i == 1:
                cv2.circle(frame, (cXh, cYh), 5, colors[i], -1)
            if i == 2:
                cv2.line(frame, (cXh, cYh), (cXt, cYt), (0, 0, 255), 5)
            # set up feature publishing to data log the centroids
            lhandx.data = cXh
            lhandy.data = cYh
            lhandx_pub.publish(lhandx)
            lhandy_pub.publish(lhandy)
            # set up float message for publishing debug of hand location in camera pixels
            ltargetx.data = cXt
            ltargety.data = cYt
            ltargetx_pub.publish(ltargetx)
            ltargety_pub.publish(ltargety)
            # publish previously set finger and thumb feature points
            lfingerx_pub.publish(lfingerx)
            lfingery_pub.publish(lfingery)
            lthumbx_pub.publish(lthumbx)
            lthumby_pub.publish(lthumby)

        else:
            cv2.circle(frame, (cX, cY), 5, rcolors[i], -1)
            cv2.rectangle(frame, p1, p2, rcolors[i], 2, 1)
            if i == 1:
                cv2.circle(frame, (int((cX0+cX)/2), int((cY0+cY)/2)), 5, rcolors[i], -1)
            if i == 2:
                cv2.line(frame, (cXh, cYh), (cXt, cYt), (0, 0, 255), 5)
            # set up feature publishing to data log the centroids
            rhandx.data = cXh
            rhandy.data = cYh
            rhandx_pub.publish(rhandx)
            rhandy_pub.publish(rhandy)
            # set up float message for publishing debug of hand location in camera pixels
            rtargetx.data = cXt
            rtargety.data = cYt
            rtargetx_pub.publish(rtargetx)
            rtargety_pub.publish(rtargety)
            # publish previously set finger and thumb feature points
            rfingerx_pub.publish(rfingerx)
            rfingery_pub.publish(rfingery)
            rthumbx_pub.publish(rthumbx)
            rthumby_pub.publish(rthumby)

    # else:
    #   cv2.putText(frame, "Tracking Failure!", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
    if isLeft:
        left_img_debug_pub.publish(bridge.cv2_to_imgmsg(frame, IMGENC))
        publish_centroid(cXh, cYh, cXt, cYt, frame, isLeft)
    else:
        right_img_debug_pub.publish(bridge.cv2_to_imgmsg(frame, IMGENC))
        publish_centroid(cXh, cYh, cXt, cYt, frame, isLeft)

#----------------------------------------------CENTROID/ERROR PUBLISHING-------------------------#


def publish_centroid(cXHand, cYHand, cX, cY, cv_image, is_left):
    # Error vector data for publishing
    Ex = (cX - cXHand)
    Ey = (cY - cYHand)
    Em = math.sqrt((cX-cXHand)**2+(cY-cYHand)**2)  # magnitude of the error

    if is_left:
        # publish left image plane error vector
        errorLeft.position.x = Ex
        errorLeft.position.y = Ey
        leftErrorPub.publish(errorLeft)
        # publish left image plane error vector magnitude
        lemMsg.data = Em
        leftErrorMagnitudePublisher.publish(lemMsg)
    else:
        # publish right image plane error vector
        errorRight.position.x = Ex
        errorRight.position.y = Ey
        rightErrorPub.publish(errorRight)
        # publish left image plane error vector magnitude
        remMsg.data = Em
        rightErrorMagnitudePublisher.publish(remMsg)


# left image callback
def leftcallback(data):
    global new_left, left_frame
    # convert the ros Image message to cv2 object (using mono since MultiSense is mono anyways)
    try:
        left_frame = bridge.imgmsg_to_cv2(data, IMGENC)  # mono8, mono16, bgr8
        new_left = True
    except CvBridgeError as e:
        print(e)


# right image callback
def rightcallback(data):
    global new_right, right_frame
    # convert the ros Image message to cv2 object (using mono since MultiSense is mono anyways)
    try:
        right_frame = bridge.imgmsg_to_cv2(data, IMGENC)  # mono8, mono16, bgr8
        new_right = True
    except CvBridgeError as e:
        print(e)

# TODO: add occlusion handling
# def init_occlusion_handling(tolerance):
# def occlusion_handler(cX, cY, isLeft)


# start by setting up ROS main framework
#-------------------------------------------------MAIN METHOD------------------------------------#
if __name__ == '__main__':
    # initialize ROS node
    global left_frame, right_frame, new_left, new_right
    rospy.init_node('error2DCalculation', anonymous=True)

    # SET SUBSCRIBER TOPIC and setup subscribers
    # Panda sim: "/left_camera/left_camera/camera/color/image_raw"
    LEFT_SUB_TOPIC = rospy.get_param('/left_camera_topic')
    # Panda sim: "/right_camera/right_camera/camera/color/image_raw"
    RIGHT_SUB_TOPIC = rospy.get_param('/right_camera_topic')

    rospy.Subscriber(LEFT_SUB_TOPIC, Image, leftcallback, queue_size=1)
    rospy.Subscriber(RIGHT_SUB_TOPIC, Image, rightcallback, queue_size=1)

    left_img_debug_pub = rospy.Publisher(
        "/kcf_left_centroid", Image, queue_size=1)
    right_img_debug_pub = rospy.Publisher(
        "/kcf_right_centroid", Image, queue_size=1)
    # publish image-plane error for left and right images
    leftErrorPub = rospy.Publisher("/Error_Left", Pose, queue_size=1)
    rightErrorPub = rospy.Publisher("/Error_Right", Pose, queue_size=1)
    leftErrorMagnitudePublisher = rospy.Publisher(
        "/vs_lemp", Float64, queue_size=1)
    rightErrorMagnitudePublisher = rospy.Publisher(
        "/vs_remp", Float64, queue_size=1)

    # set up feature tracking raw point data publishers
    lhandx_pub = rospy.Publisher("/vs_lhandx", Float64, queue_size=1)
    lhandy_pub = rospy.Publisher("/vs_lhandy", Float64, queue_size=1)
    rhandx_pub = rospy.Publisher("/vs_rhandx", Float64, queue_size=1)
    rhandy_pub = rospy.Publisher("/vs_rhandy", Float64, queue_size=1)
    ltargetx_pub = rospy.Publisher("/vs_ltargetx", Float64, queue_size=1)
    ltargety_pub = rospy.Publisher("/vs_ltargety", Float64, queue_size=1)
    rtargetx_pub = rospy.Publisher("/vs_rtargetx", Float64, queue_size=1)
    rtargety_pub = rospy.Publisher("/vs_rtargety", Float64, queue_size=1)
    # KCF-specific feature tracking raw point data publishers
    lfingerx_pub = rospy.Publisher("/vs_lfingerx", Float64, queue_size=1)
    lfingery_pub = rospy.Publisher("/vs_lfingery", Float64, queue_size=1)
    rfingerx_pub = rospy.Publisher("/vs_rfingerx", Float64, queue_size=1)
    rfingery_pub = rospy.Publisher("/vs_rfingery", Float64, queue_size=1)
    lthumbx_pub = rospy.Publisher("/vs_lthumbx", Float64, queue_size=1)
    lthumby_pub = rospy.Publisher("/vs_lthumby", Float64, queue_size=1)
    rthumbx_pub = rospy.Publisher("/vs_rthumbx", Float64, queue_size=1)
    rthumby_pub = rospy.Publisher("/vs_rthumby", Float64, queue_size=1)

    # define state because used in callbacks
    global state
    state = 0

    # instantiate frames and flags for callbacks
    left_frame = np.zeros((0, 0, 0))
    right_frame = np.zeros((0, 0, 0))
    new_left = False
    new_right = False

    # begin by acquiring the first image message for left and right cameras
    left_img_msg = rospy.wait_for_message(LEFT_SUB_TOPIC, Image,)
    right_img_msg = rospy.wait_for_message(RIGHT_SUB_TOPIC, Image,)
    # convert ROS image to opencv data structures
    left_frame_init = bridge.imgmsg_to_cv2(
        left_img_msg, IMGENC)  # mono8, mono16, bgr8
    right_frame_init = bridge.imgmsg_to_cv2(right_img_msg, IMGENC)

    # initialize data structures for storing bounding boxes and their colors
    bboxes = []
    rbboxes = []
    global colors, rcolors
    colors = []
    rcolors = []
    count = 1
    rcount = 1
    MAXPOINTS = 3  # starting off with centroids for just the hand and the target

    # set the loop rate:
    rate = rospy.Rate(120)  # 30Hz

    while not rospy.is_shutdown():
        # select the left hand data:
        if state == 0:
            if state_running == 0:
                state_running = 1
                # get initial regions of interest for hand and target: TODO: select two fingers

                # select hand and target in left image plane
                print("Please select the regions of interest in the left image frame")
                while len(bboxes) < MAXPOINTS:
                    bbox = cv2.selectROI(
                        left_frame_init, True, fromCenter=True)
                    bboxes.append(bbox)
                    # color the bounding boxes based on order boxes were selected for consistent reference colors
                    colors.append((255/2*count, 0, 255/2*count))
                    count = count + 1

                # reset the opencv windows before selecting right image points
                cv2.destroyAllWindows()

                # select hand and target in right image plane
                print("Please select the regions of interest in the right image frame")
                while len(rbboxes) < MAXPOINTS:
                    rbbox = cv2.selectROI(
                        right_frame_init, True, fromCenter=True)
                    rbboxes.append(rbbox)
                    # color the bounding boxes based on order boxes were selected for consistent reference colors
                    rcolors.append((0, 255/2*rcount, 255/2*rcount))
                    rcount = rcount + 1

                # left image multitracker initialization, adding KCF trackers for each bbox
                if len(bboxes) == MAXPOINTS:
                    for bbox in bboxes:
                        select_success = multiTracker.add(
                            cv2.TrackerKCF_create(), left_frame_init, bbox)
                else:
                    print("NOT ENOUGH BBOXES ADDED! TRY AGAIN")

                # right image multitracker initialization, adding KCF trackers for each bbox
                if len(rbboxes) == MAXPOINTS:
                    for rbbox in rbboxes:
                        select_success = rightMultiTracker.add(
                            cv2.TrackerKCF_create(), right_frame_init, rbbox)
                else:
                    print("NOT ENOUGH RBBOXES! TRY AGAIN!")

            # exit opencv windows if q is pressed:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()

            # point selected successfully, state complete and move to spinning updates
            if select_success:
                cv2.destroyAllWindows()
                print("Thank you for selecting the ROIs, moving to next state")
                state = 1
                state_running = 0

        if state == 1:
            # Now that initial frame and initial bbox have been selected, and tracker is initialized
            # move to spinning updates and using tracker on the actual data:
            threadlist = list()

            if new_left:
                left_img_thread = threading.Thread(
                    target=kcf_tracking, args=(left_frame, True,))
                threadlist.append(left_img_thread)
                left_img_thread.start()
                new_left = False

            if new_right:
                right_img_thread = threading.Thread(
                    target=kcf_tracking, args=(right_frame, False,))
                threadlist.append(right_img_thread)
                right_img_thread.start()
                new_right = False

            if state_running == 0:
                state_running = 1
                print("subbing and pubbing images")

            for thread in threadlist:
                thread.join()

            rate.sleep()

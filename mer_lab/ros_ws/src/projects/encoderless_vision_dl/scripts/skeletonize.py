#!/usr/bin/env python3

# This node skeletonizes depth images
import rospy
import cv2
import csv
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from skimage.morphology import skeletonize
from scipy.interpolate import splprep, splev
from astar import MyAstar
import time

bridge = CvBridge() # bridge object

q = 0
it = 0              # iterator
num_of_segments = 0 # spline segments
k = 0               # degree
control_rate = 0    # control rate
csvwriter = None
binary_image_pub = rospy.Publisher('origami_vs/binary_image', Image, queue_size=1)
skel_pub = rospy.Publisher('origami_vs/skeleton', Image, queue_size=1)
feature_pub = rospy.Publisher('origami_vs/control_points', Float64MultiArray, queue_size=1)

ee_marker = []      # list for storing ee co-ordinates
base_marker = []    # list for storing base co-ordinates
base_corner_br = []
base_corner_bl = []
ee_corner_tl = []
ee_corner_tr = []
ee_corner_br = []
ee_corner_bl = []
cur_img = None
ref_img = None
roi = None
cur_depth_img = None

# For finding intensity values in ROI
def region_of_interest():
    global cur_depth_img
    # ------------ ROI to find MAX and MIN in grayscale ----------
    print(cur_img.shape)        
    roi = cv2.selectROI(cur_img)

        # print("max: ", max(roi))
        # print("min: ", min(roi))

    # print(cur_depth_img.shape)
    cur_depth_img = cur_depth_img[roi[1]:roi[1]+roi[3], roi[0]:roi[0]+roi[2]]
    
    print("max: ", np.amax(cur_depth_img))
    print("min: ", np.amin(cur_depth_img))

# Get color image
def getImage(msg):
    global cur_img, bridge, q
    cur_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    cv2.imwrite("/home/jc-merlab/Pictures/Data/plotting_data/raw_images_shape/controlpoint_img_" + str(q) + ".png", cur_img )
    # print("rgb:",cur_img.shape)
    cur_img = cv2.pyrDown(cv2.pyrDown(cur_img))
    # print("rgb_d:",cur_img.shape)
    # cur_img = cv2.pyrDown(cur_img)

    q = q+1

# Get marker positions
def getMarkers(msg):
    global ee_marker, base_marker, base_corner_br, base_corner_bl
    global ee_corner_br, ee_corner_bl, ee_corner_tl, ee_corner_tr    

    ee_marker = msg.data[0:2]
    base_marker = msg.data[2:4]

    print("ee_marker", ee_marker)
    print("base_marker", base_marker)

    base_corner_br = msg.data[6:8]      # bottom right
    base_corner_bl = msg.data[8:10]     # bottom left
    ee_corner_br = msg.data[10:12]      # bottom right
    ee_corner_bl = msg.data[12:14]      # bottom left
    ee_corner_tl = msg.data[14:16]      # top left
    ee_corner_tr = msg.data[16:18]      # top right

def getRefImg(msg):
    global ref_img, bridge
    # print("Received ref img")
    ref_img = bridge.imgmsg_to_cv2(msg, "bgr8")


# Get depth image
def getDepthImage(msg):
    global bridge, cur_depth_img
    cur_depth_img = bridge.imgmsg_to_cv2(msg, "8UC1")
    # print("depth:",cur_depth_img.shape)
    cur_depth_img = cv2.pyrDown(cv2.pyrDown(cur_depth_img))
    # print("depth_d:",cur_depth_img.shape)
    # cur_depth_img = cv2.pyrDown(cur_depth_img)
    # cv2.imwrite("binary1.jpg", cur_depth_img)
    # skeleton_time_start = time.time()
    skeletonization()
    # skeleton_time_end = time.time()
    # print("skeletonize time: ", skeleton_time_end - skeleton_time_start)

# Constrain the skeleton to robot
def constrain_skeleton(skeleton):
    # start_t = time.time()
    # Center of base marker bottom edge
    robot_base = [int((base_corner_bl[0] + base_corner_br[0])/2), int((base_corner_bl[1] + base_corner_br[1])/2)]

    # Skeleton pts
    skel_px = np.flip(np.argwhere(skeleton>0))

    # Find skeleton points closest to base and end-effector
    distance_from_base = (robot_base[0]-skel_px[:,0])**2 + (robot_base[1] - skel_px[:,1])**2
    distance_from_ee = (ee_marker[0]-skel_px[:,0])**2 + (ee_marker[1] - skel_px[:,1])**2

    # pts with min distance
    start = tuple(skel_px[np.argmin(distance_from_base)])
    end = tuple(skel_px[np.argmin(distance_from_ee)])
    # end_t = time.time()

    # print("constrain:", end_t - start_t)

    return start, end, robot_base, skel_px

# Skeletonization process
def skeletonization():
    start_t = time.time()
    global bridge, it, cur_depth_img, cur_img
    # Debugging downsampling issue
    # cv2.circle(cur_img, (int(ee_marker[0]),int(ee_marker[1])), 5, [255, 133, 26], -1)
    # cv2.circle(cur_img, (int(base_marker[0]),int(base_marker[1])), 5, [255, 133, 26], -1)
    # cv2.imshow("color",cur_img)
    # cv2.waitKey(1)

    # roi(cur_img, cur_depth_img)
    # region_of_interest()
    # Binarize image and segment robot
    dmin = 2
    dmax = 5

    cur_depth_img = np.where((cur_depth_img<dmin) | (cur_depth_img>dmax), 0, cur_depth_img)
    # cv2.imwrite("first" + str(it) + ".jpg", cur_depth_img)

    cur_depth_img = np.where(cur_depth_img>0,255,cur_depth_img)
    # cv2.imwrite("second" +str(it) + ".jpg", cur_depth_img)
    # cv2.imshow("cur", cur_depth_img)
    # cv2.waitKey(1)
    # Segment robot mount and other annoyances
    # for x_i in range(cur_depth_img.shape[1]):
    #     for y_i in range(cur_depth_img.shape[0]):
    #         if y_i < base_marker[1]:
    #         # if y_i < max(base_corner_bl[1], base_corner_br[1]):
    #             cur_depth_img[y_i, x_i] = 0
    #         elif y_i > max(ee_corner_bl[1], ee_corner_br[1]):
    #             cur_depth_img[y_i, x_i] = 0 
    # cv2.imwrite("third" + str(it) + ".jpg", cur_depth_img)
    
    # cv2.imshow("seg", cur_depth_img)
    # cv2.waitKey(1)

    kernel_dilate = np.ones((5, 5), np.uint8)
    kernel_erode = np.ones((5,5), np.uint8)

    cur_depth_img = cv2.dilate(cur_depth_img, kernel_dilate, iterations = 3)
    # cv2.imwrite("fifth" + str(it) + ".jpg", cur_depth_img)
    
    cur_depth_img = cv2.erode(cur_depth_img, kernel_erode, iterations = 1)    
    # cv2.imwrite("sixth"+str(it) + ".jpg", cur_depth_img)
    
    cur_depth_img = cv2.GaussianBlur(cur_depth_img, (9,9), 0)  
    # cv2.imwrite("seventh"+str(it)+".jpg", cur_depth_img)
    # cv2.imshow("blur", cur_depth_img)
    # cv2.waitKey(1)
    # Skeletonize robot
    cur_depth_img = np.where(cur_depth_img == 255, 1, cur_depth_img)
    skeleton = skeletonize(cur_depth_img, method='lee')
    skeleton = np.where(skeleton >0, 255, skeleton)
    cv2.imwrite("skeleton_img" + (str(it)).zfill(3) + ".jpg", skeleton)
    # cv2.imshow("skel",skeleton)
    # cv2.waitKey(1)
    # Constrain skeleton
    start, goal, robot_base, skel_px = constrain_skeleton(skeleton)
    
    # Recontruct skeleton
    a_star = MyAstar(start, goal, skeleton.shape[0], skeleton.shape[1], skeleton)
    if(a_star.IsValid(start[0], start[1])):
        if(a_star.IsValid(goal[0], goal[1])):
            if(a_star.IsObstacle(start[0],start[1]) == False):
                if(a_star.IsObstacle(goal[0], goal[1]) == False):
                    (exploredStates, backtrackStates, distanceFromStartToGoal) = a_star.search()

                    if(distanceFromStartToGoal == float('inf')):
                        print("\nNo optimal path found.")
                    else:
                        pass
                        # print("\nOptimal path found. Distance is " + str(distanceFromStartToGoal))
                else:
                    print("The entered goal node is an obstacle ")
                    print("Please check README.md file for running astar.py file.")
            else:
                print("The entered start node is an obstacle ")
                print("Please check README.md file for running astar.py file.")
        else:
            print("The entered goal node outside the map ")
            print("Please check README.md file for running astar.py file.")
    else:
        print("The entered start node is outside the map ")
        print("Please check README.md file for running astar.py file.")
    
    # Downsampling skeleton for spline fit
    n = k + num_of_segments - 1     # 1/downsample factor
    seg_size = int(len(backtrackStates)/n) - 1       
    downsampled_skeleton_x = []
    downsampled_skeleton_y = []
    for i in range(n+1):
        downsampled_skeleton_x.append(backtrackStates[i*seg_size][0])
        downsampled_skeleton_y.append(backtrackStates[i*seg_size][1])

    # end_t = time.time()
    # print("Astar:", end_t-start_t)

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

    print("control_points", cp)
    csvwriter.writerow(cp)
    

    # Drawing skeleton stuff
    # cv2.circle(cur_img, (robot_base[0], robot_base[1]), 5, (98,254,254),-1)
    # cv2.circle(cur_img, (start[0], start[1]), 5, (183,95,211), -1)

    # cv2.circle(cur_img, (int(ee_marker[0]), int(ee_marker[1])), 5, (106,190,225),-1)
    # cv2.circle(cur_img, (goal[0], goal[1]), 5, (166,176,64), -1)

    # Drawing spline
    # for i in range(len(spline_x)):
    #     cv2.circle(cur_img, (int(spline_x[i]), int(spline_y[i])), 3, (89,17,212), -1)

    # Drawing skeleton
    # for pixel in backtrackStates:
    #     cv2.circle(cur_img, (pixel[0],pixel[1]), 1, (255,255,255), -1)

    # Drawing control points
    for i in range(len(cx)):
        cv2.circle(cur_img, (int(cx[i]), int(cy[i])), 5, (255, 133, 26), -1)

    cv2.imwrite("/home/jc-merlab/Pictures/Data/skel_test/outimage_" + (str(it)) + ".jpg", cur_img)

    # overlaying ref
    if ref_img is not None:
        cur_img = cv2.addWeighted(cur_img,1.0,ref_img, 0.5, 0)

    # cv2.imwrite("rgb_skel_spline"+(str(it)).zfill(3) + ".jpg", cur_img)

    # Publish control points
    feature_msg = Float64MultiArray()
    feature_msg.data = cp
    feature_pub.publish(feature_msg)

    # Publish skeleton
    skel_msg = bridge.cv2_to_imgmsg(cur_img, "bgr8")
    cur_depth_img_msg = bridge.cv2_to_imgmsg(skeleton, "8UC1")

    cur_img = cv2.pyrUp(cv2.pyrUp(cur_img))

    # cv2.imwrite("/home/jc-merlab/Pictures/Data/skel_test/spline_" + (str(it)).zfill(3) + ".jpg", cur_img)
    # cv2.imwrite("/home/jc-merlab/Pictures/Data/skel_test/outimage_" + (str(it)) + ".jpg", cur_img)
    
    binary_image_pub.publish(cur_depth_img_msg)
    skel_pub.publish(skel_msg)

    # Write image
    # filename = str(it) + ".png"
    # cv2.imwrite(filename,img)

    # # Increment iterator for image name
    it = it+1

def main():
    global num_of_segments, k, control_rate, csvwriter
    # Initialize ROS
    rospy.init_node('depth_image_writer')
    
    num_of_segments = rospy.get_param("origami_skeleton_vs/num_of_segments")
    k = rospy.get_param("origami_skeleton_vs/degree")
    control_rate = rospy.get_param("origami_skeleton_vs/control_rate")
    r = rospy.Rate(control_rate)
    # Wait for the camera
    rospy.sleep(15)
    
    # Subscribers
    depth_img_sub = rospy.Subscriber("camera/aligned_depth_to_color/image_raw", Image, getDepthImage, queue_size = 1)
    marker_sub = rospy.Subscriber("origami_vs/aruco/pose", Float64MultiArray, getMarkers, queue_size=1)
    img_sub = rospy.Subscriber("camera/color/image_raw", Image, getImage, queue_size=1)
    ref_img_sub = rospy.Subscriber("origami_vs/ref_img", Image, getRefImg, queue_size=1)

    filename = "/home/jc-merlab/Pictures/Data/plotting_data/controlpoints.csv"
    col_name = ['cp1x', 'cp1y', 'cp2x', 'cp2y', 'cp3x', 'cp3y']

    f = open(filename, 'w')
    csvwriter = csv.writer(f)
    csvwriter.writerow(col_name)


    rospy.spin()

if __name__ == "__main__":
    main()
#!/usr/bin/env python3
import numpy as np
import sys
from pyrsistent import m
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt
import rospy
import cv2
import cv2.aruco as aruco
import math
import time
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray
# import pyrealsense2 as rs
from cv_bridge import CvBridge, CvBridgeError
from skimage.morphology import skeletonize
bridge = CvBridge()

ee_center_x = 0
ee_center_y = 0

base_center_x = 0
base_center_y = 0

ros_depth_img = None
cv_img = None

marker_base_flag = False
marker_ee_flag = False
ee_corners_list = []
base_corners_list = []

i = 0
j = 0
k = 0
l = 0
m = 0
n = 0
p = 0

# def ee_callback(ee):
#     global ee_center_x, ee_center_y

#     ee_center_x = ee.data[0]
#     ee_center_y = ee.data[1]

#     # print("EE Center", ee_center_x, ee_center_y)

# def base_callback(base):
#     global base_center_x, base_center_y

#     base_center_x = base.data[0]
#     base_center_y = base.data[1]

#     # print("Base Center", base_center_x, base_center_y)


def marker_callback(img_msg):
    global bridge, ros_img, ee_center_x, base_center_x, ee_center_y, base_center_y, marker_base_flag,\
        marker_ee_flag, ee_corners_list, base_corners_list, cv_img, p
    # if p == 20:
    cv_img = bridge.imgmsg_to_cv2(img_msg, 'bgr8')

    gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    arucoParameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(
        gray, aruco_dict, parameters=arucoParameters)
    # print("Id generated", ids) #Remove
    id_list = []
    id_list.clear()
    for i in ids:
        if int(i) == 32:
            id_list.append(int(i))
        elif int(i) == 34:
            id_list.append(int(i))
    # Aruco ID for each marker
    base_id = 34
    ee_id = 32
    # Check if both markers are found
    try:
        base_index = id_list.index(base_id)
        marker_base_flag = True
    except:
        marker_base_flag = False
    try:
        ee_index = id_list.index(ee_id)
        marker_ee_flag = True
    except:
        marker_ee_flag = False
    # if not marker_flag:
    #     print("MARKER NOT FOUND")
    # Separating the corner pixel co-ordinates for each marker
    if (marker_base_flag) and (marker_ee_flag):
        ee_corners_list = corners[ee_index].reshape(4, 2)
        base_corners_list = corners[base_index].reshape(4, 2)
    # print("ee corner list", ee_corners_list) #Remove
    # Averaging base corner co-ordinates to obtain marker center
    base_center_x = (base_corners_list[0][0] + base_corners_list[1]
                     [0] + base_corners_list[2][0] + base_corners_list[3][0])/4
    base_center_y = (base_corners_list[0][1] + base_corners_list[1]
                     [1] + base_corners_list[2][1] + base_corners_list[3][1])/4
    base_center = [base_center_x, base_center_y]
    # print(base_center)
    # Averaging ee corner co-ordinates to obtain marker center
    ee_center_x = (ee_corners_list[0][0] + ee_corners_list[1]
                   [0] + ee_corners_list[2][0] + ee_corners_list[3][0])/4
    ee_center_y = (ee_corners_list[0][1] + ee_corners_list[1]
                   [1] + ee_corners_list[2][1] + ee_corners_list[3][1])/4
    ee_center = [ee_center_x, ee_center_y]
    cv2.circle(cv_img, (int(ee_center_x), int(
        ee_center_y)), 4, [255, 255, 0], -1)
    cv2.circle(cv_img, (int(base_center_x), int(
        base_center_y)), 4, [255, 255, 0], -1)

    p += 1


def image_callback(img):
    global bridge, ros_depth_img, i, j, k, l, m, n

    # if cv_img is not None:
    #     cv2.imwrite('rgb_marker'+str(l)+'.png', cv_img)

    cv_depth_img = bridge.imgmsg_to_cv2(img, '8UC1')
    # cv_depth_img = cv2.cvtColor(cv_depth_img, cv2.COLOR_GRAY2BGR)

    if ee_center_x != 0.0 and ee_center_y != 0.0 and base_center_x != 0.0 and base_center_y != 0.0:
        cv2.circle(cv_depth_img, (int(base_center_x), int(
            base_center_y)), 5, [255, 255, 255], -1)
        cv2.circle(cv_depth_img, (int(ee_center_x), int(
            ee_center_y)), 5, [255, 255, 255], -1)

    # r = cv2.selectROI(cv_depth_img)
    # print(r[0], r[1], r[2], r[3])
    # depth_roi = cv_depth_img[r[1]:r[1]+r[3], r[0]:r[0]+r[2]]
    # print(depth_roi[0])
        # 5 3 1125 671
        # 54 3 976 668
        # 50 8 1067 667
        # 59 7 1073 668
        # 36 5 1099 671

    # r = rospy.get_param('vsbot/depth_baseline/cropper')
    # cv_depth_img = cv_depth_img[r[1]:r[1]+r[3], r[0]:r[0]+r[2]]

    cv2.imwrite('marker'+str(j)+'.png', cv_depth_img)

    dmin = rospy.get_param('vsbot/depth_baseline/dmin')
    dmax = rospy.get_param('vsbot/depth_baseline/dmax')

    cv_depth_img = np.where((cv_depth_img <= dmin) | (
        cv_depth_img > dmax), 0, cv_depth_img)

    depth = cv_depth_img
    depth = cv2.cvtColor(depth, cv2.COLOR_GRAY2BGR)
    k1 = rospy.get_param('vsbot/depth_baseline/kernel1')
    k2 = rospy.get_param('vsbot/depth_baseline/kernel2')
    kernel = np.ones((k1, k2), np.uint8)
    depth = cv2.GaussianBlur(depth, (5, 5), 0)
    depth = cv2.dilate(depth, kernel, iterations=3)
    depth = cv2.erode(depth, kernel, iterations=2)
    depth = cv2.morphologyEx(depth, cv2.MORPH_OPEN, kernel)

    tstart = time.time()
    # cv2.imwrite('binary'+str(j)+'.png', depth)
    binary = np.where(depth > 0, 1, depth)
    skeleton = skeletonize(binary, method='lee')
    skeleton = cv2.cvtColor(skeleton, cv2.COLOR_BGR2GRAY)
    skeleton = np.where(skeleton == 1, 255, skeleton)

    # cv2.imshow("Image_test", skeleton)
    # cv2.waitKey(1)

    tend = time.time()

    ttime = tend - tstart

    print("Skeletonization timing", ttime)

#     # skeleton = cv2.cvtColor(skeleton, cv2.COLOR_GRAY2BGR)

#     # if ee_center_x != 0.0 and ee_center_y !=0.0 and base_center_x != 0.0 and base_center_y != 0.0:
#     #     cv2.circle(skeleton, (int(base_center_x), int(base_center_y)), 3, [0, 255, 255], -1)
#     #     cv2.circle(skeleton, (int(ee_center_x), int(ee_center_y)), 3, [255, 0, 0], -1)

#     # cv2.imwrite('orig_skel'+str(k)+'.png', skeleton)

# #     cv2.circle(skeleton, (int(base_center_x), int(base_center_y)), 5, [0, 0, 0], -1)
# #     cv2.circle(skeleton, (int(ee_center_x), int(ee_center_y)), 5, [0, 0, 0], -1)
# #     skeleton = cv2.cvtColor(skeleton, cv2.COLOR_BGR2GRAY)

#     # Block to find the end points

    (rows, cols) = np.nonzero(skeleton)
    skel_coords = []
    for (r, c) in zip(rows, cols):
        # Extract an 8-connected neighbourhood
        (col_neigh, row_neigh) = np.meshgrid(
            np.array([c-1, c, c+1]), np.array([r-1, r, r+1]))
        # Cast to int to index into image
        col_neigh = col_neigh.astype('int')
        row_neigh = row_neigh.astype('int')
        # Convert into a single 1D array and check for non-zero locations
        pix_nbhood = skeleton[row_neigh, col_neigh].ravel() != 0
    # If the number of non-zero locations equals 2, add this to
    # the list of co-ordinates for end points
        if np.sum(pix_nbhood) < 3:
            skel_coords.append((c, r))

    # print(skel_coords)

#     # skeleton = cv2.cvtColor(skeleton, cv2.COLOR_GRAY2BGR)

#     # if len(skel_coords)==2:
#     #     cv2.circle(skeleton, (skel_coords[0][0], skel_coords[0][1]), 3, (255, 0, 0))
#     #     cv2.circle(skeleton, (skel_coords[1][0], skel_coords[1][1]), 3, (0, 0, 255))

#     #     cv2.imwrite("end point.png", skeleton)

#     # if len(skel_coords)==2:

    # robot_ee = skel_coords[0]
    # print("Robot base and ee", robot_base, robot_ee)
    robot_base = skel_coords[-1]
    pixels = np.argwhere(skeleton == 255)
    pixels = np.flip(pixels)
    # print(len(pixels))
    # print(pixels[0])
    pix_x = pixels[:, 0]
    pix_y = pixels[:, 1]
    dist_near_base = np.sqrt((pix_x - base_center_x) **
                             2 + (pix_y - base_center_y) ** 2)
    dist_near_ee = np.sqrt((pix_x - ee_center_x) **
                           2 + (pix_y - ee_center_y) ** 2)
    dist_min_base = np.argmin(dist_near_base)
    dist_min_ee = np.argmin(dist_near_ee)
    pixels = pixels.tolist()
#
    closest_base = pixels[dist_min_base]
    closest_ee = pixels[dist_min_ee]

    # pixel_cb = skeleton[closest_base[0], closest_base[1]]

    # print(pixel_cb)
    # print("Robot closest base and ee", closest_base, closest_ee)

    cut_base_index = pixels.index([robot_base[0], robot_base[1]])
    base_index = pixels.index([closest_base[0], closest_base[1]])
    points_removed_base = pixels[cut_base_index:base_index-1]

    for i in range(len(points_removed_base)-1):
        # print(len(points_removed_base))
        skeleton[points_removed_base[i][1], points_removed_base[i][0]] = 0  
    
    # cv2.imshow("New Skeleton", skeleton)
    # cv2.waitKey(1)

    # print("base index, -1, +1, cut_base_index", base_index, base_index-1, base_index+1, cut_base_index)

    # print("base pixels", pixels[base_index])
    # print("base pixels - 1", pixels[base_index-1])
    # print("base pixels + 1", pixels[base_index+1])
    # print("pixels cut_base_index", pixels[cut_base_index])

    # skeleton = cv2.cvtColor(skeleton, cv2.COLOR_GRAY2BGR)

    # for q in range(len(points_removed_base)):
    #     x = int(points_removed_base[q][0])
    #     y = int(points_removed_base[q][1])
    #     cv2.circle(skeleton, (x, y), 5, (0, 0, 0), -1)

    pixels = np.argwhere(skeleton == 255)
    pixels = np.flip(pixels)

    pix_x = pixels[:, 0]
    pix_y = pixels[:, 1]
    # pixels = pixels.tolist()

    copy_pix = pixels
    init_point = pixels[0]
    # print("Print init_point",init_point) #extract the center of the base marker and get initial position
    row_index, = np.where(np.all(copy_pix == (init_point), axis=1))
    copy_pix = np.delete(copy_pix, row_index, axis=0)
    ordered = init_point
    while len(copy_pix) > 0:
        distances = np.sqrt(
            (copy_pix[:, 0] - init_point[0]) ** 2 + (copy_pix[:, 1] - init_point[1]) ** 2)
#     print(distances)
        nearest_index = np.argmin(distances)
        # print("Point {} matched with {}".format(init_point, copy_pix[nearest_index]))
        # print("Nearest index: {}".format(nearest_index))
        init_point = copy_pix[nearest_index]  # (target[0], target[1])
        # print("Initial Pose", init_point)
        copy_pix = np.delete(copy_pix, nearest_index, axis=0)
#         print(copy_pix[0])
        ordered = np.append(ordered, [init_point])

    ordered = np.reshape(ordered, (-1, 2))

    pixels = ordered.tolist()

#
    # init_point = (closest_base[0], closest_base[1])  # base pixel used as seed point for NN-search
    # init_point = (pixels[0][0], pixels[0][1])
    # dist_mat = []
    # for i in range(len(pix_x)):
    #     dist = np.sqrt((pix_x[i]-init_point[0])**2 + (pix_y[i]-init_point[1])**2)
    #     dist_mat = np.append(dist_mat, dist)

    # index = np.argsort(dist_mat)
    # xinit, yinit = pix_x[index], pix_y[index]
    # pixels = np.c_[xinit, yinit]
    # pixels = pixels.tolist()

    # start_index = pixels.index([closest_base[0], closest_base[1]])
    start_index = pixels.index([init_point[0], init_point[1]])
    end_index = pixels.index([closest_ee[0], closest_ee[1]])

    # print("Start and End", start_index, end_index)

    points_considered = pixels[end_index:start_index]
    points_removed_ee = pixels[end_index+1:]

    # print(points_removed)

    for i in range(len(points_removed_ee)):
        skeleton[points_removed_ee[i][1], points_removed_ee[i][0]] = 0
    
    # cv2.imshow(" Skeleton", skeleton)
    # cv2.waitKey(1)

    # for q in range(len(points_removed)):
    #     x = int(points_removed[q][0])
    #     y = int(points_removed[q][1])
    #     cv2.circle(skeleton, (x, y), 5, (0, 0, 0), -1)
    # cv2.circle(skeleton,(closest_ee[0], closest_ee[1]),0,(255,255,255),-1)

    # print(skeleton.shape)

    (rows, cols) = np.nonzero(skeleton)
    skel_coords = []
    intersection = []
    for (r, c) in zip(rows, cols):
        # Extract an 8-connected neighbourhood
        (col_neigh, row_neigh) = np.meshgrid(
            np.array([c-1, c, c+1]), np.array([r-1, r, r+1]))
        # Cast to int to index into image
        col_neigh = col_neigh.astype('int')
        row_neigh = row_neigh.astype('int')
        # Convert into a single 1D array and check for non-zero locations
        pix_nbhood = skeleton[row_neigh, col_neigh].ravel() != 0
    # If the number of non-zero locations equals 2, add this to
    # the list of co-ordinates for end points
        if np.sum(pix_nbhood) < 3:
            skel_coords.append((c, r))
        # the list of co-ordinates for intersection points
        elif np.sum(pix_nbhood) > 3:
            intersection.append((c, r))

    # pixels = np.argwhere(skeleton == 255)
    # pixels = np.flip(pixels).tolist()

    # print(pixels[0])

    # print(closest_base, closest_ee)
    # print(skel_coords)
    # print(intersection)

    # if intersection:
    #     intersection_index = pixels.index(
    #         [intersection[0][0], intersection[0][1]])
    #     print(intersection_index)
    #     for i in range(len(skel_coords)):
    #         if i != 0 and i != -1:
    #             end_index = pixels.index(
    #                 [skel_coords[1][0], skel_coords[1][0]])
    #             intersection_seg = pixels[intersection_index:end_index]

    # for i in range(len(intersection_seg)):
    #     skeleton[intersection_seg[i][1], intersection_seg[i][0]] = 0
    
    new_pix = np.argwhere(skeleton == 255)
    new_pix = np.flip(new_pix)
    copy_pix = new_pix
    init_point = new_pix[0]
    row_index, = np.where(np.all(copy_pix == (init_point), axis=1))
    copy_pix = np.delete(copy_pix, row_index, axis=0)
    ordered = init_point
    while len(copy_pix) > 0:
        distances = np.sqrt(
            (copy_pix[:, 0] - init_point[0]) ** 2 + (copy_pix[:, 1] - init_point[1]) ** 2)
#     print(distances)
        nearest_index = np.argmin(distances)
        # print("Point {} matched with {}".format(init_point, copy_pix[nearest_index]))
        # print("Nearest index: {}".format(nearest_index))
        init_point = copy_pix[nearest_index]  # (target[0], target[1])
        # print("Initial Pose", init_point)
        copy_pix = np.delete(copy_pix, nearest_index, axis=0)
#         print(copy_pix[0])
        ordered = np.append(ordered, [init_point])

    ordered = np.reshape(ordered, (-1, 2))

    new_pix = ordered

    print("New Pix", new_pix)

    xinit = new_pix[:,0]
    yinit = new_pix[:,1]

    num_of_segments = rospy.get_param("vsbot/shape_control/num_of_segments")
    k = rospy.get_param("vsbot/shape_control/degree")
    point_jump = (len(xinit))/((num_of_segments+k)-1)
    x = np.array(xinit[0::math.floor(point_jump)])
    y = np.array(yinit[0::math.floor(point_jump)])

    if (x[-1] != xinit[-1]) or (y[-1] != yinit[-1]) :
      x = np.append(x, xinit[-1])
      y = np.append(y, yinit[-1])
      if len(x) > (num_of_segments+k):
        x = np.delete(x, -2)
        y = np.delete(y, -2)

    skeleton = cv2.cvtColor(skeleton, cv2.COLOR_GRAY2BGR)

    cv2.circle(skeleton,(x[0], y[0]),5,(255,0,0),-1)
    cv2.circle(skeleton,(x[1], y[1]),5,(0,255,0),-1)
    cv2.circle(skeleton,(x[2], y[2]),5,(0,0,255),-1)
    cv2.circle(skeleton,(x[3], y[3]),5,(255,0,255),-1)

    cv2.imshow("downsampled points", skeleton)
    cv2.waitKey(1)

    print("input x", x)
    print("input y", y)
    
    tck, u_params = splprep([x, y], k = k, s = 0)

    print("control points x", tck[1][0])
    print("control points x", tck[1][1])

    cv2.circle(skeleton,(int(tck[1][0][0]), int(tck[1][1][0])),5,(255,0,0),-1)
    cv2.circle(skeleton,(int(tck[1][0][1]), int(tck[1][1][1])),5,(0,255,0),-1)
    cv2.circle(skeleton,(int(tck[1][0][2]), int(tck[1][1][2])),5,(0,0,255),-1)
    cv2.circle(skeleton,(int(tck[1][0][3]), int(tck[1][1][3])),5,(255,0,255),-1)

    cv2.imshow("control points", skeleton)
    cv2.waitKey(1)

    new_params1 = np.linspace(0,1,10)
    new_pts1 = splev(new_params1, tck)
    x = new_pts1[0]
    x = x.tolist()
    y = new_pts1[1]
    y = y.tolist()
    
    points = np.c_[x, y]

    for i in range(len(points)):
        x = np.int(points[i][0])
        y = np.int(points[i][1]) 
        cv2.circle(skeleton,(x, y),5,(255,255,0),-1)

    # for q in range(len(x)):
    #     print(x[q])
    #     x = int(x[q])
    #     y = int(y[q])
    #     cv2.circle(skeleton, (x, y), 5, (255, 255, 0), -1)

    cv2.imshow("evaluated points", skeleton)
    cv2.waitKey(1)


# #     cv2.imwrite('marked_skel'+str(m)+'.png', skeleton)


# # #     # print(xinit)
# # #     # print(yinit)
# # #     # num_of_segments = rospy.get_param("vsbot/shape_control/num_of_segments")
# # #     # k = rospy.get_param("vsbot/shape_control/degree")
# # #     # point_jump = (len(xinit))/((num_of_segments+k)-1)
# # #     # x = np.array(xinit[0::math.floor(point_jump)])
# # #     # y = np.array(yinit[0::math.floor(point_jump)])

    # i += 1
    j += 1
#     k += 1
#     l += 1
#     m += 1
#     n += 1

# #     # print("new callback #",n)

    # ros_depth_img = bridge.cv2_to_imgmsg(cv_depth_img, 'bgr8')


def main(args):
    # Initialize ROS
    rospy.init_node('skeleton_node')

    # Subscribe the base and ee markers
    # ee_sub = rospy.Subscriber("ee/Pose", Float32MultiArray, ee_callback, queue_size=1)
    # base_sub = rospy.Subscriber("base/Pose", Float32MultiArray, base_callback, queue_size=1)

    # Subscribe the rgb image for marker detection
    image_rgb_sub = rospy.Subscriber(
        "/camera/color/image_raw", Image, marker_callback, queue_size=1)
    image_depth_sub = rospy.Subscriber(
        "/camera/aligned_depth_to_color/image_raw", Image, image_callback, queue_size=1)

    # Subscribe the depth image from aligned depth

    # image_depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, image_callback, queue_size=1)

    # Publisher for the marked image
    image_pub = rospy.Publisher("marked/ros/Image", Image, queue_size=1)

    rate = rospy.Rate(30)
    # Publish the marked depth image
    while not rospy.is_shutdown():
        if ros_depth_img:
            image_pub.publish(ros_depth_img)

        rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)

#!/usr/bin/env python3

# This script is used to create two images
# The robot and skeleton
# The robot and parametric curve

from PIL import Image
import glob
import cv2
import cv2.aruco as aruco
import numpy as np
from skimage.morphology import remove_small_objects, dilation, skeletonize, medial_axis, binary_closing
from skimage.segmentation import clear_border
from skimage.filters import unsharp_mask, threshold_triangle, difference_of_gaussians
from skimage import exposure
from scipy.interpolate import splprep, splev
from os.path import expanduser
import math

home = expanduser("~")


def marker_detect(cv_img):

    ## Marker Detection
    gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    arucoParameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(
        gray, aruco_dict, parameters=arucoParameters)

    id_list = []
    id_list.clear()
    for i in ids:
        if int(i) == 32:
            id_list.append(int(i))
        elif int(i) == 34:
            id_list.append(int(i))

    print(id_list)
    # Aruco ID for each marker
    base_id = 34
    ee_id = 32
    # Check if both markers are found
    marker_base_flag = False
    try:
        base_index = id_list.index(base_id)
        marker_base_flag = True
    except:
        marker_base_flag = False

    marker_ee_flag = False
    try:
        ee_index = id_list.index(ee_id)
        marker_ee_flag = True
    except:
        marker_ee_flag = False

    # Separating the corner pixel co-ordinates for each marker
    if (marker_base_flag) and (marker_ee_flag):
        ee_corners_list = corners[ee_index].reshape(4, 2)
        base_corners_list = corners[base_index].reshape(4, 2)

        # Averaging base corner co-ordinates to obtain marker center
        base_center_x = (base_corners_list[0][0] + base_corners_list[1]
                        [0] + base_corners_list[2][0] + base_corners_list[3][0])/4
        base_center_y = (base_corners_list[0][1] + base_corners_list[1]
                        [1] + base_corners_list[2][1] + base_corners_list[3][1])/4
        base_center = [base_center_x, base_center_y]

        # Averaging ee corner co-ordinates to obtain marker center
        ee_center_x = (ee_corners_list[0][0] + ee_corners_list[1]
                    [0] + ee_corners_list[2][0] + ee_corners_list[3][0])/4
        ee_center_y = (ee_corners_list[0][1] + ee_corners_list[1]
                    [1] + ee_corners_list[2][1] + ee_corners_list[3][1])/4
        ee_center = [ee_center_x, ee_center_y]
    
    return(base_center, ee_center)


def depth_processing(cv_depth_img, base_center, ee_center, dashed):
    # Depth image processing
    dmin = 3
    dmax = 5

    num_eval_pts = 0

    if dashed:
        num_eval_pts = 35
    else:
        num_eval_pts = 300

    cv_depth_img = np.where((cv_depth_img <= dmin) | (
    cv_depth_img > dmax), 0, cv_depth_img)

    # cv_depth_img = cv2.cvtColor(cv_depth_img, cv2.COLOR_GRAY2BGR)
    k1 = 9
    k2 = 9
    kernel = np.ones((k1, k2), np.uint8)
    cv_depth_img = cv2.GaussianBlur(cv_depth_img, (5, 5), 0)
    cv_depth_img = cv2.dilate(cv_depth_img, kernel, iterations=3)
    cv_depth_img = cv2.erode(cv_depth_img, kernel, iterations=2)
    cv_depth_img = cv2.morphologyEx(cv_depth_img, cv2.MORPH_OPEN, kernel)

    # Skeletonization and processing the skeleton

    binary = np.where(cv_depth_img > 0, 1, cv_depth_img)
    skeleton = skeletonize(binary, method='lee')
    skeleton = cv2.cvtColor(skeleton, cv2.COLOR_BGR2GRAY)
    skeleton = np.where(skeleton == 1, 255, skeleton)  

    # Block to find the end points
    (rows, cols) = np.nonzero(skeleton)
    end_coords = [] # storing the end points
    for (r, c) in zip(rows, cols):
        (col_neigh, row_neigh) = np.meshgrid(
        np.array([c-1, c, c+1]), np.array([r-1, r, r+1]))
        col_neigh = col_neigh.astype('int')
        row_neigh = row_neigh.astype('int')
        pix_nbhood = skeleton[row_neigh, col_neigh].ravel() != 0
        if np.sum(pix_nbhood) < 3:
            end_coords.append((c, r))

    # in the following blosck we are finding the white pixels on the skeleton closest to the markers
    robot_base = end_coords[-1]

    # cv2.circle(skeleton, (int(robot_base[0]), int(robot_base[1])), 5, (255, 255, 255), -1)

    pixels = np.argwhere(skeleton == 255)
    pixels = np.flip(pixels)
    pix_x = pixels[:, 0]
    pix_y = pixels[:, 1]
    dist_near_base = np.sqrt((pix_x - base_center[0]) **
                        2 + (pix_y - base_center[1]) ** 2)
    dist_near_ee = np.sqrt((pix_x - ee_center[0]) **
                        2 + (pix_y - ee_center[1]) ** 2)
    dist_min_base = np.argmin(dist_near_base)
    dist_min_ee = np.argmin(dist_near_ee)
    pixels = pixels.tolist()
    closest_base = pixels[dist_min_base]   # pixel closest to base center
    closest_ee = pixels[dist_min_ee]       # pixel closest to ee center


    # block to remove the part of the skeleton from base marker to the base of robot
    cut_base_index = pixels.index([robot_base[0], robot_base[1]])
    base_index = pixels.index([closest_base[0], closest_base[1]])
    points_removed_base = pixels[cut_base_index:base_index-1]

    for i in range(len(points_removed_base)-1):
        skeleton[points_removed_base[i][1], points_removed_base[i][0]] = 0  


    # Order the remaining white pixels for further skeleton processing
    pixels = np.argwhere(skeleton == 255)
    pixels = np.flip(pixels)
    copy_pix = pixels
    init_point = pixels[0]  
    row_index, = np.where(np.all(copy_pix == (init_point), axis=1))
    copy_pix = np.delete(copy_pix, row_index, axis=0)
    ordered = init_point
    while len(copy_pix) > 0:
        distances = np.sqrt(
            (copy_pix[:, 0] - init_point[0]) ** 2 + (copy_pix[:, 1] - init_point[1]) ** 2)
        nearest_index = np.argmin(distances)
        init_point = copy_pix[nearest_index]  
        copy_pix = np.delete(copy_pix, nearest_index, axis=0)
        ordered = np.append(ordered, [init_point])
    ordered = np.reshape(ordered, (-1, 2))
    pixels = ordered.tolist()

    # Remove the part of skeleton from ee marker to end of the robot
    start_index = pixels.index([init_point[0], init_point[1]])
    end_index = pixels.index([closest_ee[0], closest_ee[1]])

    points_considered = pixels[end_index:start_index]
    points_removed_ee = pixels[end_index+1:]

    for i in range(len(points_removed_ee)):
        skeleton[points_removed_ee[i][1], points_removed_ee[i][0]] = 0

    ## Reordering white pixels

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
        nearest_index = np.argmin(distances)
        init_point = copy_pix[nearest_index] 
        copy_pix = np.delete(copy_pix, nearest_index, axis=0)
        ordered = np.append(ordered, [init_point])

    ordered = np.reshape(ordered, (-1, 2))
    new_pix = ordered

    # Spline fitting

    xinit = new_pix[:,0]
    yinit = new_pix[:,1]

    # skeleton = cv2.cvtColor(skeleton, cv2.COLOR_GRAY2BGR)
    # for i in range(len(points)):
    #     x = np.int(points[i][0])
    #     y = np.int(points[i][1])
    #     cv2.circle(cv_img,(x, y),3,(255,0,0),1)

    # Downsampling the points in the skeleton to fit a curve
    num_of_segments = 2
    k = 3
    point_jump = (len(xinit))/((num_of_segments+k)-1)
    x = np.array(xinit[0::math.floor(point_jump)])
    y = np.array(yinit[0::math.floor(point_jump)])

    if (x[-1] != xinit[-1]) or (y[-1] != yinit[-1]) :
        x = np.append(x, xinit[-1])
        y = np.append(y, yinit[-1])
    if len(x) > (num_of_segments+k):
        x = np.delete(x, -2)
        y = np.delete(y, -2)


    x[0] = base_center[0]
    y[0] = base_center[1]
    x[-1] = ee_center[0]
    y[-1] = ee_center[1]

    tck, u_params = splprep([x, y], k = k, s = 0)

    cx = tck[1][0]  
    cx = cx.tolist()
    cy = tck[1][1]
    cy = cy.tolist()
    
    # List of control points
    # putting 1ist control points at the end as it is not changing and can be discarded
    cp = []
    for i in range(len(cx)-1):
        cp.append(cx[i]) 
        cp.append(cy[i])


    # Evaluating the spline
    new_params = np.linspace(0,1,num_eval_pts)
    new_pts = splev(new_params, tck)
    x = new_pts[0].tolist()
    y = new_pts[1].tolist()


    points = np.c_[x,y]

    ct_pts = np.c_[cx, cy]

    return(points, ct_pts, ordered)


def main():

    im_path = home + "/Desktop/robot_img.png"
    im_depth_path = home + "/Desktop/robot_depth_img.png"
    
    # Read in image
    robot_img = cv2.imread(im_path)
    robot_depth_img = cv2.imread(im_depth_path)
    
    # Find the markers
    base_marker, ee_marker = marker_detect(robot_img)

    # Process
    curve_points, control_points, skeleton = depth_processing(robot_depth_img, base_marker, ee_marker, dashed=False)

    # Draw skeleton on image
    for i in range(len(skeleton)):
        x = np.int(skeleton[i][0])
        y = np.int(skeleton[i][1])
        cv2.circle(robot_img,(x, y),2,(255,133,26),-1)
    
    # Save image with skeleton
    write_path = home + "/Desktop/skeleton.png"
    cv2.imwrite(write_path, robot_img)
    
    # Draw curve
    for i in range(len(curve_points)):
        x = np.int(curve_points[i][0])
        y = np.int(curve_points[i][1])
        cv2.circle(robot_img,(x, y),2,(89,17,212),-1)

    # Save image with curve
    write_path = home + "/Desktop/curve.png"
    cv2.imwrite(write_path, robot_img)


if __name__ == "__main__":
    main()
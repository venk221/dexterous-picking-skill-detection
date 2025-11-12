#!/usr/bin/env python3

import numpy as np
import cv2
import sys
from skimage.morphology import skeletonize
import math
from scipy.interpolate import splprep, splev
from os.path import expanduser
import glob


home = expanduser("~")
ee_traj_x = []
ee_traj_y = []

# def read_img(im_path):
#     cv_img = cv2.imread(im_path)

#     return cv_img

def convert_binary(cv_img):
    #  converting to hsv
    hsvimg = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
    # Define image color bounds 
    orange_lower = np.array([10, 100, 20], np.uint8) 
    orange_upper = np.array([25, 255, 255], np.uint8) 
    white_lower = np.array([0, 0, 200], np.uint8)
    white_upper = np.array([145, 60, 255], np.uint8) 
    blue_lower = np.array([78,158,124])
    blue_upper = np.array([138,255,255])

    # Binarizing individual colors & combining
    orange_mask = cv2.inRange(hsvimg, orange_lower, orange_upper)
    blue_mask = cv2.inRange(hsvimg, blue_lower, blue_upper)
    white_mask = cv2.inRange(hsvimg, white_lower, white_upper)

    cv_binary = orange_mask + blue_mask + white_mask

    return cv_binary, blue_mask

def marker_detection(blue_mask):
    global ee_traj_x, ee_traj_y
    # define kernel 
    kernel = np.ones((5, 5), "uint8")
    # define contour for the marker
    blue_mask_contour = cv2.dilate(blue_mask, kernel)
    #second way of finding center of marker
    contours, hierarchy = cv2.findContours(blue_mask_contour, 
                                       cv2.RETR_TREE, 
                                       cv2.CHAIN_APPROX_SIMPLE)   



    for contour in contours:
        M = cv2.moments(contour)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        print("Second Centers", cX, cY)
    
    marker = [cX, cY]
    ee_traj_x.append(cX)
    ee_traj_y.append(cY)
    return marker



def skeletonizer(cv_binary, num_of_segments, k, marker):
    binary = np.where(cv_binary > 0, 1, cv_binary)    
    skeleton = skeletonize(binary, method='lee')    
    skeleton = np.where(skeleton == 1, 255, skeleton)

    pixels = np.argwhere(skeleton > 0)
    xinit = pixels[:, 1]
    yinit = pixels[:, 0]

    init_point = (149, 149)  # base pixel used as seed point for NN-search
    dist_mat = []
    for i in range(len(xinit)):
        dist = np.sqrt((xinit[i]-init_point[0])**2 + (yinit[i]-init_point[1])**2)
        dist_mat = np.append(dist_mat, dist)


    index = np.argsort(dist_mat)
    xinit, yinit = xinit[index], yinit[index]
    point_jump = (len(xinit))/((num_of_segments+k)-1)
    x = np.array(xinit[0::math.floor(point_jump)])
    y = np.array(yinit[0::math.floor(point_jump)])

    if (x[-1] != xinit[-1]) or (y[-1] != yinit[-1]) :
        x = np.append(x, xinit[-1])
        y = np.append(y, yinit[-1])
        if len(x) > (num_of_segments+k):
            x = np.delete(x, -2)
            y = np.delete(y, -2)
    
    x[-1] = marker[0]
    y[-1] = marker[1]

    return skeleton, np.c_[xinit, yinit], np.c_[x,y]
        # skeletonized img, ordered skel points, downsampled pts


def fit_curve(downsampled_pts, k, no_of_points):
    # Spline Fitting
    x = downsampled_pts[0:, 0]
    y = downsampled_pts[0:, 1]
    tck, u_params = splprep([x, y], k = k, s = 0)

    # Extracting control points

    cx = tck[1][0]  
    cx = cx.tolist()
    cy = tck[1][1]
    cy = cy.tolist()

    # List of control points
    # putting 1ist control points at the end as it is not changing and can be discarded
    cp = []
    # print(type(cp))
    for i in range(len(cx)-1):
        cp.append(cx[i+1]) 
        cp.append(cy[i+1])

    # Spline evaluation
    new_params = np.linspace(0,1,no_of_points)  
    new_pts = splev(new_params, tck)
    x = new_pts[0]
    x = x.tolist()
    y = new_pts[1]
    y = y.tolist()


    return cp, np.c_[x,y]


# def draw_curves(cv_img, points, color):
#     # evaluated points to be drawn
#     # cv_img = np.zeros((300,300,3),np.uint8)
#     for i in range(len(points)):
#         x = np.int(points[i][0])
#         y = np.int(points[i][1])
#         cv2.circle(cv_img,(x, y),2,color,-1)

#     return cv_img


def read_imgs(path):
    file_list = []
    img_array = []

    # Cache image files
    for file in glob.glob(path + "*.png"):
        file_tup = file.partition('.')
        file_num = file_tup[0].rsplit("/")
        file_list.append(int(file_num[-1]))

    # Sort cached files
    file_list.sort()

    # Read in files as cv img
    for file in file_list:
        img = cv2.imread(path+"/"+str(file)+".png")
        img_array.append(img)

    return img_array


def plot_curve(cv_img, curve, target_curve,itr):
    # current curve
    for i in range(len(curve)):
        x = np.int(curve[i][0])
        y = np.int(curve[i][1])
        cv2.circle(cv_img,(x, y),2,(255,133,26),-1)

    # target curve
    for i in range(len(target_curve)):
        x = np.int(target_curve[i][0])
        y = np.int(target_curve[i][1])
        cv2.circle(cv_img,(x, y),3,(89,17,212),-1)

    # trajectory
    if (itr>149):
        for i in range(150,itr-1):
            start_pt1 = (int(ee_traj_x[i]), int(ee_traj_y[i]))
            end_pt1 = (int(ee_traj_x[i+1]), int(ee_traj_y[i+1]))
            cv_img = cv2.line(cv_img, start_pt1, end_pt1,(0,97,233), 2)
    
    return cv_img


def draw_curves(exp_imgs, target_img, num_of_segments, k):
    curve_imgs = []

    # get goal curve
    bin_img = convert_binary(target_img)[0]
    blue_mask_img = convert_binary(target_img)[1]
    goal_marker = marker_detection(blue_mask_img)
    target_skel_img, target_ordered_pts, target_downsampled_pts =  skeletonizer(bin_img, num_of_segments, k, goal_marker)
    target_control_points, target_evaluated_points = fit_curve(target_downsampled_pts, k, no_of_points=15)

    # draw curves
    for i in range(len(exp_imgs)):
        bin_img = convert_binary(exp_imgs[i])[0]
        blue_mask_img = convert_binary(exp_imgs[i])[1]
        ee_marker = marker_detection(blue_mask_img)
        skel_img, ordered_pts, downsampled_pts = skeletonizer(bin_img, num_of_segments, k, ee_marker)
        control_pts, evaluated_pts = fit_curve(downsampled_pts, k, no_of_points=200)

        # Draw current curve
        img = plot_curve(exp_imgs[i], evaluated_pts, target_evaluated_points, i)
        #  color=(255,133,26))

        # Draw goal curve
        # img = draw_curves(img, target_evaluated_points, color=(89,17,212))

        curve_imgs.append(img)

    return curve_imgs


def write_imgs(curve_imgs, exp_folder, num):
    print("Writing Images")
    write_path = exp_folder + str(num)+"/curve_imgs/"

    for i in range(len(curve_imgs)):
        cv2.imwrite(write_path+str(i)+".png", curve_imgs[i])
    return True


def write_vid(curve_imgs, exp_folder, num):
    height,width,layers = curve_imgs[0].shape
    vid_size = (width, height)
    vid_path = exp_folder + str(num) + "/servo_video.avi"
    out = cv2.VideoWriter(vid_path,cv2.VideoWriter_fourcc(*'XVID'), 120, vid_size)   # 30 fps
    # print(len(curve_imgs))
    for i in range(150,len(curve_imgs)):
        out.write(curve_imgs[i])
    out.release()
    print("Video Saved")
    return True


def main():
    global ee_traj_x, ee_traj_y
    # exp_folder = home + "/Pictures/curve_estimation_adaptive_vs_gazebo/servoing/exp3-1-IROS-3link/"
    exp_folder = home + "/Pictures/curve_estimation_adaptive_vs_gazebo/servoing/exps-config2-for-paper/"
    exp_no = [3]

    # Specify fit params
    num_of_segments = 1
    k = 3

    for num in exp_no:
        
        path_to_imgs = exp_folder + str(num) + "/raw/"
        target_img_path = exp_folder + str(num) + "/goal.tiff"

        # Read images
        exp_imgs = read_imgs(path_to_imgs)
        target_img = cv2.imread(target_img_path)

        # Draw curves
        curve_imgs = draw_curves(exp_imgs, target_img, num_of_segments, k)

        # Export images
        write_imgs(curve_imgs, exp_folder, num)

        # Write video
        write_vid(curve_imgs, exp_folder, num)

        ee_traj_x.clear()
        ee_traj_y.clear()

        
        
        # bin_img = convert_binary(cv_img)[0]
        # target_bin_img = convert_binary(target_cv_img)[0]

        # blue_mask_img = convert_binary(cv_img)[1]
        # marker_img = marker_detection(blue_mask_img)
        # blue_mask_target = convert_binary(target_cv_img)[1]
        # marker_target = marker_detection(blue_mask_target)
        # skel_img, ordered_pts, downsampled_pts = skeletonizer(bin_img,  num_of_segments, k, marker_img)
        # target_skel_img, target_ordered_pts, target_downsampled_pts = skeletonizer(target_bin_img, num_of_segments, k, marker_target)

        # control_points, evaluated_points = fit_curve(downsampled_pts, k, no_of_points=200)  
        # target_control_points, target_evaluated_points = fit_curve(target_downsampled_pts, k, no_of_points=15)

        # cv_img = draw_curves(cv_img, evaluated_points, color=(0,255,0))
        # # cv_img = draw_curves(cv_img, target_evaluated_points, color=(0,0,255) )
        # # Save image
        # save_path = home + "/Desktop/initial_" + img_no+".png"
        # cv2.imwrite(save_path,cv_img)


if __name__ == "__main__":
    main()
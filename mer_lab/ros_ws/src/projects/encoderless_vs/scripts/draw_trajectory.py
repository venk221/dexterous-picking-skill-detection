#!/usr/bin/env python3

# This script will take a path to a shape VS experiment folder
# 1. Draw curves on all the raw experiment images
# 2. Draw goal curve
# 3. Overlay all of the servoing images with the goal curve
# 4. Assumes a window size of 50, but this can be changed
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
ee_traj_x = []
ee_traj_y = []
benchmark_traj_x = []
benchmark_traj_y = []

def import_imgs(path):
    global image_array
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
        # height, width, layers = img.shape
        # size = (width, height)
        img_array.append(img)

    return img_array


def get_markers(cv_img):
    global ee_traj_x, ee_traj_y
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

    # print(id_list)
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
    # base_center = [253.5, 439.5]
    ee_traj_x.append(ee_center_x)
    ee_traj_y.append(ee_center_y)

    return(base_center, ee_center)
    

def plot_traj(raw_imgs, goal):
    for i in range(len(raw_imgs)):
        print(i)
        base, ee = get_markers(raw_imgs[i])
    
        if i > 76:
            for j in range(77, i-1):
                # draw trajectory
                start_pt1 = (int(ee_traj_x[j]), int(ee_traj_y[j]))
                end_pt1 = (int(ee_traj_x[j+1]), int(ee_traj_y[j+1]))
                raw_imgs[i] = cv2.line(raw_imgs[i], start_pt1, end_pt1,(0,97,233), 2)
            # Draw benchmark traj
            for j in range(83,len(benchmark_traj_x)-1):
                start_pt2 = (int(benchmark_traj_x[j]), int(benchmark_traj_y[j]))
                end_pt2 = (int(benchmark_traj_x[j+1]), int(benchmark_traj_y[j+1]))
                raw_imgs[i] = cv2.line(raw_imgs[i], start_pt2, end_pt2, (26,255,26), 2)

            # draw target pose
            raw_imgs[i] = cv2.circle(raw_imgs[i],(goal[0], goal[1]),3,(89,17,212),2)
    return raw_imgs

def export_imgs(curve_imgs, exp_folder, num):
    print("Writing Images")
    write_path = exp_folder + str(num)+"/curve_imgs/"

    for i in range(len(curve_imgs)):
        cv2.imwrite(write_path+str(i)+".png", curve_imgs[i])
    return True


def write_vid(curve_imgs, exp_folder, num):
    height,width,layers = curve_imgs[0].shape
    vid_size = (width, height)
    vid_path = exp_folder + str(num) + "/servo_video.avi"
    out = cv2.VideoWriter(vid_path,cv2.VideoWriter_fourcc(*'XVID'), 45, vid_size)   # 14.9 fps
    # print(len(curve_imgs))
    for i in range(77,len(curve_imgs)):
        out.write(curve_imgs[i])
    out.release()
    print("Video Saved")
    return True

# def main():
#     exp_folder = home + "/Pictures/baseline/"
#     exp_no = [3]
#     # goal = [384,215]
#     goal = [398,116]
#     for num in exp_no:
        
#         path_to_exp_imgs = exp_folder + str(num) + "/raw/"

#         # Read in and sort all raw images
#         raw_imgs = import_imgs(path_to_exp_imgs)

#         # Draw trajectory
#         traj_imgs = plot_traj(raw_imgs, goal)

#         # Write all images, delete tmp images
#         export_imgs(traj_imgs, exp_folder, num)

#         # Export to video
#         write_vid(traj_imgs, exp_folder, num)


def main():
    global benchmark_traj_x, benchmark_traj_y
    exp_folder = home + "/Pictures/robustness/baseline/exps/"
    goal = [384,215]

    # Benchmark
    num = "1a"    
    path_to_exp_imgs = exp_folder + str(num) + "/raw/"

    # Read in and sort all raw images
    raw_imgs = import_imgs(path_to_exp_imgs)

    # Benchmark trajectory
    for i in range(len(raw_imgs)):
        base, ee = get_markers(raw_imgs[i])
        benchmark_traj_x.append(ee[0])
        benchmark_traj_y.append(ee[1])

    ee_traj_x.clear()
    ee_traj_y.clear()

    num = 1
    path_to_exp_imgs = exp_folder + str(num) + "/raw/"

    # Read in and sort all raw images
    raw_imgs = import_imgs(path_to_exp_imgs)

    # Draw trajectory
    traj_imgs = plot_traj(raw_imgs, goal)

    # Write all images, delete tmp images
    export_imgs(traj_imgs, exp_folder, num)

    # Export to video
    write_vid(traj_imgs, exp_folder, num)

if __name__ == "__main__":
    main()
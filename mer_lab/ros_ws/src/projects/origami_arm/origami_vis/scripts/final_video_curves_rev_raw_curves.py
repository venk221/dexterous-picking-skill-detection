#!/usr/bin/env python3

# import rospy
# import os
import cv2
from numpy import pi
import numpy as np
import math
from pyclothoids import Clothoid
import cv2.aruco as aruco
import yaml

#home_path = (os.path.expanduser('~'))
#folder_path = home_path + "/Pictures/origami_skeleton_vs/test/"
#folder_path = "./raw_imgs_all/"
#raw_path = folder_path + "clothoids_2-25/shape-convergence/5/"
#img_path = folder_path + "clothoids_2-25/shape-convergence/5/raw_images_all/"
img_path = "./raw_imgs_all/"
#write_path = folder_path + "clothoids_2-25/shape-convergence/5/final_images/"
write_path = "./final_images/"
start_img = 22
end_img = 105
marker_flag=None

def marker_detect(img):
    global marker_flag
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50) 
    arucoParameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=arucoParameters)

    # Aruco ID for each marker
    base_id = 34
    ee_id = 32
    
    # print("Id generated", ids) #Remove
    id_list = []    
    id_list.clear()
    for i in ids:
        if int(i) == ee_id:
            id_list.append(int(i))
        elif int(i) == base_id:
            id_list.append(int(i))

    # Check if both markers are found
    try:    
        base_index = id_list.index(base_id)
        ee_index = id_list.index(ee_id)
        marker_flag = True
    except:
        marker_flag = False

    if marker_flag:
        ee_corners_list = corners[ee_index].reshape(4,2)
        base_corners_list = corners[base_index].reshape(4,2)

    x0 = int((base_corners_list[2][0] + base_corners_list[3][0])/2)
    y0 = int((base_corners_list[2][1] + base_corners_list[3][1])/2)
    t0 = pi/2
    xe = int((ee_corners_list[0][0] + ee_corners_list[1][0])/2)
    ye = int((ee_corners_list[0][1] + ee_corners_list[1][1])/2)
    te = math.atan2((ee_corners_list[2][1] - ee_corners_list[1][1]),(ee_corners_list[2][0] - ee_corners_list[1][0])) 
    
    markers = [x0, y0, t0, xe, ye, te]
    return(markers)

def rev_raw_markers(img_marker):

    yaml_filename = './origami_features.yaml'
    with open(yaml_filename, 'r') as file:
        prime_service = yaml.safe_load(file)
        
        goal_features = prime_service['origami_skeleton_vs']['goal_features']

        kappa_gain = 1000000

        kd = goal_features[5] - goal_features[2]
        kd = kd/kappa_gain

        base_center = [img_marker[0], img_marker[1]]

        base_kappa = goal_features[2]/kappa_gain - kd

        clothoid_01 = Clothoid.Forward(base_center[0], base_center[1], np.pi/2 ,base_kappa,
                                       goal_features[6], goal_features[7])
        ref_ee_center = [goal_features[6], goal_features[7]]
        ref_th_e = clothoid_01.ThetaEnd
        # ref_th_e = 0.5

        markers = [img_marker[0], img_marker[1], img_marker[2], ref_ee_center[0], ref_ee_center[1], ref_th_e]

        return(markers)

def fit_clothoid(markers):
    # Calculate cur clothoid
    cur_curve = Clothoid.G1Hermite(markers[0],markers[1],markers[2],markers[3],markers[4],markers[5], tol=1e-10)
    
    # Compute curve parameters
    l = cur_curve.length
    k0 = cur_curve.KappaStart
    k1 = cur_curve.KappaEnd
    dk = cur_curve.dk

    # Sample points
    s = np.linspace(0, l, 4)
    s = s[1:]

    clothoid_pts = []
    clothoid_pts.clear()

    for s_i in s:
            clothoid_pts.append(cur_curve.X(s_i))
            clothoid_pts.append(cur_curve.Y(s_i))
    
    X,Y = cur_curve.SampleXY(50)

    return(X,Y,clothoid_pts)

def draw_curves(ref_markers, markers, raw_image):
    # Get features and points
    ref_X, ref_Y, ref_features = fit_clothoid(ref_markers)
    cur_X, cur_Y, cur_features = fit_clothoid(markers)
    
    # Draw reference curve
    for i in range(50):
        cv2.circle(raw_image, (int(ref_X[i]), int(ref_Y[i])),6, (183, 95, 211), -1 )
    # Draw reference features
    for i in range(3):
        cv2.circle(raw_image, (int(ref_features[i*2]), int(ref_features[i*2 + 1])), 6, (98, 254, 254), 3)
    
    # Draw cur curve
    for i in range(50):
        cv2.circle(raw_image, (int(cur_X[i]), int(cur_Y[i])),2, (155, 58, 93), -1 )
    # Draw current features
    for i in range(3):
        cv2.circle(raw_image, (int(cur_features[i*2]), int(cur_features[i*2 + 1])), 5, (0, 97, 230), -1)
    
    return(raw_image)


def main():
    #ROS stuff
   # rospy.init_node('write_raw_imgs')
    # read raw ref image
   # ref_raw = cv2.imread((raw_path+"ref_raw.jpg"), cv2.IMREAD_COLOR)
    j = 0
    for i in range(start_img, end_img+1):

        # # Read image
        raw_image = cv2.imread((img_path + str(i)+".jpeg"), cv2.IMREAD_COLOR)

        # # detect ref markers
        # ref_markers = marker_detect(ref_raw)

        # detect markers
        markers =  marker_detect(raw_image)

        ref_markers = rev_raw_markers(markers)
        
        # draw curves
        final_image = draw_curves(ref_markers, markers, raw_image)

        # write images
        cv2.imwrite((write_path+str(j)+".jpeg"),final_image)
        j=j+1

    print("done")
    # rospy.spin()

if __name__ == "__main__":
    main()

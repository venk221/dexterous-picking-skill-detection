#!/usr/bin/env python3
import numpy as np
import sys
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt
import rospy
import cv2
import time
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import pyrealsense2 as rs
from cv_bridge import CvBridge, CvBridgeError
from skimage.morphology import skeletonize, skeletonize_3d, thin, remove_small_objects
from encoderless_vs.srv import bin_img, bin_imgResponse

bridge = CvBridge()
current_ros_image = None

i = 0
j=0
k = 0
xinit= []
yinit= []
dist_int_base = 0
dist_int_end = 0

def binary_image_service(ros_image):
    global bridge, i, j, xinit, yinit, dist_int_end, dist_int_base, k
    current_ros_image = ros_image
    if current_ros_image is not None:
        print(True)
    else:
        print("No Image Found")
    
    print("i prev", i)
    depth_img = bridge.imgmsg_to_cv2(current_ros_image, '8UC1')
    # depth_img = cv2.cvtColor()
    # depth_img = np.array(depth_img, dtype=np.uint8)
    # depth_img = depth_img.astype("float32")
    # cv2.normalize(depth_img, depth_img, 0, 1, cv2.NORM_MINMAX)

    depth_img = cv2.cvtColor(depth_img, cv2.COLOR_GRAY2BGR)

    cv2.imwrite('/home/jani/Documents/pics/CV-Depth'+str(i)+'.tiff', depth_img)

    print(depth_img.shape)

    # r = cv2.selectROI(depth_img)
    # print(r[0], r[1], r[2], r[3])
    # depth_roi = depth_img[r[1]:r[1]+r[3], r[0]:r[0]+r[2]]
    # print(np.max(depth_roi))

    r = rospy.get_param('vsdepth/shape_control/cropper')
    depth_img_cropped = depth_img[r[1]:r[1]+r[3], r[0]:r[0]+r[2]]

    cv2.imwrite('/home/jani/Documents/pics/depth_cropped'+str(i)+'.tiff', depth_img_cropped)

    # r = cv2.selectROI(depth_img_cropped)
    # print(r[0], r[1], r[2], r[3])
    # depth_roi = depth_img_cropped[r[1]:r[1]+r[3], r[0]:r[0]+r[2]]
    # print(np.max(depth_roi))

    dmin = rospy.get_param('vsdepth/shape_control/dmin')
    dmax = rospy.get_param('vsdepth/shape_control/dmax')

    depth_img_cropped = np.where((depth_img_cropped <= dmin) | (
        depth_img_cropped > dmax), 0, depth_img_cropped)
    
    # depth_img_cropped = np.where(depth_img_cropped > dmax, 0, depth_img_cropped)

    depth = depth_img_cropped
    # depth = cv2.cvtColor(depth, cv2.COLOR_GRAY2BGR)
    k1 = rospy.get_param('vsdepth/shape_control/kernel1')
    k2 = rospy.get_param('vsdepth/shape_control/kernel2')
    kernel = np.ones((k1, k2), np.uint8)
    depth = cv2.GaussianBlur(depth, (5,5), 0)
    depth = cv2.dilate(depth, kernel, iterations=2)
    depth = cv2.erode(depth, kernel, iterations=3)
    depth = cv2.morphologyEx(depth, cv2.MORPH_OPEN, kernel)
 
    cv2.imwrite('/home/jani/Documents/pics/depth_gray'+str(i)+'.tiff', depth)

    binary = np.where(depth > 0, 1, depth)
    skeleton = skeletonize(binary, method='lee')
# skeleton = np.array(skeleton, dtype=np.uint8)
    skeleton = cv2.cvtColor(skeleton, cv2.COLOR_BGR2GRAY)
    skeleton = np.where(skeleton == 1, 255, skeleton)
    cv2.imwrite('/home/jani/Documents/pics/orig_skel'+str(i)+'.tiff', skeleton)  

    pixels = np.argwhere(skeleton == 255)
    pixels = np.flip(pixels)
    copy_pix = pixels

    initial_pos = pixels[0] #extract the center of the base marker and get initial position    
    row_index, = np.where(np.all(copy_pix == [initial_pos], axis=1))
    copy_pix = np.delete(copy_pix, row_index, axis=0)
    ordered = initial_pos

    while len(copy_pix)>0:
        distances = np.sqrt((copy_pix[:,0] - initial_pos[0]) ** 2 + (copy_pix[:,1] - initial_pos[1]) ** 2)
#     print(distances)
        nearest_index = np.argmin(distances)
        # print("Point {} matched with {}".format(initial_pos, copy_pix[nearest_index]))
        # print("Nearest index: {}".format(nearest_index))
        initial_pos = copy_pix[nearest_index] #(target[0], target[1])
        # print("Initial Pose", initial_pos)
        copy_pix = np.delete(copy_pix, nearest_index, axis=0)
#         print(copy_pix[0])
        ordered = np.append(ordered, [initial_pos])

    ordered = np.reshape(ordered, (-1, 2))        

    skeleton = cv2.cvtColor(skeleton, cv2.COLOR_GRAY2BGR)

    points = ordered

    for j in range(len(points)):
            x = np.int(points[j][0])
            y = np.int(points[j][1])
            cv2.circle(skeleton,(x, y),0,(0,255,255),-1)

    skeleton = np.where(skeleton == 255, 0, skeleton)

    for j in range(len(points)):
            x = np.int(points[j][0])
            y = np.int(points[j][1])
            cv2.circle(skeleton,(x, y),0,(255,255,255),-1)

    cv2.imwrite('/home/jani/Documents/pics/ordered_skel'+str(i)+'.tiff', skeleton)

    skel_no_branch = skeleton

    skeleton = cv2.cvtColor(skeleton, cv2.COLOR_BGR2GRAY)

    print((skeleton.shape))

    (rows,cols) = np.nonzero(skeleton)
    # print(rows)
    # print(cols)

    skel_coords = []
    intersection = []

    for (r,c) in zip(rows,cols):

    # Extract an 8-connected neighbourhood
        (col_neigh,row_neigh) = np.meshgrid(np.array([c-1,c,c+1]), np.array([r-1,r,r+1]))
        # Cast to int to index into image
        col_neigh = col_neigh.astype('int')
        row_neigh = row_neigh.astype('int')

        # Convert into a single 1D array and check for non-zero locations
        # print(skeleton[row_neigh,col_neigh].ravel())
        pix_nbhood = skeleton[row_neigh,col_neigh].ravel() != 0 
        # print(pix_nbhood)
        # print(np.sum(pix_nbhood))

    # If the number of non-zero locations equals 2, add this to 
    # our list of co-ordinates for end points
        if np.sum(pix_nbhood) < 3:
#             print(np.sum(pix_nbhood))
            skel_coords.append((c,r))
        # our list of co-ordinates for intersection points
        elif np.sum(pix_nbhood) > 3:
#             print(np.sum(pix_nbhood))
            intersection.append((c,r))
    
    print("Skel Co_ords", skel_coords)
    print("Intersection", intersection)
    print(len(skel_coords))
    skeleton = cv2.cvtColor(skeleton, cv2.COLOR_GRAY2BGR)

    for j in range(len(skel_coords)):
        cv2.circle(skeleton,(skel_coords[j]),5,(255,0,0),-1)
    for k in range(len(intersection)):
        cv2.circle(skeleton,(intersection[k]),5,(255,0,255),-1) 
    
    cv2.imwrite('/home/jani/Documents/pics/skel_corner'+str(i) + '.tiff', skeleton)

    if (len(intersection) > 0):
        dist_int_end = np.sqrt((skel_coords[0][0] - intersection[0][0]) ** 2 + (skel_coords[0][1] - intersection[0][1]) ** 2)
        dist_int_base = np.sqrt((skel_coords[-1][0] - intersection[0][0]) ** 2 + (skel_coords[-1][1] - intersection[0][1]) ** 2)

        pix = points.tolist()        

        base_index = pix.index([skel_coords[-1][0],skel_coords[-1][1]])
        end_ind1 = pix.index([skel_coords[0][0],skel_coords[0][1]])
        end_ind2 = pix.index([skel_coords[1][0],skel_coords[1][1]])
        mid_index = pix.index([intersection[0][0], intersection[0][1]])
        print(end_ind1, end_ind2, base_index, mid_index)

        if end_ind1 > end_ind2 and (mid_index < end_ind2):
                short_pix = pix[mid_index+1:end_ind2+1]
        elif end_ind1 > end_ind2 and (mid_index > end_ind2):
            short_pix = pix[mid_index+1:end_ind1+1]
        else:
            short_pix = pix[mid_index+1:end_ind1+1]

        print("short_pix", short_pix)

        # points = np.array(pts_short)
        # for j in range(len(points)):
        #     x = np.int(points[j][0])
        #     y = np.int(points[j][1])
        #     cv2.circle(skeleton,(x, y),0,(0,255,255),-1)
        
        # cv2.imwrite('/home/jani/Documents/pics/short_pix'+str(k)+'.tiff', skeleton)
        # k = k+1

        # if dist_int_base > dist_int_end:
        #     if end_ind1 > end_ind2:
        #         short_pix = pix[mid_index+1:end_ind2+1]
        #     else:
        #         short_pix = pix[mid_index+1:end_ind1+1]
        # else:
        #     if end_ind1 < end_ind2:
        #         short_pix = pix[mid_index+1:end_ind2+1]
        #     else:
        #         short_pix = pix[mid_index+1:end_ind1+1]
        pix = [ele for ele in pix if ele not in short_pix]
        pts = (np.reshape(pix, (-1, 2))).tolist()
        points = np.array(pts)
        for j in range(len(points)):
            x = np.int(points[j][0])
            y = np.int(points[j][1])
            cv2.circle(skeleton,(x, y),0,(0,255,255),-1) 

        skeleton = np.where(skeleton == 255, 0, skeleton)

        for j in range(len(points)):
            x = np.int(points[j][0])
            y = np.int(points[j][1])
            cv2.circle(skeleton,(x, y),0,(255,255,255),-1)
        
        skel_branched = skeleton
    
    if (len(intersection) > 0):
        skel = skel_branched

    else:
        skel = skel_no_branch
    
    cv2.imwrite('/home/jani/Documents/pics/final_skel'+str(i)+'.tiff', skel)

    skel = cv2.cvtColor(skel, cv2.COLOR_BGR2GRAY)

    pixels = np.argwhere(skel > 0)
    pixels = np.flip(pixels)
    copy_pix = pixels

    initial_pos = pixels[0] #extract the center of the base marker and get initial position    
    row_index, = np.where(np.all(copy_pix == [initial_pos], axis=1))
    copy_pix = np.delete(copy_pix, row_index, axis=0)
    ordered = initial_pos

    while len(copy_pix)>0:
        distances = np.sqrt((copy_pix[:,0] - initial_pos[0]) ** 2 + (copy_pix[:,1] - initial_pos[1]) ** 2)
#     print(distances)
        nearest_index = np.argmin(distances)
        # print("Point {} matched with {}".format(initial_pos, copy_pix[nearest_index]))
        # print("Nearest index: {}".format(nearest_index))
        initial_pos = copy_pix[nearest_index] #(target[0], target[1])
        # print("Initial Pose", initial_pos)
        copy_pix = np.delete(copy_pix, nearest_index, axis=0)
#         print(copy_pix[0])
        ordered = np.append(ordered, [initial_pos])

    ordered = np.reshape(ordered, (-1, 2))        

    # print("skeleton white pixels after branching", pixels)

    xinit = ordered[:,0]
    yinit = ordered[:,1]

    num_of_segments = rospy.get_param("vsdepth/shape_control/num_of_segments")

    point_jump = (len(xinit))/num_of_segments
    # print(point_jump)   
    x = np.array(xinit[0::round(point_jump)])
    y = np.array(yinit[0::round(point_jump)])   
    if (x[-1] != xinit[-1]) or (y[-1] != yinit[-1]) :
        x = np.append(x, xinit[-1])
        y = np.append(y, yinit[-1])    
        print("x first condition", x)
        print("y first condition", y)
        if len(x) > num_of_segments + 1:
            x = np.delete(x, -2)
            y = np.delete(y, -2)
            print("x second condition", x)
            print("y second condition", y)
    
    print("x and y to be evaluated", x, y)    
    
    tck, u_params = splprep([x, y], k = num_of_segments, s = 1)
    cx = tck[1][0]
    cy = tck[1][1]
    cts = np.c_[cx, cy]
    params = np.linspace(0, 1, 20)

    # new points after evaluation
    newPts =  np.array(splev(params, tck))
    x = newPts[0]
    y = newPts[1]
    points = np.c_[x,y]

    skel = cv2.cvtColor(skel, cv2.COLOR_GRAY2BGR)

    for j in range(len(cts)):
        x = np.int(cts[j][0])
        y = np.int(cts[j][1])
        cv2.circle(skel,(x, y),5,(0,255,255),-1)
    for k in range(len(points)):
        x = np.int(points[k][0])
        y = np.int(points[k][1])
        cv2.circle(skel,(x, y),3,(255,0,0),-1)

    cv2.imwrite('/home/jani/Documents/pics/eval_pts'+str(i)+'.tiff', skel)

    i = i+1
    print("i next", i)


#     24 - [(309, 194), (308, 195), (309, 195)]

# 32 - (309, 194)

# 33 - (322, 208)

# 39 - (308, 194)



    

        

#         cv2.imwrite('/home/jani/Documents/pics/skel_branch'+str(j)+'.jpg', new_img)



    # points = ordered

    # for j in range(len(points)):
    #         x = np.int(points[j][0])
    #         y = np.int(points[j][1])
    #         cv2.circle(skeleton,(x, y),0,(255,255,255),-1)

    # cv2.imwrite('/home/jani/Documents/pics/depth_skel'+str(i)+'.jpg', skeleton)


    
    
    #     points = np.array
    
    # print(dist_int_base, dist_int_end)


    # if (len(skel_coords)>2):



   
#     if (len(skel_coords) > 2) and (len(intersection)>0):
#         # skeleton = cv2.cvtColor(skeleton, cv2.COLOR_GRAY2BGR)
#         cv2.circle(skeleton,(skel_coords[0]),3,(0,0,255),-1) 
#         cv2.circle(skeleton,(skel_coords[1]),3,(0,255,0),-1)
#         cv2.circle(skeleton,(skel_coords[2]),5,(255,255,255),-1)
#         # for i in range(len(intersection)):
#         #     cv2.circle(skeleton,(intersection[i]),5,(255,0,0),-1) 
#         cv2.imwrite('/home/jani/Documents/pics/skel_corner'+str(j) + '.jpg', skeleton)
#         pixels = ordered
#         pix = pixels.tolist()
#         base_index = pix.index([skel_coords[-1][0],skel_coords[-1][1]])
#         end_ind1 = pix.index([skel_coords[0][0],skel_coords[0][1]])
#         end_ind2 = pix.index([skel_coords[1][0],skel_coords[1][1]])
#         mid_index = pix.index([intersection[0][0], intersection[0][1]])
#         print(end_ind1, end_ind2, base_index, mid_index)
#         if end_ind1 < end_ind2:
#             short_pix = pix[mid_index+1:end_ind2+1]
#         else:
#             short_pix = pix[mid_index+1:end_ind1+1]
#         pix = [ele for ele in pix if ele not in short_pix]
#         pts = (np.reshape(pix, (-1, 2))).tolist()
#         points = np.array(pts)
#         new_img = np.zeros((500, 500, 3), dtype = "uint8")
#         for i in range(len(points)):
#             x = np.int(points[i][0])
#             y = np.int(points[i][1])
#             cv2.circle(new_img,(x, y),0,(255,255,255),-1) 
#         cv2.imwrite('/home/jani/Documents/pics/skel_branch'+str(j)+'.jpg', new_img)
#         j = j+1
#         xinit = points[:, 0]
#         yinit = points[:, 1]

#     elif (len(skel_coords) > 2) and (len(intersection)==0):
#         for i in range(len(skel_coords)):
#             cv2.circle(skeleton,(skel_coords[i]),5,(255,0,0),-1) 
#         cv2.imwrite('/home/jani/Documents/pics/skel_corner'+str(j) + '.jpg', skeleton)
#         j = j+1
    
#     else:
#         xinit = ordered[:,0]
#         yinit = ordered[:,1]
    

#     num_of_segments = rospy.get_param("vsdepth/shape_control/num_of_segments")
#     point_jump = (len(xinit))//num_of_segments
#     print(point_jump)
#     x = np.array(xinit[0::round(point_jump)])
#     y = np.array(yinit[0::round(point_jump)])
#     if (x[-1] != xinit[-1]) or (y[-1] != yinit[-1]) :
#         x = np.append(x, xinit[-1])
#         y = np.append(y, yinit[-1])    
#         print(x)
#         print(y)
#         if len(x) > num_of_segments + 1:
#             x = np.delete(x, -2)
#             y = np.delete(y, -2)
#     tck, u_params = splprep([x, y], k = num_of_segments, s = 1)
#     cx = tck[1][0]
#     cy = tck[1][1]
#     cts = np.c_[cx, cy]
#     params = np.linspace(0, 1, 20)
# # ne   w points after evaluation
#     newPts =  np.array(splev(params, tck))
#     x = newPts[0]
#     y = newPts[1]
#     points = np.c_[x,y]
#     for j in range(len(cts)):
#         x = np.int(cts[j][0])
#         y = np.int(cts[j][1])
#         cv2.circle(skeleton,(x, y),5,(0,255,255),-1)
#     for k in range(len(points)):
#         x = np.int(points[k][0])
#         y = np.int(points[k][1])
#         cv2.circle(skeleton,(x, y),3,(255,0,0),-1)
#     cv2.imwrite('/home/jani/Documents/pics/eval_pts'+str(i)+'.jpg', skeleton)

#     else:
#         points = ordered
#         xinit = points[:, 0]
#         yinit = points[:, 1]

#         num_of_segments = rospy.get_param("vsdepth/shape_control/num_of_segments")

#         point_jump = (len(xinit))//num_of_segments
#         print(point_jump)

#         x = np.array(xinit[0::round(point_jump)])
#         y = np.array(yinit[0::round(point_jump)])

#         if (x[-1] != xinit[-1]) or (y[-1] != yinit[-1]) :
#             x = np.append(x, xinit[-1])
#             y = np.append(y, yinit[-1])    
#             print(x)
#             print(y)
#             if len(x) > num_of_segments + 1:
#                 x = np.delete(x, -2)
#                 y = np.delete(y, -2)

#         tck, u_params = splprep([x, y], k = num_of_segments, s = 1)
#         cx = tck[1][0]
#         cy = tck[1][1]
# rospy.Subscriber("/camera/color/image_raw", Image, rgb_image_callback, queue_size=1)
#         cts = np.c_[cx, cy]

#         params = np.linspace(0, 1, 20)
# # ne    w points after evaluation
#         newPts =  np.array(splev(params, tck))
#         x = newPts[0]
#         y = newPts[1]
#         points = np.c_[x,y]


        # for j in range(len(cts)):
        #     x = np.int(cts[j][0])
        #     y = np.int(cts[j][1])
        #     cv2.circle(skeleton,(x, y),5,(0,255,255),-1)

        # for k in range(len(points)):
        #     x = np.int(points[k][0])
        #     y = np.int(points[k][1])
        #     cv2.circle(skeleton,(x, y),3,(255,0,0),-1)
        
    # i = i+1
    





    


    # cv2.imshow("Depth Window 2", depth_img_cropped)
    # cv2.waitKey(2)

    # r = cv2.selectROI(depth_img)
    # print(r[0], r[1], r[2], r[3])
    # depth_roi = depth_img[r[1]:r[1]+r[3], r[0]:r[0]+r[2]]
    # print(np.max(depth_roi))
    # First phase to convert thresholded image to black and white
    # converting the dark grey part which is the robot to white, and anything else to 0

    # cv2.imshow("Depth Window 3", depth_img)
    # cv2.waitKey(2)

    # cv_binary = depth_img

    # # chaging the channel encoding 8UC1 to unsigned integer to use threshold function
    # depth_img = np.array(depth_img, dtype=np.uint8)

    # # Second phase to further converting the image to black and white

    # kernel = np.ones((5, 5), np.uint8)
    # depth_img = cv2.GaussianBlur(depth_img, (5, 5), 0)
    # ret3, cv_binary = cv2.threshold(depth_img, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    # cv_binary = cv2.erode(cv_binary, kernel, iterations=2)
    # cv_binary = cv2.morphologyEx(cv_binary, cv2.MORPH_OPEN, kernel)

    # cv_binary = cv2.morphologyEx(depth_img, cv2.MORPH_CLOSE, kernel)
    # cv_binary = cv2.erode(cv_binary, kernel, iterations=3)
    # cv_binary = cv2.morphologyEx(depth_img, cv2.MORPH_OPEN, kernel)
    # cv_binary = cv2.erode(cv_binary,kernel,iterations = 5)

    # cv2.imshow("CV Binary", cv_binary)
    # cv2.waitKey(2)

    # binary = np.where(cv_binary > 0, 1, cv_binary)
    # skeleton = skeletonize(binary, method='lee')
    # # skeleton = remove_small_objects(skeleton)
    # skeleton = np.where(skeleton == 1, 255, skeleton)

    # new_skel = cv2.cvtColor(skeleton, cv2.COLOR_GRAY2RGB)

    # cv2.imshow("Skeleton", skeleton)
    # cv2.waitKey(2)

    # pixels = np.argwhere(skeleton == 255)

    # xinit = pixels[:, 1]
    # yinit = pixels[:, 0]
    # init_point = (490, 220)  # base pixel used as seed point for NN-search
    # dist_mat = []
    # for i in range(len(xinit)):
    #     dist = np.sqrt((xinit[i]-init_point[0])**2 +
    #                    (yinit[i]-init_point[1])**2)
    #     dist_mat = np.append(dist_mat, dist)

    # index = np.argsort(dist_mat)
    # xinit, yinit = xinit[index], yinit[index]
    # point_jump = (len(xinit))/2
    # x = np.array(xinit[0::round(point_jump)])
    # y = np.array(yinit[0::round(point_jump)])

    # if (x[-1] != xinit[-1]) or (y[-1] != yinit[-1]):
    #     x = np.append(x, xinit[-1])
    #     y = np.append(y, yinit[-1])
    #     if ((x[-1] - x[-2] == 1) or (x[-1] - x[-2] == -1)) and ((y[-1] - y[-2] == 1) or (y[-1] - y[-2] == -1)):
    #         x = np.delete(x, -2)
    #         y = np.delete(y, -2)

    # points = np.c_[x, y]
    # # new_img = np.zeros((480, 848, 3), dtype = "uint8")

    # # for i in range(len(points)):
    # #   x = np.int(points[i][0])
    # #   y = np.int(points[i][1])
    # #   cv2.circle(new_img,(x, y),2,(255,0,0),-1)

    # tck, u_params = splprep([x, y], k=2, s=1)

    # u = np.linspace(0, 1, 20)
    # newPts = splev(u, tck)
    # x = newPts[0]
    # y = newPts[1]

    # points = np.c_[x, y]

    # for i in range(len(points)):
    #     x = np.int(points[i][0])
    #     y = np.int(points[i][1])
    #     cv2.circle(new_skel, (x, y), 3, (0, 0, 255), -1)

    # # final_img = cv2.addWeighted(skeleton, 0.5, new_img, 0.5, 0)

    # cv2.imshow("eval Points", new_skel)
    # cv2.waitKey(2)

# def image_callback(ros_image):
#   global current_ros_image, bridge
#   current_ros_image = ros_image
#   if current_ros_image is not None:
#     print(type(current_ros_image))
#     print(True)
#     current_cv_image = bridge.imgmsg_to_cv2(current_ros_image, '32FC1')
#     depth_img = np.array(current_cv_image, dtype=np.float32)
#     cv2.normalize(depth_img, depth_img, 0, 1, cv2.NORM_MINMAX)

#     cv2.imshow("Depth Window", depth_img)
#     cv2.waitKey(200)

#   else:
#     print("No Image Found")


def main(args):
    # Initialize ROS
    rospy.init_node('image_segmentation')

    image_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, binary_image_service, queue_size=1)
    # image_pub = rospy.Publisher("current_ros_image", Image, queue_size=1)

    # while not rospy.is_shutdown():
    #   if current_ros_image is not None:
    #     image_pub.publish(current_ros_image)

    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)

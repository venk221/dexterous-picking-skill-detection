#!/usr/bin/env python3
from os import defpath
import numpy as np
import sys
from scipy.interpolate import splprep, splev
import rospy
import cv2
import time
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from skimage.morphology import skeletonize, thin, remove_small_objects
from encoderless_vs.srv import bin_img, bin_imgResponse

bridge = CvBridge()
current_ros_image = None
def binary_image_service(ros_image):
    global bridge
    current_ros_image = ros_image
    if current_ros_image is not None:    
      print(True)
    else:
      print("No Image Found")

    depth_img = bridge.imgmsg_to_cv2(current_ros_image, '8UC1') 
    # depth_img = cv2.cvtColor()
    # depth_img = np.array(depth_img, dtype=np.uint8) 
    # depth_img = current_cv_image.astype("float32")
    # cv2.normalize(depth_img, depth_img, 0, 1, cv2.NORM_MINMAX)

    cv2.imshow("Depth Window 1", depth_img)
    cv2.waitKey(2)
    # print(np.max(depth_img))      

    depth_img[0:300, 0:300] = 0
    depth_img[0:80, 230:230+370] = 0
    depth_img[0:480, 550:550+300] = 0
    depth_img[240:240+240, 0:848] = 0

    # depth_img = cv2.bitwise_not(depth_img)

    # r = cv2.selectROI(depth_img)
    # print(r[0], r[1], r[2], r[3])
    # depth_roi = depth_img[r[1]:r[1]+r[3], r[0]:r[0]+r[2]]
    # print(np.max(depth_roi))


    # depth_roi = depth_img[100:100+150, 270: 270+300]

    # Removing the flickers on image
    depth_img = np.where(depth_img < 1, 30, depth_img)

    cv2.imshow("Depth Window 2", depth_img)
    cv2.waitKey(2)

    # r = cv2.selectROI(depth_img)
    # print(r[0], r[1], r[2], r[3])
    # depth_roi = depth_img[r[1]:r[1]+r[3], r[0]:r[0]+r[2]]
    # print(np.max(depth_roi))

    # First phase to convert thresholded image to black and white
    # converting the dark grey part which is the robot to white, and anything else to 0
    depth_img = np.where(depth_img <= 7, 255, depth_img)
    depth_img = np.where(depth_img != 255, 0, depth_img)

    cv2.imshow("Depth Window 3", depth_img)
    cv2.waitKey(2)

    # chaging the channel encoding 8UC1 to unsigned integer to use threshold function
    depth_img = np.array(depth_img, dtype=np.uint8) 

    # Second phase to further converting the image to black and white

    kernel = np.ones((5, 5), np.uint8)
    depth_img = cv2.GaussianBlur(depth_img, (5, 5), 0)
    ret3, cv_binary = cv2.threshold(depth_img, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    cv_binary = cv2.erode(cv_binary, kernel, iterations=2)
    cv_binary = cv2.morphologyEx(cv_binary, cv2.MORPH_OPEN, kernel) 

    # cv_binary = cv2.morphologyEx(depth_img, cv2.MORPH_CLOSE, kernel)
    # cv_binary = cv2.erode(cv_binary, kernel, iterations=3)   
    # cv_binary = cv2.morphologyEx(depth_img, cv2.MORPH_OPEN, kernel)  
    # cv_binary = cv2.erode(cv_binary,kernel,iterations = 5)   

    cv2.imshow("CV Binary", cv_binary)
    cv2.waitKey(2)

    binary = np.where(cv_binary > 0, 1, cv_binary)
    skeleton = skeletonize(binary, method='lee')
    # skeleton = remove_small_objects(skeleton)
    skeleton = np.where(skeleton == 1, 255, skeleton)

    new_skel = cv2.cvtColor(skeleton, cv2.COLOR_GRAY2RGB)

    cv2.imshow("Skeleton", skeleton)
    cv2.waitKey(2)

    pixels = np.argwhere(skeleton == 255)

    xinit = pixels[:, 1]
    yinit = pixels[:, 0]
    init_point = (490, 220)  # base pixel used as seed point for NN-search
    dist_mat = []
    for i in range(len(xinit)):
        dist = np.sqrt((xinit[i]-init_point[0])**2 +
                       (yinit[i]-init_point[1])**2)
        dist_mat = np.append(dist_mat, dist)

    index = np.argsort(dist_mat)
    xinit, yinit = xinit[index], yinit[index]
    point_jump = (len(xinit))/2
    x = np.array(xinit[0::round(point_jump)])
    y = np.array(yinit[0::round(point_jump)])

    if (x[-1] != xinit[-1]) or (y[-1] != yinit[-1]):
        x = np.append(x, xinit[-1])
        y = np.append(y, yinit[-1])
        if ((x[-1] - x[-2] == 1) or (x[-1] - x[-2] == -1)) and ((y[-1] - y[-2] == 1) or (y[-1] - y[-2] == -1)):
            x = np.delete(x, -2)
            y = np.delete(y, -2)

    points = np.c_[x, y]
    # new_img = np.zeros((480, 848, 3), dtype = "uint8")

    # for i in range(len(points)):
    #   x = np.int(points[i][0])
    #   y = np.int(points[i][1])
    #   cv2.circle(new_img,(x, y),2,(255,0,0),-1)

    tck, u_params = splprep([x, y], k=2, s=1)

    u = np.linspace(0, 1, 20)
    newPts = splev(u, tck)
    x = newPts[0]
    y = newPts[1]

    points = np.c_[x, y]

    for i in range(len(points)):
        x = np.int(points[i][0])
        y = np.int(points[i][1])
        cv2.circle(new_skel, (x, y), 3, (0, 0, 255), -1)

    # final_img = cv2.addWeighted(skeleton, 0.5, new_img, 0.5, 0)

    cv2.imshow("eval Points", new_skel)
    cv2.waitKey(2)

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

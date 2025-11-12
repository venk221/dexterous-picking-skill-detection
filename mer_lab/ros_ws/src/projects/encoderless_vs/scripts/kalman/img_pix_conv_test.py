#!/usr/bin/env python3.8

from Kinematics_mod_DH import Kinematics
from scipy.spatial.transform import Rotation as R
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image, CameraInfo, JointState
# from forward import Kinematics
from cv_bridge import CvBridge

bridge = CvBridge()
camera_K = None
p_j2 = None
world_coords = None
img_pixels = None
image_pix = None

def world_coords(joints):    
    global world_coords
    joint_angle = joints.position[0:7]
    # print(type(joint_angle[0]))
    # print("Joint_angles", joint_angle)
    
    # joint_angle = joint_angle[0:7]
    
    kine = Kinematics(joint_angle)
    ht = kine.forward()
    # print("Forward Kinematics Transform", ht)
    
    p_j0 = np.dot(np.eye(4), np.transpose([0, 0, 0, 1]))
    p_j1 = np.dot(ht[0],np.transpose([0, 0, 0, 1]))
    # print(p_j1)
    p_j2 = np.dot(ht[1], np.transpose([0, 0, 0, 1]))
    # print(p_j2)
    p_j3 = np.dot(ht[2], np.transpose([0, 0, 0, 1]))
    p_j4 = np.dot(ht[3], np.transpose([0, 0, 0, 1]))
    p_j5 = np.dot(ht[4], np.transpose([0, 0, 0, 1]))
    p_j6 = np.dot(ht[5], np.transpose([0, 0, 0, 1]))
    p_j7 = np.dot(ht[6], np.transpose([0, 0, 0, 1]))

    world_coords = [p_j0, p_j1, p_j2, p_j3, p_j4, p_j5, p_j6, p_j7]

    # print(p_j1,p_j2,p_j3,p_j4,p_j5,p_j6,p_j7)

def image_cb():

    cv_img = cv2.imread("/home/merlab/DREAM/data/real/panda-3cam_realsense_short/000000.rgb.jpg")

    cv2.imwrite("pix_test_01.jpg", cv_img)

    cv2.circle(cv_img,(415, 442),5,(0, 0, 255),-1)
    cv2.circle(cv_img,(411, 301),10,(0, 255, 0),-1)
    cv2.circle(cv_img,(491, 205),5,(255, 0, 0),-1)
    cv2.circle(cv_img,(476, 178),10,(255, 255, 0),-1)
    cv2.circle(cv_img,(334, 207),5,(0, 255, 255),-1)
    cv2.circle(cv_img,(295, 208),10,(255, 0, 255),-1)
    cv2.circle(cv_img,(294, 265),10,(255, 153, 255),-1)
    # cv2.circle(cv_img,(int(image_pix[7][0]), int(image_pix[7][1])),5,(150, 150, 0),-1)
    

    cv2.imwrite("pix_test_02.jpg", cv_img)


# def camera_intrinsics(camera_info):
#         # Create camera intrinsics matrix
#         global camera_K
#         fx = camera_info.K[0]
#         fy = camera_info.K[4]
#         cx = camera_info.K[2]
#         cy = camera_info.K[5]
#         camera_K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])

# def camera_ht(tvec, quat):

#         rot = R.from_quat(quat).as_matrix()
#         camera_ext = np.array([[rot[0][0], rot[0][1], rot[0][2], tvec[0]],
#                                     [rot[1][0], rot[1][1], rot[1][2], tvec[1]],
#                                     [rot[2][0], rot[2][1], rot[2][2], tvec[2]],
#                                                                 [0, 0, 0, 1]])

#         # camera_ext = np.array([[ 0.92643493, -0.22293637,  0.3033442,  -0.04640514],
#         #                 [-0.34977655, -0.80769267,  0.47464609, -0.71582177],
#         #                 [ 0.13919301, -0.54583141, -0.82625261,  1.74504516],
#         #                 [ 0.   ,       0.     ,     0.       ,   1.        ]])

#         return camera_ext

# def image_pixels(camera_ext, world_coords):
#     global img_pixels
#     print("is image pixels getting called")
#     proj_model = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]]
#     image_pix1 = np.dot(np.dot(camera_K, proj_model), np.dot(camera_ext, world_coords[0]))
#     image_pix2 = np.dot(np.dot(camera_K, proj_model), np.dot(camera_ext, world_coords[1]))
#     image_pix3 = np.dot(np.dot(camera_K, proj_model), np.dot(camera_ext, world_coords[2]))
#     image_pix4 = np.dot(np.dot(camera_K, proj_model), np.dot(camera_ext, world_coords[3]))
#     image_pix5 = np.dot(np.dot(camera_K, proj_model), np.dot(camera_ext, world_coords[4]))
#     image_pix6 = np.dot(np.dot(camera_K, proj_model), np.dot(camera_ext, world_coords[5]))
#     image_pix7 = np.dot(np.dot(camera_K, proj_model), np.dot(camera_ext, world_coords[6]))
#     image_pix8 = np.dot(np.dot(camera_K, proj_model), np.dot(camera_ext, world_coords[7]))

    
#     u1 = image_pix1[0]/image_pix1[2]
#     v1 = image_pix1[1]/image_pix1[2]

#     # u1 = image_pix1[0]
#     # v1 = image_pix1[1]

#     u2 = image_pix2[0]/image_pix2[2]
#     v2 = image_pix2[1]/image_pix2[2]

#     # u2 = image_pix2[0]
#     # v2 = image_pix2[1]

#     u3 = image_pix3[0]/image_pix3[2]
#     v3 = image_pix3[1]/image_pix3[2]

#     # u3 = image_pix3[0]
#     # v3 = image_pix3[1]

#     u4 = image_pix4[0]/image_pix4[2]
#     v4 = image_pix4[1]/image_pix4[2]

#     # u4 = image_pix4[0]
#     # v4 = image_pix4[1]

#     u5 = image_pix5[0]/image_pix5[2]
#     v5 = image_pix5[1]/image_pix5[2]
    
#     # u5 = image_pix5[0]
#     # v5 = image_pix5[1]
    
#     u6 = image_pix6[0]/image_pix6[2]
#     v6 = image_pix6[1]/image_pix6[2]

#     # u6 = image_pix6[0]
#     # v6 = image_pix6[1]

#     u7 = image_pix7[0]/image_pix7[2]
#     v7 = image_pix7[1]/image_pix7[2]

#     # u7 = image_pix7[0]
#     # v7 = image_pix7[1]    

#     u8 = image_pix8[0]/image_pix8[2]
#     v8 = image_pix8[1]/image_pix8[2]

#     # u8 = image_pix8[0]
#     # v8 = image_pix8[1]

#     img_pixels =  [(u1, v1), (u2, v2), (u3, v3), (u4, v4), (u5, v5), (u6, v6), (u7, v7), (u8, v8)]

#     print(img_pixels)

#     return img_pixels

def main():
    global image_pix
    # Initialize the node
    rospy.init_node('image_pix_gen')
    print("is main getting called")
    # subscriber for rgb image to detect markers
    image_cb()
    # cam_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, camera_intrinsics, queue_size=1)
    # joint_sub = rospy.Subscriber("/joint_states", JointState, world_coords, queue_size=1)

    # tvec = [-0.45028442,  0.36399106,  1.79568686]

    # quat = [ 0.94344849, -0.15176051,  0.11726587,  0.27041156] 

    # rate = rospy.Rate(10)
    # while not rospy.is_shutdown():
        
    #     rate.sleep()
    #     print(image_pix)

    # rospy.spin()

if __name__ == "__main__":
    main()
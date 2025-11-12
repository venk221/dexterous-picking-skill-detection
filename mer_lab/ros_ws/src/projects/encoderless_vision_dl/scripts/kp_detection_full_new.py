#!/usr/bin/env python3.8

from DH_v2 import Kinematics
from scipy.spatial.transform import Rotation as R
import numpy as np
import cv2
import rospy
from std_msgs.msg import Float64, Bool, Float64MultiArray
from sensor_msgs.msg import Image, CameraInfo, JointState
# from forward import Kinematics
from cv_bridge import CvBridge
import tf
import tf2_ros as tf2
import json
import os
from datetime import datetime
from os.path import expanduser
from utils import DataPrePro


# global variable used in different functions in the nodes
bridge = CvBridge()
camera_K = None
world_coords = []
img_pixels = None
image_pix = None
joint_vel = None
joint_angle = None
control_flag = None
ros_img = None
status = None
listener = None


dpp = DataPrePro()
# Specifying path to store generated data for future use. User should be able to change the folder names as per requirement
parent_path = dpp.home + "/Pictures/" + "Data/"

# folder name suffix for main dataset with date. As per this every day a new folder will be created to store data
new_path = dpp.year + "-" + dpp.month + "-" + dpp.day + "/"

# folder name suffix to store json files in dream format (might not be required in the end depending on paper narrative)
new_path_dream = "dream" + dpp.year + "-" + dpp.month + "-" + dpp.day + "/"

# folder for main dataset
file_path = os.path.join(parent_path, new_path)

# folder for dream jason files (might be removed in the end depending on paper narrative)
file_path_dream = os.path.join(parent_path, new_path_dream)

# making sure if the target folder exist. If the folder does not exist a new folder will be created else we will continue with the existing folder
if os.path.exists(file_path) and os.path.exists(file_path_dream):
    print("folders exist")
else:
    os.mkdir(file_path)
    os.mkdir(file_path_dream)

# listing the file names in the desired folder in order
files = sorted(os.listdir(file_path))

# block to append new image and json files in an already existing folder for the day
if len(files) == 0:
    i = 0
else:
    i = int(files[-1].split('.')[0])+1


# prefix to the image and json files names
int_stream = "000000"


# this callback is to shutdown the node once the velocity is 0


def stop_cb(msg):
    global status

    status = msg.data

# image call back to read rgb image from camera and


def image_cb(img):
    # print("image_callback getting called")
    global control_flag, ros_img

    ros_img = img
    if ros_img is not None:
        control_flag = True

    # if control_flag == True:

    flag_pub.publish(control_flag)


def kp_gen(flag, img):
    global cv_img, bridge, i, file_path, file_path_dream
    print(f"image number: {i}")

    # print("kp_gen is called")
    if flag == True:
        cv_img = bridge.imgmsg_to_cv2(img, "rgb8")

        new_stream = int_stream[0:-len(str(i))]
        cv2.imwrite(file_path + new_stream + str(i) + ".rgb.jpg", cv_img)
        image_file = new_stream + str(i) + ".rgb.jpg"
        id = i

        data = {"id": id,
                "image_rgb": image_file,
                "bboxes": [
                    [
                        image_pix[0][0]-10,
                        image_pix[0][1]-10,
                        image_pix[0][0]+10,
                        image_pix[0][1]+10
                    ],
                    [
                        image_pix[2][0]-10,
                        image_pix[2][1]-10,
                        image_pix[2][0]+10,
                        image_pix[2][1]+10
                    ],
                    [
                        image_pix[3][0]-10,
                        image_pix[3][1]-10,
                        image_pix[3][0]+10,
                        image_pix[3][1]+10
                    ],
                    [
                        image_pix[4][0]-10,
                        image_pix[4][1]-10,
                        image_pix[4][0]+10,
                        image_pix[4][1]+10
                    ],
                    [
                        image_pix[5][0]-10,
                        image_pix[5][1]-10,
                        image_pix[5][0]+10,
                        image_pix[5][1]+10
                    ],
                    [
                        image_pix[6][0]-10,
                        image_pix[6][1]-10,
                        image_pix[6][0]+10,
                        image_pix[6][1]+10
                    ]
                ],
                "keypoints": [
                    [
                        [
                            image_pix[0][0],
                            image_pix[0][1],
                            1
                        ]
                    ],
                    [
                        [
                            image_pix[2][0],
                            image_pix[2][1],
                            1
                        ]
                    ],
                    [
                        [
                            image_pix[3][0],
                            image_pix[3][1],
                            1
                        ]
                    ],
                    [
                        [
                            image_pix[4][0],
                            image_pix[4][1],
                            1
                        ]
                    ],
                    [
                        [
                            image_pix[5][0],
                            image_pix[5][1],
                            1
                        ]
                    ],
                    [
                        [
                            image_pix[6][0],
                            image_pix[6][1],
                            1
                        ]
                    ]
                ]
                }

        json_obj = json.dumps(data, indent=4)
        filename = file_path + new_stream + str(i)+".json"
        with open(filename, "w") as outfile:
            outfile.write(json_obj)

    i = i + 1


def world_coords_tf_old(joints):
    global world_coords, joint_angle, joint_vel
    joint_angle = joints.position
    joint_vel = joints.velocity
    # print(type(joint_angle[0]))
    # print("Joint_angles", joint_angle)

    # joint_angle = joint_angle[0:7]
    # print("joint_angle", joint_angle)

    kine = Kinematics(joint_angle)
    ht = kine.forward()
    # print("Forward Kinematics Transform", ht)
    # print("before __________ht[6]",ht[6])
    # ht[6][1][3] += 0.2
    # print("after____________ht[6]",ht[6])
    # ht[6][1][3] += -0.0636

    # p_j0 = np.dot(np.eye(4), np.transpose([0, 0, 0, 1]))  # 4x4 4x1 = 4x1
    # p_j1 = np.dot(ht[0], np.transpose([0, 0, 0, 1]))
    # # print(p_j1)
    # p_j2 = np.dot(ht[1], np.transpose([0, 0, 0, 1]))
    # # print(p_j2)
    # p_j3 = np.dot(ht[2], np.transpose([0, 0, 0, 1]))
    # p_j4 = np.dot(ht[3], np.transpose([0, 0, 0, 1]))
    # p_j5 = np.dot(ht[4], np.transpose([0, 0, 0, 1]))
    # p_j6 = np.dot(ht[5], np.transpose([0, 0, 0, 1]))
    # p_j7 = np.dot(ht[6], np.transpose([0, 0, 0, 1]))

    # T_rf = np.array([[0.7071, 0.7071, 0, 0],
    #                 [-0.7071, 0.7071, 0, 0],
    #                 [0, 0, 1, 0.1],
    #                 [0, 0, 0, 1]])

    # p_j8 = np.dot(ht[6] @ T_rf, np.transpose([0, 0.1, 0, 1]))
    # p_j9 = np.dot(ht[6] @ T_rf, np.transpose([0, -0.1, 0, 1]))

    ##

    # T_rs = np.array([[0.7071096, 0.7071040, 0, 0.09],
    #                 [-0.7071040, 0.7071096, 0, 0],
    #                 [0, 0, 1, 0.1070],
    #                 [0, 0, 0, 1]])

    # T_ls = np.array([[0.7071096, 0.7071040, 0,  -0.09],
    #                 [-0.7071040, 0.7071096, 0, 0],
    #                 [0, 0, 1, 0.1070],
    #                 [0, 0, 0, 1]])

    # T_rf = np.array([[0.7071096, 0.7071040, 0, joint_angle[7]],
    #                 [-0.7071040, 0.7071096, 0, 0],
    #                 [0, 0, 1, 0.20],
    #                 [0, 0, 0, 1]])

    # T_lf = np.array([[0.7071096, 0.7071040, 0, -joint_angle[8]],
    #                 [-0.7071040, 0.7071096, 0, 0],
    #                 [0, 0, 1, 0.20],  # 1654
    #                 [0, 0, 0, 1]])

    ##

    T_rs = np.array([[1, 0, 0, 0.09],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.1070],
                    [0, 0, 0, 1]])

    T_ls = np.array([[1, 0, 0,  -0.09],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.1070],
                    [0, 0, 0, 1]])

    T_rf = np.array([[1, 0, 0, joint_angle[7]],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.20],
                    [0, 0, 0, 1]])

    T_lf = np.array([[1, 0, 0, -joint_angle[8]],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.20],  # 1654
                    [0, 0, 0, 1]])

    # print("ht[6]:     ", ht[6])
    # print("ht[7]:     ", ht[7])

    T_rs1 = np.dot(ht[6], T_rs)
    T_ls1 = np.dot(ht[6], T_ls)

    p_j8 = np.dot(T_rs1, np.transpose([0, 0, 0, 1]))
    p_j9 = np.dot(T_ls1, np.transpose([0, 0, 0, 1]))
    p_j10 = np.dot(ht[6] @ T_rf, np.transpose([0, 0, 0, 1]))
    p_j11 = np.dot(ht[6] @ T_lf, np.transpose([0, 0, 0, 1]))

    world_coords = [np.eye(4), ht[0], ht[1], ht[2], ht[3],
                    ht[4], ht[5], ht[6], T_rs1, T_ls1, ht[6] @ T_rf, ht[6] @ T_lf]

    # world_coords = [p_j0, p_j1, p_j2, p_j3, p_j4,
    #                 p_j5, p_j6, p_j7, p_j8, p_j9, p_j10, p_j11]


def world_coords_tf(joints):

    # print("get world_coords_tf")
    global listener, world_coords
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans1, _) = listener.lookupTransform(
                '/panda_link0', '/panda_link1', rospy.Time(0))
            (trans2, _) = listener.lookupTransform(
                '/panda_link0', '/panda_link2', rospy.Time(0))
            (trans3, _) = listener.lookupTransform(
                '/panda_link0', '/panda_link3', rospy.Time(0))
            (trans4, _) = listener.lookupTransform(
                '/panda_link0', '/panda_link4', rospy.Time(0))
            (trans5, _) = listener.lookupTransform(
                '/panda_link0', '/panda_link5', rospy.Time(0))
            (trans6, _) = listener.lookupTransform(
                '/panda_link0', '/panda_link6', rospy.Time(0))
            (trans7, _) = listener.lookupTransform(
                '/panda_link0', '/panda_link7', rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        trans1.append(1)
        trans2.append(1)
        trans3.append(1)
        trans4.append(1)
        trans5.append(1)
        trans6.append(1)
        trans7.append(1)
        world_coords = [trans1, trans2, trans3, trans4, trans5, trans6, trans7]
        # row_vector1 = np.array(trans1)
        # col_vector1 = np.column_stack([row_vector1])

        # row_vector2 = np.array(trans2)
        # col_vector2 = np.column_stack([row_vector2])

        # row_vector3 = np.array(trans3)
        # col_vector3 = np.column_stack([row_vector3])

        # row_vector4 = np.array(trans4)
        # col_vector4 = np.column_stack([row_vector4])

        # row_vector5 = np.array(trans5)
        # col_vector5 = np.column_stack([row_vector5])

        # row_vector6 = np.array(trans6)
        # col_vector6 = np.column_stack([row_vector6])

        # row_vector7 = np.array(trans7)
        # col_vector7 = np.column_stack([row_vector7])

        # world_coords = [row_vector1, row_vector2, row_vector3, row_vector4, row_vector5, row_vector6, row_vector7]
        # world_coords = [col_vector1, col_vector2, col_vector3,
        #                 col_vector4, col_vector5, col_vector6, col_vector7]
        
        # print("world_coords",world_coords)


def camera_intrinsics(camera_info):
    # Create camera intrinsics matrix
    global camera_K
    fx = camera_info.K[0]
    fy = camera_info.K[4]
    cx = camera_info.K[2]
    cy = camera_info.K[5]
    camera_K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])


# homogenous tranformation from 4X1 translation and
def transform(tvec, quat):

    r = R.from_quat(quat).as_matrix()

    ht = np.array([[r[0][0], r[0][1], r[0][2], tvec[0]],
                   [r[1][0], r[1][1], r[1][2], tvec[1]],
                   [r[2][0], r[2][1], r[2][2], tvec[2]]])

    # print("*************ht**************", ht)

    return ht


# using the camera extrinsics from DREAM, and the world coords from tf transformation in this function we are creating the
def image_pixels(camera_ext, world_coords):
    global img_pixels

    # print("start getting image_pixels")

    # print("camera_K.shape, camera_ext.shape, world_coords[0].shape",camera_K.shape, camera_ext.shape, world_coords[0].shape)

    # 3x3  4x4 =3x4   3x4 4x1 = 3x1
    image_pix1 = np.dot(np.dot(camera_K, camera_ext), world_coords[0])
    image_pix2 = np.dot(np.dot(camera_K, camera_ext), world_coords[1])
    image_pix3 = np.dot(np.dot(camera_K, camera_ext), world_coords[2])
    image_pix4 = np.dot(np.dot(camera_K, camera_ext), world_coords[3])
    image_pix5 = np.dot(np.dot(camera_K, camera_ext), world_coords[4])
    image_pix6 = np.dot(np.dot(camera_K, camera_ext), world_coords[5])
    image_pix7 = np.dot(np.dot(camera_K, camera_ext), world_coords[6])

    u1 = image_pix1[0]/image_pix1[2]
    v1 = image_pix1[1]/image_pix1[2]

    u2 = image_pix2[0]/image_pix2[2]
    v2 = image_pix2[1]/image_pix2[2]

    u3 = image_pix3[0]/image_pix3[2]
    v3 = image_pix3[1]/image_pix3[2]

    u4 = image_pix4[0]/image_pix4[2]
    v4 = image_pix4[1]/image_pix4[2]

    u5 = image_pix5[0]/image_pix5[2]
    v5 = image_pix5[1]/image_pix5[2]

    u6 = image_pix6[0]/image_pix6[2]
    v6 = image_pix6[1]/image_pix6[2]

    u7 = image_pix7[0]/image_pix7[2]
    v7 = image_pix7[1]/image_pix7[2]

    img_pixels = [(u1, v1), (u2, v2), (u3, v3), (u4, v4),
                  (u5, v5), (u6, v6), (u7, v7)]

    # print("**************img_pixels*******function******", img_pixels)

    return img_pixels

# latest changes


def main():
    global image_pix, flag_pub, listener
    # Initialize the node
    rospy.init_node('image_pix_gen')

    print("is main getting called")

    listener = tf.TransformListener()
    # subscriber for rgb image to detect markers
    image_sub = rospy.Subscriber(
        "/camera/color/image_raw", Image, image_cb, queue_size=1)
    cam_info_sub = rospy.Subscriber(
        "/camera/color/camera_info", CameraInfo, camera_intrinsics, queue_size=1)
    joint_sub = rospy.Subscriber(
        "/joint_states", JointState, world_coords_tf, queue_size=1)

    status_sub = rospy.Subscriber("/node/stop", Float64, stop_cb, queue_size=1)

    # publisher to publish flag to start control points svc
    flag_pub = rospy.Publisher("/franka/control_flag", Bool, queue_size=1)

    # tvec = [-0.21874574,  0.45616769,  1.46707864]
    # quat = [0.6841951,   0.02721269, -0.04761625,  0.72723395]
    
    tvec = [-0.21843653,  0.4612956,   1.48258252]
    quat = [ 0.70106285, -0.04892933,  0.02925397,  0.71081714]





    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        # print("Camera K", camera_K)
        if camera_K is not None and control_flag == True:
            camera_ext = transform(tvec, quat)
            # print("camera_ext", camera_ext)

            image_pix = image_pixels(camera_ext, world_coords)
            # print("-----------image_pix-----main-----",image_pix)
            kp_gen(control_flag, ros_img)

        if status == 0.0:
            rospy.signal_shutdown("I have good reason!")
        rate.sleep()
        # print(image_pix)

    rospy.spin()


if __name__ == "__main__":
    main()

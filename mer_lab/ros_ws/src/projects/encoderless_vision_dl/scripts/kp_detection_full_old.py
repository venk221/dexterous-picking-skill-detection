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
world_coords = None
img_pixels = None
image_pix = None
joint_vel = None
joint_angle = None
control_flag = None
ros_img = None
status = None

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

    print("kp_gen is called")
    if flag == True:
        cv_img = bridge.imgmsg_to_cv2(img, "rgb8")

        new_stream = int_stream[0:-len(str(i))]
        cv2.imwrite(file_path + new_stream + str(i) + ".rgb.jpg", cv_img)
        image_file = new_stream + str(i) + ".rgb.jpg"
        id = i
        data_dream = {
            "objects": [
                {
                    "class": "panda",
                    "visibility": 1,
                    "location": [
                        world_coords[0][0],
                        world_coords[0][1],
                        world_coords[0][2]
                    ],
                    "keypoints": [
                        {
                            "name": "panda_link0",
                            "location": [
                                world_coords[0][0],
                                world_coords[0][1],
                                world_coords[0][2]

                            ],
                            "projected_location": [
                                image_pix[0][0],
                                image_pix[0][1]
                            ]
                        },
                        {
                            "name": "panda_link2",
                            "location": [
                                world_coords[2][0],
                                world_coords[2][1],
                                world_coords[2][2]

                            ],
                            "projected_location": [
                                image_pix[2][0],
                                image_pix[2][1]
                            ]
                        },
                        {
                            "name": "panda_link3",
                            "location": [
                                world_coords[3][0],
                                world_coords[3][1],
                                world_coords[3][2]

                            ],
                            "projected_location": [
                                image_pix[3][0],
                                image_pix[3][1]
                            ]
                        },
                        {
                            "name": "panda_link4",
                            "location": [
                                world_coords[4][0],
                                world_coords[4][1],
                                world_coords[4][2]

                            ],
                            "projected_location": [
                                image_pix[4][0],
                                image_pix[4][1]
                            ]
                        },
                        {
                            "name": "panda_link6",
                            "location": [
                                world_coords[6][0],
                                world_coords[6][1],
                                world_coords[6][2]

                            ],
                            "projected_location": [
                                image_pix[6][0],
                                image_pix[6][1]
                            ]
                        },
                        {
                            "name": "panda_link7",
                            "location": [
                                world_coords[7][0],
                                world_coords[7][1],
                                world_coords[7][2]

                            ],
                            "projected_location": [
                                image_pix[7][0],
                                image_pix[7][1]
                            ]
                        },
                        {
                            "name": "panda_rightfinger",
                            "location": [
                                world_coords[8][0],
                                world_coords[8][1],
                                world_coords[8][2]

                            ],
                            "projected_location": [
                                image_pix[8][0],
                                image_pix[8][1]
                            ]
                        },
                        {
                            "name": "panda_leftfinger",
                            "location": [
                                world_coords[9][0],
                                world_coords[9][1],
                                world_coords[9][2]

                            ],
                            "projected_location": [
                                image_pix[9][0],
                                image_pix[9][1]
                            ]
                        },
                        {
                            "name": "panda_leftfinger",
                            "location": [
                                world_coords[10][0],
                                world_coords[10][1],
                                world_coords[10][2]

                            ],
                            "projected_location": [
                                image_pix[10][0],
                                image_pix[10][1]
                            ]
                        },
                        {
                            "name": "panda_leftfinger",
                            "location": [
                                world_coords[11][0],
                                world_coords[11][1],
                                world_coords[11][2]

                            ],
                            "projected_location": [
                                image_pix[11][0],
                                image_pix[11][1]
                            ]
                        }
                    ]
                }
            ],
            "sim_state": {
                "joints": [
                    {
                        "name": "panda_joint1",
                        "position": joint_angle[0],
                        "velocity": joint_vel[0]
                    },
                    {
                        "name": "panda_joint2",
                        "position": joint_angle[1],
                        "velocity": joint_vel[1]
                    },
                    {
                        "name": "panda_joint3",
                        "position": joint_angle[2],
                        "velocity": joint_vel[2]
                    },
                    {
                        "name": "panda_joint4",
                        "position": joint_angle[3],
                        "velocity": joint_vel[3]
                    },
                    {
                        "name": "panda_joint5",
                        "position": joint_angle[4],
                        "velocity": joint_vel[4]
                    },
                    {
                        "name": "panda_joint6",
                        "position": joint_angle[5],
                        "velocity": joint_vel[5]
                    },
                    {
                        "name": "panda_joint7",
                        "position": joint_angle[6],
                        "velocity": joint_vel[6]
                    }
                ]
            }
        }

        json_obj_dream = json.dumps(data_dream, indent=4)
        filename_dream = file_path_dream + new_stream + str(i)+".json"
        with open(filename_dream, "w") as outfile:
            outfile.write(json_obj_dream)

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
                        image_pix[6][0]-10,
                        image_pix[6][1]-10,
                        image_pix[6][0]+10,
                        image_pix[6][1]+10
                    ],
                    [
                        image_pix[7][0]-10,
                        image_pix[7][1]-10,
                        image_pix[7][0]+10,
                        image_pix[7][1]+10
                    ],
                    [
                        image_pix[8][0]-10,
                        image_pix[8][1]-10,
                        image_pix[8][0]+10,
                        image_pix[8][1]+10
                    ],
                    [
                        image_pix[9][0]-10,
                        image_pix[9][1]-10,
                        image_pix[9][0]+10,
                        image_pix[9][1]+10
                    ],
                    [
                        image_pix[10][0]-10,
                        image_pix[10][1]-10,
                        image_pix[10][0]+10,
                        image_pix[10][1]+10
                    ],
                    [
                        image_pix[11][0]-10,
                        image_pix[11][1]-10,
                        image_pix[11][0]+10,
                        image_pix[11][1]+10
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
                            image_pix[6][0],
                            image_pix[6][1],
                            1
                        ]
                    ],
                    [
                        [
                            image_pix[7][0],
                            image_pix[7][1],
                            1
                        ]
                    ],
                    [
                        [
                            image_pix[8][0],
                            image_pix[8][1],
                            1
                        ]
                    ],
                    [
                        [
                            image_pix[9][0],
                            image_pix[9][1],
                            1
                        ]
                    ],
                    [
                        [
                            image_pix[10][0],
                            image_pix[10][1],
                            1
                        ]
                    ],
                    [
                        [
                            image_pix[11][0],
                            image_pix[11][1],
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


def world_coords_tf(joints):
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

    p_j0 = np.dot(np.eye(4), np.transpose([0, 0, 0, 1]))  # 4x4 4x1 = 4x1
    p_j1 = np.dot(ht[0], np.transpose([0, 0, 0, 1]))
    # print(p_j1)
    p_j2 = np.dot(ht[1], np.transpose([0, 0, 0, 1]))
    # print(p_j2)
    p_j3 = np.dot(ht[2], np.transpose([0, 0, 0, 1]))
    p_j4 = np.dot(ht[3], np.transpose([0, 0, 0, 1]))
    p_j5 = np.dot(ht[4], np.transpose([0, 0, 0, 1]))
    p_j6 = np.dot(ht[5], np.transpose([0, 0, 0, 1]))
    p_j7 = np.dot(ht[6], np.transpose([0, 0, 0, 1]))

    # T_rf = np.array([[0.7071, 0.7071, 0, 0],
    #                 [-0.7071, 0.7071, 0, 0],
    #                 [0, 0, 1, 0.1],
    #                 [0, 0, 0, 1]])

    # p_j8 = np.dot(ht[6] @ T_rf, np.transpose([0, 0.1, 0, 1]))
    # p_j9 = np.dot(ht[6] @ T_rf, np.transpose([0, -0.1, 0, 1]))

    T_rs = np.array([[0.7071096, 0.7071040, 0, 0.09],
                    [-0.7071040, 0.7071096, 0, 0],
                    [0, 0, 1, 0.1070],
                    [0, 0, 0, 1]])

    T_ls = np.array([[0.7071096, 0.7071040, 0,  -0.09],
                    [-0.7071040, 0.7071096, 0, 0],
                    [0, 0, 1, 0.1070],
                    [0, 0, 0, 1]])

    T_rf = np.array([[0.7071096, 0.7071040, 0, joint_angle[7]],
                    [-0.7071040, 0.7071096, 0, 0],
                    [0, 0, 1, 0.20],
                    [0, 0, 0, 1]])

    T_lf = np.array([[0.7071096, 0.7071040, 0, -joint_angle[8]],
                    [-0.7071040, 0.7071096, 0, 0],
                    [0, 0, 1, 0.20],  # 1654
                    [0, 0, 0, 1]])

    print("ht[6]:     ", ht[6])
    print("ht[7]:     ", ht[7])

    T_rs1 = np.dot(ht[6], T_rs)
    T_ls1 = np.dot(ht[6], T_ls)

    p_j8 = np.dot(T_rs1, np.transpose([0, 0, 0, 1]))
    p_j9 = np.dot(T_ls1, np.transpose([0, 0, 0, 1]))
    p_j10 = np.dot(ht[6] @ T_rf, np.transpose([0, 0, 0, 1]))
    p_j11 = np.dot(ht[6] @ T_lf, np.transpose([0, 0, 0, 1]))

    world_coords = [p_j0, p_j1, p_j2, p_j3, p_j4,
                    p_j5, p_j6, p_j7, p_j8, p_j9, p_j10, p_j11]


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
                   [r[2][0], r[2][1], r[2][2], tvec[2]],
                   [0, 0, 0, 1]])

    return ht


# using the camera extrinsics from DREAM, and the world coords from tf transformation in this function we are creating the
def image_pixels(camera_ext, world_coords):
    global img_pixels
    # print("is image pixels getting called")
    proj_model = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]]  # 3x4
    image_pix1 = np.dot(np.dot(camera_K, proj_model),  # 3x3 3x4 =3x4
                        np.dot(camera_ext, world_coords[0]))  # 4x4  4x1 =4x1    3x4 4x1 = 3x1
    image_pix2 = np.dot(np.dot(camera_K, proj_model),
                        np.dot(camera_ext, world_coords[1]))
    image_pix3 = np.dot(np.dot(camera_K, proj_model),
                        np.dot(camera_ext, world_coords[2]))
    image_pix4 = np.dot(np.dot(camera_K, proj_model),
                        np.dot(camera_ext, world_coords[3]))
    image_pix5 = np.dot(np.dot(camera_K, proj_model),
                        np.dot(camera_ext, world_coords[4]))
    image_pix6 = np.dot(np.dot(camera_K, proj_model),
                        np.dot(camera_ext, world_coords[5]))
    image_pix7 = np.dot(np.dot(camera_K, proj_model),
                        np.dot(camera_ext, world_coords[6]))
    image_pix8 = np.dot(np.dot(camera_K, proj_model),
                        np.dot(camera_ext, world_coords[7]))

    image_pix9 = np.dot(np.dot(camera_K, proj_model),
                        np.dot(camera_ext, world_coords[8]))
    image_pix10 = np.dot(np.dot(camera_K, proj_model),
                         np.dot(camera_ext, world_coords[9]))

    image_pix11 = np.dot(np.dot(camera_K, proj_model),
                         np.dot(camera_ext, world_coords[10]))
    image_pix12 = np.dot(np.dot(camera_K, proj_model),
                         np.dot(camera_ext, world_coords[11]))

    u1 = image_pix1[0]/image_pix1[2]
    v1 = image_pix1[1]/image_pix1[2]

    # u1 = image_pix1[0]
    # v1 = image_pix1[1]

    u2 = image_pix2[0]/image_pix2[2]
    v2 = image_pix2[1]/image_pix2[2]

    # u2 = image_pix2[0]
    # v2 = image_pix2[1]

    u3 = image_pix3[0]/image_pix3[2]
    v3 = image_pix3[1]/image_pix3[2]

    # u3 = image_pix3[0]
    # v3 = image_pix3[1]

    u4 = image_pix4[0]/image_pix4[2]
    v4 = image_pix4[1]/image_pix4[2]

    # u4 = image_pix4[0]
    # v4 = image_pix4[1]

    u5 = image_pix5[0]/image_pix5[2]
    v5 = image_pix5[1]/image_pix5[2]

    # u5 = image_pix5[0]
    # v5 = image_pix5[1]

    u6 = image_pix6[0]/image_pix6[2]
    v6 = image_pix6[1]/image_pix6[2]

    # u6 = image_pix6[0]
    # v6 = image_pix6[1]

    u7 = image_pix7[0]/image_pix7[2]
    v7 = image_pix7[1]/image_pix7[2]

    # u7 = image_pix7[0]
    # v7 = image_pix7[1]

    u8 = image_pix8[0]/image_pix8[2]
    v8 = image_pix8[1]/image_pix8[2]

    u9 = image_pix9[0]/image_pix9[2]
    v9 = image_pix9[1]/image_pix9[2]

    u10 = image_pix10[0]/image_pix10[2]
    v10 = image_pix10[1]/image_pix10[2]

    u11 = image_pix11[0]/image_pix11[2]
    v11 = image_pix11[1]/image_pix11[2]

    u12 = image_pix12[0]/image_pix12[2]
    v12 = image_pix12[1]/image_pix12[2]

    # u8 = image_pix8[0]
    # v8 = image_pix8[1]

    img_pixels = [(u1, v1), (u2, v2), (u3, v3), (u4, v4),
                  (u5, v5), (u6, v6), (u7, v7), (u8, v8), (u9, v9), (u10, v10), (u11, v11), (u12, v12)]

    return img_pixels

# latest changes


def main():
    global image_pix, flag_pub
    # Initialize the node
    rospy.init_node('image_pix_gen')
    print("is main getting called")
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

    # kp_gen(control_flag, ros_img)

    # tvec =  [-0.15090792,  0.52414599,  1.67750524]
    # quat = [0.66799252,  0.05898101, -0.10356415,  0.73456225]

    # 27 dec 2022
    # tvec = [-0.07670547,  0.50127457,  1.79583101]
    # quat = [0.68015778,  0.06770412, -0.1031019,   0.72261438]

    # tvec = [-0.07291735,  0.50500857,  1.83304405]
    # quat = [0.73619799,  0.02631464, -0.05072662,  0.67434922]

    # tvec = [-0.05802229,  0.50011217,  1.80114979]
    # quat = [0.65072221,  0.0964,     -0.13362087,  0.74122406]

    # tvec = [-0.05426875,  0.4965772,   1.85036886]
    # quat = [0.71194839,  0.08611222, -0.12149673,  0.68625996]

    # tvec = [-0.0608392,   0.4931465,   1.84891267]
    # quat = [0.69541326,  0.04393566, -0.08063294,  0.71404092]

    # tvec = [-0.18813038,  0.47515079,  1.80869346]
    # quat = [0.69431181,  0.07807451, -0.10610137,  0.70751536]

    # 02/02/2023  16.26pm
    # tvec = [-0.14415551,  0.39509832,  1.84163159]
    # quat = [ 0.72934882,  0.05642004, -0.09895423,  0.67459257]

    # 06/02/2023
    # tvec = [-0.11424774,  0.48471736,  1.79594838]
    # quat = [ 0.67265499,  0.07037732, -0.09695493,  0.73019315]

    # 07/02/2023
    # tvec = [-0.12371827,  0.48666955,  1.80306777]
    # quat = [0.68184661,  0.07548561, -0.10030204,  0.72064321]

    # 11/02/2023

    # tvec = [-0.14974297,  0.47407533,  1.92068106]
    # quat = [ 0.66754875,  0.02926382, -0.04842398,  0.74241323]

    # # 27-02-2023
    # tvec = [-0.20175074,  0.51466594,  1.85322199]
    # quat = [ 0.67885882,  0.05601674, -0.07257142,  0.72852331]

    # v1
    # tvec = [-0.20116826,  0.51271516,  1.84604358]
    # quat = [0.67797057,  0.04443369, -0.0609439,   0.73120954]

    # v2
    # tvec = [-0.20184116,  0.51401167,  1.84848969]
    # quat = [ 0.67632568,  0.04766259, -0.06358458,  0.7323038 ]

    # v3
    # tvec = [-0.19944929,  0.51294971,  1.82863287]
    # quat = [0.67037166,  0.04217194, -0.06012516,  0.73838224]

    # v4
    # tvec = [-0.19882835,  0.5125928,   1.82504029]
    # quat = [ 0.66921921,  0.03359269, -0.05151849,  0.74051538]

    # # v5
    # tvec = [-0.19910065,  0.51181459,  1.82653237]
    # quat = [0.67003574,  0.03509775, -0.05366591,  0.73955407]

    # v6
    # tvec = [-0.19893252,  0.513339,    1.82858971]
    # quat = [0.67069601,  0.04083397, -0.05952304,  0.73821166]

    # # v7
    # tvec = [-0.19935884,  0.51262629,  1.82772949]
    # quat = [ 0.67545937,  0.03870408, -0.05524949,  0.7343052 ]

    # v8
    # tvec = [-0.200066,    0.51393765,  1.8316991 ]
    # quat = [ 0.67496498,  0.04145652, -0.05737199,  0.73444679]

    # v9
    # tvec = [-0.19865176,  0.51125077,  1.82224817]
    # quat = [ 0.66891006,  0.02788303, -0.04619032,  0.74138271]

    # v10
    tvec = [-0.27606041,  0.51939796,  1.77690649]
    quat = [0.63820991,  0.00661443, -0.02881552,  0.76929449]

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        print("Camera K", camera_K)
        if camera_K is not None and control_flag == True:
            camera_ext = transform(tvec, quat)
            print("call image_pix")
            # print("camera_ext", camera_ext)
            image_pix = image_pixels(camera_ext, world_coords)
            print(image_pix)
            print("call kp_gen")
            kp_gen(control_flag, ros_img)

        if status == 0.0:
            rospy.signal_shutdown("I have good reason!")
        rate.sleep()
        # print(image_pix)

    rospy.spin()


if __name__ == "__main__":
    main()

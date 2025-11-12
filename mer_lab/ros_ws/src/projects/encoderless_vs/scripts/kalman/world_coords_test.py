#!/usr/bin/env python3.8


from Kinematics_mod_DH import Kinematics
from scipy.spatial.transform import Rotation as R
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image, CameraInfo, JointState
# from forward import Kinematics
from cv_bridge import CvBridge
import tf
import tf2_ros as tf2

    # print(p_j1,p_j2,p_j3,p_j4,p_j5,p_j6,p_j7)

ht = None

def camera_ht(tvec, quat):

    rot = R.from_quat(quat).as_matrix()
    camera_ext = np.array([[rot[0][0], rot[0][1], rot[0][2], tvec[0]],
                                    [rot[1][0], rot[1][1], rot[1][2], tvec[1]],
                                    [rot[2][0], rot[2][1], rot[2][2], tvec[2]],
                                                                [0, 0, 0, 1]])

    return camera_ext

def world_coords_tf():
    t0 = [0.000, 0.000, 0.000]
    rot0 = [0.000, 0.000, 0.000, 1.000]

    ht0 = camera_ht(t0, rot0)

    print("ht0 = ", ht0)
    

    t1 = [0.000, 0.000, 0.333]
    rot1 = [0.000, 0.000, 0.001, 1.000]

    ht1 = camera_ht(t1, rot1)

    print("ht1 = ", ht1)
    print("ht[0]", ht[0])    

    t2 = [0.000, 0.000, 0.333]
    rot2 = [-0.679, -0.196, -0.195, 0.680]


    ht2 = camera_ht(t2, rot2)

    print("ht2 = ", ht2)
    print("ht[1] = ", ht[1])   

    t3 = [-0.168, -0.000, 0.601]
    rot3 = [-0.000, -0.277, 0.003, 0.961]

    ht3 = camera_ht(t3, rot3)

    print("ht3 = ", ht3)
    print("ht[2]= ", ht[2])    

    t4 =  [-0.098, 0.000, 0.644]
    rot4 = [0.608, 0.360, -0.357, 0.611]

    ht4 = camera_ht(t4, rot4)

    print("ht4 = ", ht4)
    print("ht[3] = ", ht[3])    

    t5 = [0.198, 0.003, 0.903]
    rot5 = [-0.005, 0.507, -0.002, 0.862]

    ht5 = camera_ht(t5, rot5)

    print("ht5 = ", ht5)
    print("ht[4] = ", ht[4])    

    t6 = [0.198, 0.003, 0.903]

    rot6 = [0.641, -0.290, 0.294, 0.647]

    ht6 = camera_ht(t6, rot6)

    print("ht6 = ", ht6)
    print("ht[5] = ", ht[5])    

    t7 = [0.256, 0.003, 0.969]
    rot7 = [0.821, 0.394, 0.370, 0.182]

    ht7 = camera_ht(t7, rot7)

    print("ht7 = ", ht7)
    print("ht[6] = ", ht[6])

# def world_coords_tf_cb():

#     tfBuffer = tf2.Buffer()
#     listener = tf2.TransformListener(tfBuffer)

#     (trans, rot) = tfBuffer.lookup_transform('panda_link0', 'panda_link1', rospy.Time())

#     print("Translation:", trans)
#     print("Rotation:", rot)
    

# def world_coords(joints):    
#     global world_coords, ht
#     joint_angle = joints.position[0:7]
#     # print(type(joint_angle[0]))
#     # print("Joint_angles", joint_angle)
    
#     # joint_angle = joint_angle[0:7]
    
#     kine = Kinematics(joint_angle)
#     ht = kine.forward()
#     # print("Forward Kinematics Transform", ht)
    
#     p_j0 = np.dot(np.eye(4), np.transpose([0, 0, 0, 1]))
#     p_j1 = np.dot(ht[0],np.transpose([0, 0, 0, 1]))
#     # print(p_j1)
#     p_j2 = np.dot(ht[1], np.transpose([0, 0, 0, 1]))
#     # print(p_j2)
#     p_j3 = np.dot(ht[2], np.transpose([0, 0, 0, 1]))
#     p_j4 = np.dot(ht[3], np.transpose([0, 0, 0, 1]))
#     p_j5 = np.dot(ht[4], np.transpose([0, 0, 0, 1]))
#     p_j6 = np.dot(ht[5], np.transpose([0, 0, 0, 1]))
#     p_j7 = np.dot(ht[6], np.transpose([0, 0, 0, 1]))

#     world_coords = [p_j0, p_j1, p_j2, p_j3, p_j4, p_j5, p_j6, p_j7]


def main():
    global image_pix
    # Initialize the node
    rospy.init_node('image_pix_gen')
    print("is main getting called")
    # joint_sub = rospy.Subscriber("/joint_states", JointState, world_coords, queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
            world_coords_tf()
            rate.sleep()

    rospy.spin()



if __name__ == "__main__":
    main()
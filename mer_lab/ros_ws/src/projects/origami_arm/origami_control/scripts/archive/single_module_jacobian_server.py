#!/usr/bin/env python3

# Computes and returns the current value of Jacobian, given the cable lengths
# Output: 1x9 vector with Jacobian elements to be shaped as 3x3

from __future__ import print_function
lambda_jacobian = None
import rospy
from std_msgs.msg import Float64MultiArray
from origami_control.srv import SingleModuleJacobian, SingleModuleJacobianResponse
# from vs_control.srv import SingleModuleJacobian, SingleModuleJacobianResponse
import dill
import numpy as np
from os.path import expanduser

home = expanduser("~")

# print(os.environ.get("ROS_PACKAGE_PATH "))
# jacobian_binary_path="~/mer_lab/ros_ws/src/projects/origami_arm/origami_control/scripts/single_module_Jacobian"

#* translation
# marker's center position and tilt angle
# shift is the translation from target to marker center
def marker_translation(marker_y, marker_z, marker_theta, shift_y, shift_z):
    d = np.sqrt(shift_y**2 + shift_z**2)
    home_theta =  np.arctan2(shift_y, shift_z)
    target_y =  marker_y - d * np.sin(home_theta + marker_theta)
    target_z =  marker_z - d * np.cos(home_theta + marker_theta)

    return target_y, target_z
   
#* simple ik
def simple_ik_2d( y, z):
    if y > 0.5:
        phi =  np.pi/2
    elif y < -0.5:
        phi =  -np.pi/2
    else:
        phi =  0
        
    x = 0
    l = np.sqrt(x**2 + y**2);
    length = np.sqrt(l**2 + z**2);
    
    l2 = length * 0.5;
    cos_alpha = l / length;

    r = l2 / cos_alpha;

    kappa = 1/r;

    if kappa < 0.00001:
        s = length;
    else:
        s = r * np.arcsin(z/r);

    # print("z:", z)
    # print("r:", r)
    # print("l:", l)
    # print("cos_alpha:", cos_alpha)

    # print("s:", s)
    # print("kappa:", kappa)
    # print("phi:", phi)
    return s, kappa, phi

def arclengthKappaPhi_to_CableLength(arclength, kappa, phi, d):
    l1 = arclength * (1 - kappa*d*np.sin(phi));
    l2 = arclength * (1 + kappa*d*np.sin(np.pi/3 + phi));
    l3 = arclength * (1 - kappa*d*np.cos(np.pi/6 + phi));

    return l1, l2, l3, d

#* bad way to avoid singularity
def avoid_singularity(l2, l3):
    if abs(l2 - l3) < 0.1:
        return l2 + 0.2
    else:
        return l2
#* main handle
def handle_single_module_jacobian(req):
    # global jacobian_binary_path

    global marker_ee_y, marker_ee_z, marker_ee_theta, marker_base_y, marker_base_z, marker_base_theta
    global shift_ee_y, shift_ee_z, shift_base_y, shift_base_z, d
# #** 
#     shift_ee_y = -25
#     shift_ee_z =  25
#     shift_base_y = 0
#     shift_base_z = -54

#     marker_ee_y = 0
#     marker_ee_z = 180
#     marker_ee_theta = 0

#     marker_base_y = 0
#     marker_base_z = 2
#     marker_base_theta = 0

#     d = 40

#** 
    [ee_y, ee_z] =  marker_translation(marker_ee_y, marker_ee_z, marker_ee_theta, shift_ee_y, shift_ee_z)

    [base_y, base_z] =  marker_translation(marker_base_y, marker_base_z, marker_base_theta, shift_base_y, shift_base_z)
    # print("ee_y:", ee_y)
    # print("ee_z:", ee_z)
    # print("base_y:", base_y)
    # print("base_z:", base_z)
    y =  ee_y - base_y
    z =  ee_z - base_z

    [s, k, p] = simple_ik_2d(y, z)

    [l1, l2, l3, d] =  arclengthKappaPhi_to_CableLength(s, k, p, d)

    # lambda_jacobian =  dill.load(open(jacobian_binary_path, "rb"))
    # print("Single module cable length --> Jacobian")
    l2_s =  avoid_singularity(l2, l3)
    jacobian_matrix = lambda_jacobian(l1, l2_s, l3, d)

    array =  [jacobian_matrix[0][0], jacobian_matrix[0][1], jacobian_matrix[0][2],\
              jacobian_matrix[1][0], jacobian_matrix[1][1], jacobian_matrix[1][2], \
              jacobian_matrix[2][0], jacobian_matrix[2][1], jacobian_matrix[2][2]]
    
    return SingleModuleJacobianResponse(step=3, jv=array)

#* 
marker_ee_y = None
marker_ee_z = None
marker_ee_theta = None 
marker_base_y = None
marker_base_z = None 
marker_base_theta = None

#* ros
def pose_callback(data):
    global marker_ee_y, marker_ee_z, marker_ee_theta, marker_base_y, marker_base_z, marker_base_theta, ratio_mm_pixel 

    marker_ee_y = data.data[0] * ratio_mm_pixel
    marker_ee_z = data.data[1] * ratio_mm_pixel

    marker_base_y = data.data[2] * ratio_mm_pixel
    marker_base_z = data.data[3] * ratio_mm_pixel

    marker_ee_theta =  data.data[4]
    marker_base_theta =  data.data[5]

    # print(f"ee: {marker_ee_y}, {marker_ee_z}, {marker_ee_theta}")
    # print(f"base: {marker_base_y}, {marker_base_z}, {marker_base_theta}")


#* service server
def single_module_jacobian_server():
    global lambda_jacobian
    rospy.init_node('single_module_jacobian_server')

    rospy.Subscriber("origami_vs/aruco/pose", Float64MultiArray, pose_callback)

    param_name = rospy.search_param('trans_ee_marker_y')
    # => None
    v = rospy.get_param(param_name)
    # =>
    # {'run_id': '5fb7f412-c015-11ec-b53e-ab7c3eae037a', 'roslaunch': {'uris': {'host_hal_in_forw_rd__34837': 'http://hal-in-forw-rd:34837/'}}, 'rosversion': '1.15.14
    # ', 'rosdistro': 'noetic
    # '}

    global shift_ee_y, shift_ee_z, shift_base_y, shift_base_z, d, ratio_mm_pixel
    # marker_ee_y = req.marker_ee.x
    # marker_ee_z = req.marker_ee.y
    # marker_ee_theta = req.marker_ee.theta
    shift_ee_y = rospy.get_param("/origami_vs/trans_ee_marker_y")
    shift_ee_z = rospy.get_param("/origami_vs/trans_ee_marker_z")

    # marker_base_y = req.marker_base.x
    # marker_base_z = req.marker_base.y
    # marker_base_theta = req.marker_base.theta
    shift_base_y = rospy.get_param("/origami_vs/trans_base_marker_y")
    shift_base_z =  rospy.get_param("/origami_vs/trans_base_marker_z")

    d = rospy.get_param("/origami_vs/origami_module_d")

    ratio_mm_pixel = rospy.get_param("/origami_vs/ratio_mm_pixel")


#** 

    jacobian_binary_path=home+"/mer_lab/ros_ws/src/projects/origami_arm/origami_control/scripts/single_module_Jacobian"
    lambda_jacobian =  dill.load(open(jacobian_binary_path, "rb"))
    
    if not marker_ee_y:
        s =  rospy.Service('single_module_jacobian', SingleModuleJacobian, handle_single_module_jacobian)
  
    # s =  rospy.Service('single_module_jacobian', SingleModuleJacobian, handle_single_module_jacobian)
    # rospy.Subscriber("origami_vs/aruco/pose", Float64MultiArray, pose_callback)

    print("Ready to calculate Jacobian for single module.")
    rospy.spin()

if __name__ == "__main__":
    single_module_jacobian_server()

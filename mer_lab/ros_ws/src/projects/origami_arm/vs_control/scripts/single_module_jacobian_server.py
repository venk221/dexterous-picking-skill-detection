#!/usr/bin/env python3


# Computes and returns the current value of Jacobian, given the cable lengths
# Output: 1x9 vector with Jacobian elements to be shaped as 3x3

from __future__ import print_function
from os import path
import rospy
#from std_msgs.msg import Float64MultiArray
from vs_control.srv import SingleModuleJacobian, SingleModuleJacobianResponse
# from vs_control.srv import SingleModuleJacobian, SingleModuleJacobianResponse
import dill
HOME = path.expanduser('~')
print(HOME)
# print(os.environ.get("ROS_PACKAGE_PATH "))
jacobian_binary_path=HOME+"/mer_lab/ros_ws/src/projects/origami_arm/vs_control/scripts/single_module_Jacobian"

def avoid_singularity(l2, l3):
    if abs(l2 - l3) < 0.1:
        return l2 + 0.2
    else:
        return l2

def handle_single_module_jacobian(req):
    global jacobian_binary_path
    lambda_jacobian =  dill.load(open(jacobian_binary_path, "rb"))
    # print("Single module cable length --> Jacobian")
    l2 =  avoid_singularity(req.l2,  req.l3)
    jacobian_matrix = lambda_jacobian(req.l1, l2, req.l3, req.d)
    # print(jacobian_matrix)

    array =  [jacobian_matrix[0][0], jacobian_matrix[0][1], jacobian_matrix[0][2],\
              jacobian_matrix[1][0], jacobian_matrix[1][1], jacobian_matrix[1][2], \
              jacobian_matrix[2][0], jacobian_matrix[2][1], jacobian_matrix[2][2]]
   
    return SingleModuleJacobianResponse(step=3, jv=array)


def single_module_jacobian_server():
    rospy.init_node('single_module_jacobian_server')

    s =  rospy.Service('single_module_jacobian', SingleModuleJacobian, handle_single_module_jacobian)

    print("Ready to calculate Jacobian for single module.")
    rospy.spin()

if __name__ == "__main__":
    single_module_jacobian_server()

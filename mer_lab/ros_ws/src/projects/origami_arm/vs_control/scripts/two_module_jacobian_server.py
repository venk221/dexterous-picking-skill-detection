#!/usr/bin/env python3


# Computes and returns the current value of Jacobian, given the cable lengths
# Output: 1x9 vector with Jacobian elements to be shaped as 3x3

from __future__ import print_function
from os import path
import rospy
from std_msgs.msg import Float64MultiArray
from vs_control.srv import TwoModuleJacobian, TwoModuleJacobianResponse
import dill

HOME = path.expanduser('~')
print(HOME)

# jacobian_binary_path="~/mer_lab/ros_ws/src/krishna_dr/soft_robot_vs/scripts/origami_2_module_Jacobian"
jacobian_binary_path=HOME+"/mer_lab/ros_ws/src/projects/origami_arm/vs_control/scripts/origami_2_module_Jacobian"
def avoid_singularity(l2, l3):
    if abs(l2 - l3) < 0.1:
        return l2 + 0.2
    else:
        return l2

def handle_two_module_jacobian(req):
    global jacobian_binary_path
    lambda_jacobian =  dill.load(open(jacobian_binary_path, "rb"))
    print("Two module cable length --> Jacobian")
    l2 =  avoid_singularity(req.l2,  req.l3)
    l5 =  avoid_singularity(req.l5,  req.l6)
    jacobian_matrix = lambda_jacobian(req.l1, l2, req.l3,req.l4, l5, req.l6, req.d)
    print("jac",jacobian_matrix)

    array =  [jacobian_matrix[0][0], jacobian_matrix[0][1], jacobian_matrix[0][2],jacobian_matrix[0][3], jacobian_matrix[0][4], jacobian_matrix[0][5],\
              jacobian_matrix[1][0], jacobian_matrix[1][1], jacobian_matrix[1][2],jacobian_matrix[1][3], jacobian_matrix[1][4], jacobian_matrix[1][5],\
              jacobian_matrix[2][0], jacobian_matrix[2][1], jacobian_matrix[2][2],jacobian_matrix[2][3], jacobian_matrix[2][4], jacobian_matrix[2][5]]
    
    return TwoModuleJacobianResponse(step=6, jv=array)


def two_module_jacobian_server():
    rospy.init_node('two_module_jacobian_server')

    s =  rospy.Service('two_module_jacobian', TwoModuleJacobian, handle_two_module_jacobian)

    print("Ready to calculate Jacobian for Two module.")
    rospy.spin()

if __name__ == "__main__":
    two_module_jacobian_server()

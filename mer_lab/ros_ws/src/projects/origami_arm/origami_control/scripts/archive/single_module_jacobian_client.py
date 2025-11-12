#!/usr/bin/env python3

#* jacobian service client
# Send virtual pose information to test the jacobian service.

from __future__ import print_function

import sys
import rospy
from std_msgs.msg import Float64MultiArray
from origami_control.srv import *
import numpy as np


def single_module_jacobian_client():
    rospy.wait_for_service('single_module_jacobian')
    try:
        single_module_jacobian =  rospy.ServiceProxy('single_module_jacobian', SingleModuleJacobian)
        print("got service")
        resp1 = single_module_jacobian()

        print(resp1.jv)
        return resp1.jv
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        
def usage():
    return "%s "%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 1:
        pass
    else:
        print(usage())
        sys.exit(1)
    
    print("Requesting Jacobian (velocity)")
    jv_array = single_module_jacobian_client()

    jv = np.reshape(jv_array, (3, 3))
    dx = 0
    dy = 10
    dz = 0
    inv_jac = np.linalg.inv(jv)
    print(np.matmul(inv_jac, np.array([[dx], [dy], [dz]])))

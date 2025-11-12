#!/usr/bin/env python3
from __future__ import print_function

import sys
import rospy
#from std_msgs.msg import Float64MultiArray
from orca_misc.srv import *

def single_module_jacobian_client(l1, l2, l3, d):
    rospy.wait_for_service('single_module_jacobian')
    try:
        single_module_jacobian =  rospy.ServiceProxy('single_module_jacobian', SingleModuleJacobian)
        print("got service")
        resp1 = single_module_jacobian(l1, l2, l3, d)

        return resp1.jv
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        
def usage():
    return "%s [l1 l2 l3 d]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 5:
        l1 = float(sys.argv[1])
        l2 = float(sys.argv[2])
        l3 = float(sys.argv[3])
        d = float(sys.argv[4])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting Jacobian for (%s, %s, %s, %s)"%(l1, l2, l3, d))
    print(single_module_jacobian_client(l1, l2, l3, d))

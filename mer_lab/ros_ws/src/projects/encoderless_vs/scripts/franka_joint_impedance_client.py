#!/usr/bin/env python3
# license removed for brevity
import sys
import rospy
from controller_manager_msgs.srv import SwitchController, LoadController
from franka_msgs.srv import SetJointImpedance
from encoderless_vs.srv import vel_start, vel_startResponse
from std_msgs.msg import Bool

def main():
    # Initialize the node
    rospy.init_node('joint_impedance_node_start')    
    print("velocity service is called")
    rospy.sleep(1)
    # wait for velocity service to be up
    rospy.wait_for_service('velocity_start_service')  

    # create a handle to call the service
    velocity_start_service = rospy.ServiceProxy('velocity_start_service', vel_start)
    print("Velocity Service Started")
    srv_resp = velocity_start_service(True)
    print(srv_resp.output)
    

if __name__ == '__main__':
    main()
    

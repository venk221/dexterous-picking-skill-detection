#!/usr/bin/env python3
# license removed for brevity
import sys
import rospy
from controller_manager_msgs.srv import SwitchController, LoadController
from franka_msgs.srv import SetJointImpedance
from encoderless_vs.srv import vel_start, vel_startResponse
from std_msgs.msg import Bool

def velocity_start_service(msg):
    
    return vel_startResponse(True)

def main():
    # Initialize the node
    global res_vel
    rospy.init_node('joint_impedance_node')    
    rospy.sleep(1)
    # wait for the impedance service to be up
    rospy.wait_for_service('/franka_control/set_joint_impedance')
    # request the service to send joint_impedance
    impedance = rospy.ServiceProxy('/franka_control/set_joint_impedance', SetJointImpedance)
    joint_stiffness = [2000.0, 1000.0, 1000.0, 2000.0, 2000.0, 2000.0, 2000.0]
    res_imp = impedance(joint_stiffness) 
    print("set_joint_impedance", res_imp)
    rospy.sleep(1)
    # wait for the switchcontroller service to be up
    rospy.wait_for_service('/controller_manager/switch_controller')
    # start the velocity controllers 
    sc_service = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
    start_controllers = ['joint_group_velocity_controller']
    stop_controllers = []
    strictness = 2
    start_asap = False
    timeout = 0.0
    res_vel = sc_service(start_controllers,stop_controllers, strictness, start_asap,timeout)
    print("Started Velcoity Controller", res_vel)

    # service declaration to start the velocity controller
    vel_start_serv = rospy.Service("velocity_start_service", vel_start, velocity_start_service)


    rospy.spin()


if __name__ == '__main__':
    main()
    

#!/usr/bin/env python3
# license removed for brevity
import math
import random
import rospy
import numpy as np
from controller_manager_msgs.srv import SwitchController
from std_msgs.msg import Int64, Float64MultiArray, Float64
from sensor_msgs.msg import JointState
from numpy.linalg import multi_dot
from geometry_msgs.msg import Twist
#from rrbot_velocity_control.srv import ctrl_srv_command, ctrl_srv_commandResponse

r01 = np.matrix([[0, 0, 0], [0, 0, 0], [0, 0, 0]])
r12 = np.matrix([[0, 0, 0], [0, 0, 0], [0, 0, 0]])
r23 = np.matrix([[0, 0, 0], [0, 0, 0], [0, 0, 0]])
xdotee=0
ydotee=0
linvel0 = np.array([[0], [0], [0]]) 

pub_q1_vel = rospy.Publisher('/vsbot/joint1_velocity_controller/command', Float64, queue_size=1)
pub_q2_vel = rospy.Publisher('/vsbot/joint2_velocity_controller/command', Float64, queue_size=1)   

#pub_q1_pose = rospy.Publisher('/rrbot/joint1_position_controller/command', Float64, queue_size=1)
#pub_q2_pose = rospy.Publisher('/rrbot/joint2_position_controller/command', Float64, queue_size=1)   
 
#node_status = 0

def callback_vk(dt):
    global r01, r12, r23, xdotee, ydotee
    #if (node_status == 2):
    #print("Inside the calculation: ")
    q1 = dt.position[0]
    q2 = dt.position[1]
    c1 = np.cos(q1)
    s1 = np.sin(q1)
    c2 = np.cos(q2)
    s2 = np.sin(q2)
    c12 = np.cos(q1+q2)
    s12 = np.sin(q1+q2) 
    T01 = np.matrix([[c1, -s1, 0, 0.5*c1], [s1, c1, 0, 0.5*s1], [0, 0, 1, 0], [0, 0, 0, 1]])
    T12 = np.matrix([[c2, -s2, 0, 0.6*c2], [s2, c2, 0, 0.6*s2], [0, 0, 1, 0], [0, 0, 0, 1]]) 
    r01 = T01[0:3, 0:3]
    r12 = T12[0:3, 0:3]
    r23 = np.matrix([[-1, 0, 0], [0, -1, 0], [0, 0, 0]]) #[[0, 1, 0], [-1, 0, 0], [0, 0, 0]]
    Jp = np.matrix([[-0.5*s1 - 0.6*s12, -0.6*s12], [0.5*c1 + 0.5*c12, 0.6*c12]]) #Position Jacobian derived from matlab
    Jpinv = np.linalg.inv(Jp)
    xdot = np.float64(linvel0[0][0])
    ydot = np.float64(linvel0[1][0])
    lin_vel = np.array([[xdot],[ydot]])
    #print("Final End Effector Velocity", lin_vel)
    Theta_dot = np.dot(Jpinv, lin_vel)
    q1_dot = np.float64(Theta_dot[0][0])
    q2_dot = np.float64(Theta_dot[1][0])
    print("q1_dot: ", q1_dot)
    print("q2_dot: ", q2_dot)
    pub_q1_vel.publish(q1_dot)
    pub_q2_vel.publish(q2_dot)
        
def callback_vsbot_center(data):
    global linvel0
    #if (node_status == 2):  
    #xdotorig = data.data[0]
    #ydotorig = data.data[1]   
    xdotee = data.data[0]/350
    ydotee = data.data[1]/350
    linvelimg = np.array([[xdotee],[ydotee],[0]]) # velocity in image frame
    linvel2 = np.dot(r23, linvelimg)  # end effector velocity with respect to camera frame/frame2 
    linvel1 = np.dot(r12, linvel2) # end effector velocity with respect to frame 1
    linvel0 = np.dot(r01, linvel1) # end effector velocity with respect to frame 0	       
    #print("Inside second callback")
    #print("end effector velocity original: ", xdotorig, ydotorig)
    #print("end effector velocity with respect to image frame: ", linvelimg)
    #print("end effector velocity with respect to camera frame or frame2: ", linvel2)
    #print("end effector velocity with respect to frame 1: ", linvel1)
    #print("end effector velocity with respect to frame 0: ", linvel0)
    
            
if __name__ == "__main__":
    rospy.init_node('Controller_Node', anonymous=True)     
    sub_vel = rospy.Subscriber('/vsbot/joint_states', JointState, callback_vk)
    sub_center = rospy.Subscriber('publishError', Float64MultiArray, callback_vsbot_center)
    #srv_control_command = rospy.Service('control_pub_command', ctrl_srv_command, callback_command)  
    rospy.spin()
        
    
     


#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool

servo = False
rate = 0
module_num = 0
vel = []
motor_conversion = 0.1

def getServoFlag(msg):
    global servo
    servo = msg.data

def getVelocity(msg):
    global vel
    vel.clear()
    for i in range(len(msg.data)):
        vel.append(msg.data[i])

def main():
    global rate, module_num, motor_conversion

    # ROS node init
    rospy.init_node("simulation_controllers")

    # Read sim parameters
    module_num = rospy.get_param("origami_sim/num_of_modules")
    rate = rospy.get_param("origami_sim/control_rate")
    motor_conversion = rospy.get_param("origami_sim/motor_conversion")

    # Subscribers
    vel_sub = rospy.Subscriber("origami_vs/vel", Float64MultiArray, getVelocity, queue_size=1)
    start_servo_sub = rospy.Subscriber("origami_vs/start_servo", Bool, getServoFlag, queue_size=1)

    # Publisher
    cable_length_pub = rospy.Publisher("origami_vs/d_cable_length", Float64MultiArray,queue_size=1)
    
    d_cable_length = []
    r = rospy.Rate(rate)
    
    # Wait for servoing to start
    while not servo:
        r.sleep()

    while not rospy.is_shutdown():
        # Convert velocities to change in cable lengths
        # Assuming 10:1 ratio for motor rpm and cable length vel (mm/s)
        # vel=[-1,-1,-1,-1,-1,-1]
        # vel=[10,-10,-10]

        d_cable_length.clear()
        if len(vel) == 0:
            d_cable_length = [0]*3*module_num
        else:
            for i in range(len(vel)):
                d_cable_length.append(vel[i]*motor_conversion/rate)
        
        # Publish change in cable lengths
        d_cable_length_msg = Float64MultiArray()
        d_cable_length_msg.data = d_cable_length
        cable_length_pub.publish(d_cable_length_msg)
        r.sleep()
    
    rospy.spin()


if __name__ == "__main__":
    main()
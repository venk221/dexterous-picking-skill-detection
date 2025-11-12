#!/usr/bin/env python3

#################### Control Modes #######################

# 3 Initialize
# 0 "Absolute" Position control
# 2 Velocity control
# 1 Incremental position control

# After initializing the module is switched to absolute position control mode
# The position object is reset to all 0's
# when initialized, the module is about 85 mm long
# Module's total length is about 250 mm
# Safe extension for module is about 120 mm

# import hid
import time
# from importlib.metadata import version
import math
# import PID
import rospy
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
import numpy as np

T0 = np.array([[-1, 0, 0, 0],
               [0, 1, 0, 0],
               [0, 0, -1, 0],
               [0, 0, 0, 1]])



u =[0]*32                       # received velocity data
s =[0]*32                       # received position data
vel = [0]*32                    # recorded velocity data from encoder
pos = [0]*32                    # recorded position data from encoder
mode = 0                        # received control mode data
prev_pos = [0]*32               # recorded previous position data
output_vel = Int32()            # measured output velocity
desired_out_vel = Int32()       # desired output velocity
output_pos = Int32()            # measured output pos
desired_output_pos = Int32()    # desired output pos
pos_pid = None                  # object for position control PID params
vel_pid = None                  # object for velocity control PID params

gear_ratio = 150
encoder_magnet_poles = 7
quad_count_mode_multiplier = 4

shaft_diameter = 7.0            # unit: mm
max_module_length = 180.0       # unit: mm

cablelen_to_cnt = (gear_ratio * encoder_magnet_poles * quad_count_mode_multiplier) / (shaft_diameter * math.pi)

def write_vel_sim(desired_velocity):
    global vel, prev_pos, pos, output_vel, desired_out_vel

    for k in range(32):
        diff_pos = pos[k] - prev_pos[k]
        if diff_pos > 128:
            diff_pos -= 256
        if diff_pos < -128:
            diff_pos += 256
        # vel[k] =  vel[k]*0.99 + max(-16, min(16, diff_pos))  # averaging filter, not needed
        vel[k] = diff_pos
    prev_pos = pos

    for k in range(32):
        pos[k] = pos[k] + desired_velocity[k] 

    output_vel.data = vel[9]                        # recording for plotting
    desired_out_vel.data = desired_velocity[9]      # recording for plotting

def FK_2_modules(pos, T0):
    # module id = 2
    m0_cab_len = []
    for i in range(9, 12):
        m0_cab_len.append(pos[i]/cablelen_to_cnt)

    m0_skp = cablelen_to_skp(*m0_cab_len, 40)
    T01 = continuum_fk_arc(*m0_skp)

    T1 = T0 @ T01

    # module id = 3
    m1_cab_len = []
    for i in range(13, 16):
        m1_cab_len.append(pos[i]/cablelen_to_cnt)

    m1_skp = cablelen_to_skp(*m1_cab_len, 40)
    T12 = continuum_fk_arc(*m1_skp)

    T2 = T1 @ T12

    return T2
    

def continuum_fk_arc(s, kappa, phi, ksi=1):
    """
    Get Transformation Matrix by given (s, kappa, phi)
    """
    theta = ksi*s*kappa
    phi = np.take(phi, 0)
    R11 = np.cos(phi)*np.cos(phi)*np.cos(theta) + np.sin(phi)*np.sin(phi)
    R12 = np.cos(phi)*np.sin(phi)*(np.cos(theta)-1)
    R13 = np.cos(phi)*np.sin(theta)
    R21 = R12
    R22 = np.sin(phi)*np.sin(phi)*np.cos(theta) + np.cos(phi)*np.cos(phi)
    R23 = np.sin(phi)*np.sin(theta)
    R31 = -R13
    R32 = -R23
    R33 = np.cos(theta)
    PX = np.cos(phi)*(1-np.cos(theta))/kappa
    PY = np.sin(phi)*(1-np.cos(theta))/kappa
    PZ = np.sin(theta)/kappa

    T = np.array([[R11, R12, R13, PX],
                  [R21, R22, R23, PY],
                  [R31, R32, R33, PZ],
                  [0, 0, 0, 1]], dtype=np.float32)

    return T

def cablelen_to_skp(l1, l2, l3, d):
    """Convert cable length to s, kappa, phi"""
    s = (l1 + l2 + l3)/3
    kappa = 2* np.sqrt(l1**2 + l2**2 + l3**2 - l1*l2 - l2*l3 - l3*l1)/(d * (l1 + l2 + l3))

    if abs(l2 - l3) < 0.01:
        if l1 <= l2:
            phi = np.pi/2
        else:
            phi = np.pi*3/2

    else:
        phi = np.arctan2(l3 + l2 - 2*l1, np.sqrt(3) * (l2 - l3))

    return s, kappa, phi


def write_vel(h, desired_velocity, vel_pid):
    global vel, prev_pos, output_vel, desired_out_vel

    inData = list(h.read(64))
    pwm = [0]*32        # pwm cmd to send to motor driver

    for k in range(32):
        diff_pos = inData[k*2] - prev_pos[k]
        if diff_pos > 128:
            diff_pos -= 256
        if diff_pos < -128:
            diff_pos += 256
        # vel[k] =  vel[k]*0.99 + max(-16, min(16, diff_pos))  # averaging filter, not needed
        vel[k] = diff_pos
    prev_pos = inData[::2]
    
    for k in range(32):
        vel_pid[k].SetPoint = desired_velocity[k]
        vel_pid[k].update(vel[k])

        targetPWM= vel_pid[k].output * -1
        pwm[k] = max(min(int(targetPWM), 2047), -2047)

    h.write(bytes([0] + motorBytes(pwm)))     
    
    output_vel.data = vel[9]                        # recording for plotting
    desired_out_vel.data = desired_velocity[9]      # recording for plotting
    
def write_pose(h,desired_position, pos_pid):
    global pos, prev_pos

    inData = list(h.read(64))
    pwm = [0]*32

    for k in range(32):
        diff_pos = inData[k*2] - prev_pos[k]
        if diff_pos > 128:
            diff_pos -= 256
        if diff_pos < -128:
            diff_pos += 256
        # pos[k] += max(-16, min(16, diff_pos))
        pos[k] += diff_pos
        
    prev_pos = inData[::2]

    for k in range(32):
        pos_pid[k].SetPoint = desired_position[k]
        pos_pid[k].update(pos[k])
        targetPWM= pos_pid[k].output * -1
        pwm[k] = max(min(int(targetPWM), 2047), -2047)
    
    h.write(bytes([0] + motorBytes(pwm)))

    # record desired and current position
    output_pos.data = int(pos[9])
    desired_output_pos.data = int(desired_position[9])

def init(h):
    global pos

    pwm = [2047]*32
    # Send PWM to shrink module to min length
    h.write(bytes([0] + motorBytes(pwm)))
    print("waiting for module to init")
    rospy.sleep(10)
    print("module at init pose")
    
    # reset absolute position 
    pos = [0]*32

def motorBytes(x):
    mb = [0] * (2 * len(x))
    for i, xi in enumerate(x):
        xi = max(-2047, min(2048, round(xi)))
        mb[2*i] = (xi >> 8) & 0x0F
        mb[2*i + 1] = xi & 0xFF
    return mb

def motorMask(msg, motorEnableMask):
    u = [0] * 32
    for k in range(32):
        u[k] = motorEnableMask[k] * msg.data[k]    
    return u

def control_mode_cb(msg,args):
    global  mode
    mode = msg.data
    # h = args[0]

    if mode == 0:
        print("MODE: position control")
    elif mode == 1:
        print("MODE: incremental position control")
    elif mode == 2:
        print("MODE: velocity control")
    elif mode == 3:
        print("Init: pos = 80 mm")
        pos = [80*cablelen_to_cnt]*32
    
def control_cb(msg, args):
    # h = args[0]
    motorEnableMask = args[0]

    # Controller input for velocity control
    global u, s
    u = motorMask(msg, motorEnableMask)
    
    # Convert to controller input for position control
    for k in range(len(u)):
        s[k] = u[k] * cablelen_to_cnt
    
    if mode == 1:
        for k in range(len(u)):
            s[k] = s[k] + pos[k]

def main():
    # vid = 0xFFEF
    # proid = 0x0004

    # print(f'\nHID Module Version: {version("hidapi")}')


    motorEnable = [0, 1, 1, 1] * 4  + [0] * 4 * 4
    global prev_pos, mode, vel_pid, pos_pid

    rospy.init_node('ommd_listener', anonymous=True)

    try:
        # h = hid.device()
        # h.open(vid, proid)
        # print(f'\nUSB Device Manufacturer: {h.get_manufacturer_string()}\nUSB Device Product Name: {h.get_product_string()}\n')

        # # Write 0s read encoder info
        # h.write(bytes(65))
        # h.read(64)
        
        # Declare subscribers
        control_select = rospy.Subscriber("origami_vs/OMMD_control_mode", Int32, control_mode_cb,(motorEnable), queue_size=1)
        control_input = rospy.Subscriber("origami_vs/OMMD_control_input", Int32MultiArray, control_cb, (motorEnable), queue_size=1)
        
        # Declare publishers
        vel_plot_pub = rospy.Publisher("origami_vs/vel_plotter", Int32,queue_size=1 )
        vel_dplot_pub = rospy.Publisher("origami_vs/vel_dplotter", Int32,queue_size=1 )
        pos_plot_pub = rospy.Publisher("origami_vs/pos_plotter", Int32, queue_size=1)
        pos_dplot_pub = rospy.Publisher("origami_vs/pos_dplotter", Int32, queue_size=1)

        # # PID objects for controllers
        # vel_pid = [PID.PID(50, 0, 0.01) for k in range(32)]
        # pos_pid = [PID.PID(10, 1, 0.3) for k in range(32)] 
        
        # # Setting velocity PID params for all channels
        # for k in range(32):
        #     vel_pid[k].SetPoint = 0
        #     vel_pid[k].setSampleTime(0.01) # Update every 0.01 seconds

        #     pos_pid[k].SetPoint = 0
        #     pos_pid[k].setSampleTime(0.01) # Update every 0.01 seconds
        #     pos_pid[k].setWindup(16)
        
        # # Read encoder values
        # inData = list(h.read(64))
        # prev_pos = inData[::2]

        # Set simulation init state
        pos = [80*cablelen_to_cnt]*32
        prev_pos = pos
        
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            
            # position control
            if mode == 0:
                # Check each tendon, if des_pos > max, des_pos = max
                # write_pose(h, s, pos_pid)
                pass
            
            # incremental position control
            if mode == 1:
                # Same as velocity
                # write_pose(h, s, pos_pid)
                pass

            # velocity control
            if mode == 2:
                # Check each tendon, if cur_pos > max && velocity -> extend
                # Make velocity 0 for that tendon
                # write_vel(h, u, vel_pid)
                write_vel_sim(u)

            # Data vis 
            vel_plot_pub.publish(output_vel)
            vel_dplot_pub.publish(desired_out_vel)
            pos_plot_pub.publish(output_pos)
            pos_dplot_pub.publish(desired_output_pos)
            
            r.sleep()

        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("done!")
    
    except IOError as ex:
        print(ex)
        print("You probably don't have the hard-coded device.")
        print("Update the h.open() line in this script with the one")
        print("from the enumeration list output above and try again.")

if __name__ == "__main__":
    main()

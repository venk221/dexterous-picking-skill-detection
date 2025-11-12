#!/usr/bin/env python3
import dill
import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

ee_pose_list = []
rotation_y = np.array([[-1,0,0],[0,1,0],[0,0,-1]])
# Read symbolic Jacobian
lambda_jacobian = dill.load(open("single_module_Jacobian", "rb"))

# test = np.array(lambda_jacobian(50, 51, 49, 40))
# Test if symbolic jacobian is read and can be substituted
# print(test)

def jacobian_test(init_cable,poselist,init_pose,ref_poselist,cable_list):
    vel_vec = np.array([0.1,0,0]) # Vel [x,y,z]
    ref_vel_vec = vel_vec
    # Rotation_Y_180 : (Robot Frame to World Frame)
    vel_vec = rotation_y @ vel_vec

    cable_k = init_cable
    ref_k = init_pose

    for i in range(1000):
        # avoid singularity
        if abs(cable_k[1]-cable_k[2]) < 0.2:
            cable_k[1] = cable_k[1] + 2
        else:
            cable_k[1]

        # Find current jacobian w.r.t current cable lengths
        jac = jacobian_update(cable_k[0],cable_k[1],cable_k[2])
        jac_inv = np.linalg.inv(jac)
        # Calculate Cable velocity  = (J^-1 * ee_pose_velocity) 
        delta_cable = jac_inv @ vel_vec
        # Find new ee pose using Cable velocity
        new_cable = cable_k + delta_cable
        cable_k = new_cable  #update cable length for next step
        ee_pose_new = fk(new_cable)  # New EE Pose
        poselist.append(ee_pose_new)

        # Reference EE pose 
        ref_pose_new = ref_k + ref_vel_vec
        ref_k = ref_pose_new
        ref_poselist.append(ref_pose_new)
        
        
        # Cable list to plot
        cable_list.append(cable_k)
        print("cable lens", cable_k)

    plot_fun(poselist,ref_poselist)
    plot_cable_lens(cable_list)

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


    if kappa != 0:
        PX = np.cos(phi)*(1-np.cos(theta))/kappa
        PY = np.sin(phi)*(1-np.cos(theta))/kappa
        PZ = np.sin(theta)/kappa
    else:
        PX = 0
        PY = 0
        PZ = ksi*s

    T = np.array([[R11, R12, R13, PX],
    [R21, R22, R23, PY],
    [R31, R32, R33, PZ],
    [0, 0, 0, 1]], dtype=np.float32)

    return T


def cablelen_to_skp(l1, l2, l3, d=40):
 """Convert cable length to s, kappa, phi"""
 s = (l1 + l2 + l3)/3
 kappa = 2 * np.sqrt(l1**2 + l2**2 + l3**2 - l1*l2 - l2*l3 - l3*l1)/(d * (l1 + l2 + l3))

 if abs(l2 - l3) < 0.01:
     if l1 <= l2:
         phi = np.pi/2
     else:
         phi = np.pi*3/2

 else:
     phi = np.arctan2(l3 + l2 - 2*l1, np.sqrt(3) * (l2 - l3))

 return s, kappa, phi

def fk(c_len):
    # set init cable lengths and parameters for the module
    s,k,p = cablelen_to_skp(c_len[0], c_len[1], c_len[2])
    T = continuum_fk_arc(s,k,p)
    ee_pose = T[0:3, 3]
    # print("ee pose ",ee_pose)
    return ee_pose


# 3D plot function
def plot_fun(ee_ps,ref_ps):
    ee_ps = np.array(ee_ps)
    ref_ps = np.array(ref_ps)
    fig = plt.figure()
    ax  = plt.axes(projection='3d')

    ax.scatter(ee_ps[:,0], ee_ps[:,1], ee_ps[:,2],c='g') # Plot EE POSE
    ax.scatter(ee_ps[-1,0], ee_ps[-1,1], ee_ps[-1,2],c='r') # last point in  to find direction
    ax.scatter(ref_ps[:,0], ref_ps[:,1], ref_ps[:,2],c='b') # Plot Reference EE_POSE
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')
    name = ''.join(str(x) for x in init_lens)
    ax.set_title('Jacobian Test-C_lens ' + name +' EE Velocity[0.1,0,0];1000 points')
    # Set the scale of the x, y, and z axes
    # ax.set_xlim([-3, 1])  # Set x-axis limits to -3 and 3
    # ax.set_ylim([-0.5, 0.5])  # Set y-axis limits to -3 and 3
    # ax.set_zlim([49, 50])  # Set z-axis limits to -3 and 3
    # ax.set_xlim([-40, 0])  # Set x-axis limits to -3 and 3
    # ax.set_ylim([21, 22])  # Set y-axis limits to -3 and 3
    # ax.set_zlim([123, 124])  # Set z-axis limits to -3 and 3
    plt.show()


def plot_cable_lens(cable_list):
    t = np.linspace(0,1000,len(cable_list))
    cable_list = np.array(cable_list)
    plt.plot(t, cable_list[:,0])
    plt.plot(t, cable_list[:,1])
    plt.plot(t, cable_list[:,2])
    plt.ylabel('Lengths in mm')
    plt.xlabel('No of points')
    plt.legend( [ 'l1'   # This is l1
                , 'l2'       # This is l2
                 ,'l3'
                ] )
    plt.show()

def jacobian_update(l1,l2,l3):
    jacob = np.array(lambda_jacobian(l1, l2, l3, 40))
    jacob = jacob[0:3,:] #slice 3*3 (vx,vy,vz) from 6*3
    return jacob


def plot_single(init_lens):
    poselist = []
    ref_poselist = []
    cable_list = []
    cable_list.append(init_lens)
    initpose = fk(init_lens)
    #append initial Pose
    poselist.append(initpose)
    # first_ref_pose = rotation_y @ initpose
    ref_poselist.append(initpose)

    #Call Jacobian test
    jacobian_test(init_lens,poselist,initpose,ref_poselist,cable_list)





#### Starts here
init_lens = np.array([120,120,160])
plot_single(init_lens)


# 120,120,160
# 100,100,190
# 120,120,280
# 80,79,81
#!/usr/bin/env python3

import sys
import rospy
import cv2
import numpy as np
from skimage.morphology import skeletonize
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from scipy.interpolate import splprep, splev
import time
from cv_bridge import CvBridge, CvBridgeError

# Declaring cvbridge for ROS
bridge = CvBridge()

cp = []  # control points
bf = []  # basis function
ep = []  # evaluated points (for vis)
# ros_skeleton = None

def image_callback(ros_image):
    global bridge, cp, bf, ep, ros_skeleton
    # try:
    if ros_image is not None:
      cv_binary = bridge.imgmsg_to_cv2(ros_image, "mono8")
    # print("Test the control node", len(cv_binary))
    # except CvBridgeError as e:
    #     print(e)
    
    # Convert binary to skeleton
    binary = np.where(cv_binary > 0, 1, cv_binary)  
    skeleton = skeletonize(binary, method='lee')
    skeleton = np.where(skeleton == 1, 255, skeleton)  
    # ros_skeleton = bridge.cv2_to_imgmsg(skeleton, "mono8")
    # Ordering pixels using NN-search
    pixels = np.argwhere(skeleton > 0)
    xinit = pixels[:, 1]
    yinit = pixels[:, 0]
    
    tstart = time.time()
    init_point = (149, 149)  # base pixel used as seed point for NN-search
    dist_mat = []

    for i in range(len(xinit)):
      dist = np.sqrt((xinit[i]-init_point[0])**2 + (yinit[i]-init_point[1])**2)
      dist_mat = np.append(dist_mat, dist)

    index = np.argsort(dist_mat)
    xinit, yinit = xinit[index], yinit[index]
    point_jump = (len(xinit))/2
    x = np.array(xinit[0::round(point_jump)])
    y = np.array(yinit[0::round(point_jump)])

    print(len(x))
    print(len(y))

    if (x[-1] != xinit[-1]) or (y[-1] != yinit[-1]) :
        x = np.append(x, xinit[-1])
        y = np.append(y, yinit[-1])
        if ((x[-1] - x[-2] == 1) or (x[-1] - x[-2] == -1)) and ((y[-1] - y[-2] == 1) or (y[-1] - y[-2] == -1)):
          x = np.delete(x, -2)
          y = np.delete(y, -2)

    print(x, y)
    tstop = time.time()
    timer = tstop - tstart

    # Spline fitting
    tstart = time.time()
    tck, u_params = splprep([x, y], k =2, s = 1)
    tstop = time.time()
    timer = tstop - tstart
    print("splprep", timer)
    
    # Extracting control points
    cx = tck[1][0]
    cx = cx.tolist()
    cy = tck[1][1]
    cy = cy.tolist()

    # List of control points
    cp = [cx[0], cx[1], cx[2], cy[0], cy[1], cy[2]]

    # Evaluating points on spline
    tstart = time.time()
    new_params = np.linspace(0,1,10)
    newPts = splev(new_params, tck)
    tstop = time.time()
    timer = tstop - tstart
    print("Splev", timer)
    x = newPts[0] 
    y = newPts[1]

    # Appending evaluated points to list
    ep = np.append(ep, x)
    ep = np.append(ep, y)
    ep = ep.tolist()    
    # Basis function block
    # setting parameters
    t0 = u_params[0]
    t1 = u_params[1]
    t2 = u_params[2]

    # # setting knot vectors
    u0 = u1 = u2 = 0
    u3 = u4 = u5 = 1

    # # computing middle row of basis func matrix
    n20t1 = 1
    n11t1 = ((u3-t1)/(u3-u2)) * n20t1
    n21t1 = ((t1-u2)/(u3-u2))*n20t1
    n02t1 = ((u3-t1)/(u3-u1))* n11t1
    n12t1 = ((t1-u1)/(u3-u1))*n11t1 + ((u4-t1)/(u4-u2))*n21t1 
    n22t1 = ((t1-u2)/(u4-u2))*n21t1

    bf = [n02t1, n12t1, n22t1]

    # basis = bspline_basis_set(tck[2], tck[0],  u)
    # np_basis = [lambdify(u, b, modules=['numpy']) for b in basis]
    # tstop = time.time()
    # timer = tstop - tstart
    # print("sympy basis set timing: ", timer)
    
    # basis_funcn = []
    # for bi in np_basis:
    #   for u_val in u_ticks:
    #     if bi(u_val) == 0 or bi(u_val)== 1:
    #       basis_funcn = basis_funcn
    #     else:
    #       basis_funcn = np.append(basis_funcn, bi(u_val))
  
    
def main(args):
  global ep
  # Initializing ROS
  rospy.init_node('image_skel_spline_trial')
  
  # Initializing Subscribers
  # binary img sub
  image_sub = rospy.Subscriber("vsbot/binary_image",Image,image_callback,  queue_size = 1)
  
  # Initializing publishers
  control_pts_pub = rospy.Publisher('vsbot/control_points', Float64MultiArray, queue_size= 1)
  basis_function_pub = rospy.Publisher('vsbot/basis_function', Float64MultiArray, queue_size= 1)
  eval_pts_pub = rospy.Publisher('vsbot/evaluated_points', Float64MultiArray, queue_size= 1)
  # skeleton_pts_pub = rospy.Publisher('vsbot/skeleton_points', Image, queue_size=1)

  # Declaring msgs to publish
  control_points = Float64MultiArray()
  basis_func = Float64MultiArray()
  eval_points = Float64MultiArray()
  
  # Publish msgs at 30 Hz
  r = 30 # rospy.get_param('vsbot/estimation/rate')
  rate = rospy.Rate(r)
  while not rospy.is_shutdown():
    # Publish control points
    control_points.data = cp
    control_pts_pub.publish(control_points)
    rospy.loginfo(control_points)

    # Publish basis func
    basis_func.data = bf
    basis_function_pub.publish(basis_func)

    # Publish evaluated data points for vis
    eval_points.data = ep
    eval_pts_pub.publish(eval_points)
    ep.clear()

    #Publish skeleton Points
    # if ros_skeleton !=None:
      # skeleton_pts_pub.publish(ros_skeleton)
    rate.sleep()
  
    



if __name__ == '__main__':
    main(sys.argv)

#!/usr/bin/env python3

import sys
import rospy
import cv2
import csv
import numpy as np
import math
from skimage.morphology import skeletonize
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from scipy.interpolate import splprep, splev
import time
from sympy import lambdify, bspline_basis_set
from sympy.abc import u
from cv_bridge import CvBridge, CvBridgeError
from encoderless_vs.srv import control_points, control_pointsResponse, bin_img, bin_imgResponse

# Declaring cvbridge for ROS
bridge = CvBridge()
binary_image_output = None

coeffs = []
coeff_list = []
ep = []
cp = []
kp = []
internal_knots = []
knot_point = []

# the following service callback definies the service that 
# takes in binary image input and gives control points output  
def control_points_service(data):  
  global bridge, coeffs, coeff_list, ep, internal_knots, knot_point, kp
  
  bin_rqst_tstart = time.time()
  resp_bin = binary_image_output(1)
  bin_rqst_tstop = time.time()
  binary_resp = resp_bin.output
   
  # ros binary image as the service input
  ros_binary_image = binary_resp
  
  skel_tstart = time.time() 
  cv_binary = bridge.imgmsg_to_cv2(ros_binary_image, "mono8")
  
  
  # Convert binary to skeleton
  binary = np.where(cv_binary > 0, 1, cv_binary)    
  skeleton = skeletonize(binary, method='lee')    
  skeleton = np.where(skeleton == 1, 255, skeleton)


 # Ordering pixels using NN-search
  pixels = np.argwhere(skeleton > 0)
  xinit = pixels[:, 1]
  yinit = pixels[:, 0]
  
  init_point = (149, 149)  # base pixel used as seed point for NN-search
  dist_mat = []
  for i in range(len(xinit)):
    dist = np.sqrt((xinit[i]-init_point[0])**2 + (yinit[i]-init_point[1])**2)
    dist_mat = np.append(dist_mat, dist)

  # print("All X", xinit)
  # print("All Y", yinit)

  # print("length of X", len(xinit))
  # print("length of Y", len(yinit))

  # print("X last point", xinit[0])
  # print("Y last point", yinit[0])
    
  index = np.argsort(dist_mat)
  xinit, yinit = xinit[index], yinit[index]
  num_of_segments = rospy.get_param("vsbot/shape_control/num_of_segments")
  k = rospy.get_param("vsbot/shape_control/degree")
  point_jump = (len(xinit))/((num_of_segments+k)-1)
  x = np.array(xinit[0::math.floor(point_jump)])
  y = np.array(yinit[0::math.floor(point_jump)])

  # print("Downsampled X", x)
  # print("Downsampled Y", y)
  
  if (x[-1] != xinit[-1]) or (y[-1] != yinit[-1]) :
      x = np.append(x, xinit[-1])
      y = np.append(y, yinit[-1])
      if len(x) > (num_of_segments+k):
        x = np.delete(x, -2)
        y = np.delete(y, -2)
      # if ((x[-1] - x[-2] == 1) or (x[-1] - x[-2] == -1)) and ((y[-1] - y[-2] == 1) or (y[-1] - y[-2] == -1)):
      #   x = np.delete(x, -2)
      #   y = np.delete(y, -2)
      # elif ((x[-1] - x[-2] == 2) or (x[-1] - x[-2] == -1)) and ((y[-1] - y[-2] == 1) or (y[-1] - y[-2] == -1)):
      #   x = np.delete(x, -2)
      #   y = np.delete(y, -2)
  
  # print("Downsampled to desired cpx", x)
  # print("Downsampled to desired cpy", y)
      
  # Spline fitting
  
  tck, u_params = splprep([x, y], k = k, s = 1)


  knots = tck[0]
     
  print("Internal knots", knots)

  knot_pts = splev(knots, tck)

  kx = knot_pts[0]
  kx = kx.tolist()
  ky = knots[1]
  ky = ky.tolist()

  kp.clear()
  for i in range(len(x)):
    kp.append(x[i]) 
    kp.append(y[i])


  print(kp)
  
  # print("internal knots",knots)

  # print("knots",knots)
  # print("params", u_params)
  # print("knot vector type", type(knots))
  
  # Extracting control points

  cx = tck[1][0]  
  cx = cx.tolist()
  cy = tck[1][1]
  cy = cy.tolist()

  # print("Control Points X", cx)
  # print("Control Points Y", cy)

  
  # List of control points
  # putting 1ist control points at the end as it is not changing and can be discarded
  # cp = []
  # print(type(cp))
  cp.clear()
  for i in range(len(cx)-1):
    cp.append(cx[i+1]) 
    cp.append(cy[i+1])
  
  # print("Type control points", type(cp))
#  cp = [x1, y1, x2, y2, x3, y3...] based on number of features

  ###### Computing theta, an additional feature to be added to cp ########
  # theta is to be computed using first and last cp

  # get first control point
  # x1 = cp[0]
  # y1 = cp[1]

  # # get last control point
  # x2 = cp[len(cp)-2]
  # y2 = cp[len(cp)-1]

  # base = abs(x2 - x1)
  # hy = math.sqrt(pow((x1 - x2),2) + pow((y1 - y2),2))

  # th = math.acos(base/hy)
  
  # ##### Appending this additional feature to cp ######
  # cp.append(th)

  # # print("# of control points: "+str(len(cx)))
  
  cp_resp = Float64MultiArray()
  cp_resp.data = cp  

  # print(cp)

  # Evaluate points

  new_params = np.linspace(0,1,10)
  new_pts = splev(new_params, tck)

  # print("New Points", new_pts)
  # print("New Points type", type(new_pts))
  # print("length of array", len(new_pts))
  # print("length of new_pts x", len(new_pts[0]))
  x = new_pts[0]
  x = x.tolist()
  y = new_pts[1]
  y = y.tolist()

  ep.clear()
  for i in range(len(x)):
    ep.append(x[i]) 
    ep.append(y[i])

  # coefficient evaluation of spline polynomials

  pol_params_1 = np.linspace(0, 0.4, 3)

  pol_pts1 = splev(pol_params_1, tck)

  pol_x1 = pol_pts1[0]
  pol_y1 = pol_pts1[1]

  pol_params_2 = np.linspace(0.5, 1, 3)


  # print("length of ep", len(ep))


  # print("type new pts x", type(x))

  # Appending evaluated points to list
  # ep = np.append(ep, x)
  # ep = np.append(ep, y)
  # ep.tolist()
  
  # print("Evaluated Points", ep) 

  
  # Basis function block
  # setting parameters
  # t0 = u_params[0]
  # t1 = u_params[1]
  # t2 = u_params[2]

  # # # setting knot vectors
  # u0 = u1 = u2 = 0
  # u3 = u4 = u5 = 1
 
  # computing middle row of coeff matrix
  # n20t1 = 1
  # n11t1 = ((u3-t1)/(u3-u2)) * n20t1
  # n21t1 = ((t1-u2)/(u3-u2))*n20t1
  # n02t1 = ((u3-t1)/(u3-u1))* n11t1
  # n12t1 = ((t1-u1)/(u3-u1))*n11t1 + ((u4-t1)/(u4-u2))*n21t1 
  # n22t1 = ((t1-u2)/(u4-u2))*n21t1
  # coeffs = [n02t1, n12t1, n22t1]
  # print("Coeffs", coeffs)
  
  basis = bspline_basis_set(tck[2], tck[0],  u)
  np_basis = [lambdify(u, b, modules=['numpy']) for b in basis]
    
  basis_funcn = []
  for bi in np_basis:
    for u_val in u_params:
      basis_funcn = np.append(basis_funcn, bi(u_val))
      # if bi(u_val) == 0 or bi(u_val)== 1:
      #   basis_funcn = basis_funcn
      # else:
      #   basis_funcn = np.append(basis_funcn, bi(u_val))
  
  
  coeffs = basis_funcn  

  # new_coeff = np.reshape(coeffs, (4, -1))
  # print(new_coeff)
  coeff_resp = Float64MultiArray()
  coeff_resp.data = coeffs 

  ep_resp = Float64MultiArray()
  ep_resp.data = ep

  kp_resp = Float64MultiArray()
  kp_resp.data = kp

  # service returns control points and coeffs
  return control_pointsResponse(cp_resp, ep_resp, kp_resp, coeff_resp) #, skel_tstart, skel_tstop, nn_tstart, nn_tstop, spl_tstart, spl_tstop, bin_rqst_tstart, bin_rqst_tstop)
  # return control_pointsResponse(cp_resp)
  
  
def main(args):
  global binary_image_output
  # Initializing ROS
  rospy.init_node('control_service_node')

  # Declaring the control points service
  cp_service = rospy.Service("control_points_output", control_points, control_points_service)

  rospy.wait_for_service("binary_image_output")

  # create a handle to call the service (The handle will send the request to the service)
  binary_image_output = rospy.ServiceProxy('binary_image_output', bin_img)

  # Initializing publishers
  # control_pts_pub = rospy.Publisher('vsbot/control_points', Float64MultiArray, queue_size= 1)
  # # basis_function_pub = rospy.Publisher('vsbot/basis_function', Float64MultiArray, queue_size= 1)
  # eval_pts_pub = rospy.Publisher('vsbot/evaluated_points', Float64MultiArray, queue_size= 1)
  # knot_pts_pub = rospy.Publisher('vsbot/knot_points', Float64MultiArray, queue_size= 1)

  # cpts = Float64MultiArray()
  # basis_func = Float64MultiArray()
  # eval_points = Float64MultiArray()
  # knot_points = Float64MultiArray()

  # rate = rospy.Rate(30)
  
    # Publish control points
    # cpts.data = cp
    # control_pts_pub.publish(cpts)

    # Publish basis func
    # basis_func.data = bf
    # basis_function_pub.publish(basis_func)

    # Publish evaluated data points for vis
    # eval_points.data = ep
    # eval_pts_pub.publish(eval_points)
    # ep.clear()

    # # Publish evaluated data points for vis
    # knot_points.data = knot_point
    # knot_pts_pub.publish(knot_points)
    # knot_point.clear()



  # waiting for binary service to be up
  

   
  rospy.spin()
  
if __name__ == '__main__':
    main(sys.argv)
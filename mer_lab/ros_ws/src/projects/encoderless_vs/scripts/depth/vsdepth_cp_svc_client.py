#!/usr/bin/env python3

import rospy
import sys
import time
from std_msgs.msg import Bool
import csv
from encoderless_vs.srv import control_points, control_pointsResponse

# global flag to start the control point service
cp_flag = None

def flag_cb(msg):
  global cp_flag
  cp_flag = msg.data

def main(args):

    # initialize the node
    rospy.init_node('cp_service_request_node')

    # declare the subscriber for the flag 
    flag_sub = rospy.Subscriber("/vsbot/control_flag", Bool, flag_cb, queue_size=1)

    # wait for control points service to be up
    rospy.wait_for_service('control_points_output')

    # create a handle to call the service
    control_points_output = rospy.ServiceProxy('control_points_output', control_points)

    f = open('time_record.csv', 'w')
    writer = csv.writer(f)
    writer.writerow(['Control Points', 'Service Request Time','Binary Img Rqst Start', 
    'Binary Img Rqst Stop', 'Skeleton Start', 'Skeleton End', 'Ordering Start', 
            'Ordering End', 'Spline Start', 'Spline End', 'Service Response Time'])

    
    # assigning a rate at which the control points service will be called
    # r = rospy.get_param('vsbot/estimation/rate')
    # r = rospy.get_param('vsbot_depth/estimation/rate')
    rate = rospy.Rate(30)
  
    while not rospy.is_shutdown():
      if cp_flag == True:   
        tstart = time.time()
        srv_resp = control_points_output(1)
        tstop = time.time()
        control_pts = srv_resp.cp.data
        skel_tstart = srv_resp.skel_tstart
        skel_tstop = srv_resp.skel_tstop
        nn_tstart = srv_resp.nn_tstart
        nn_tstop = srv_resp.nn_tstop
        spline_tstart = srv_resp.spl_tstart
        spline_tstop = srv_resp.spl_tstop
        bin_rqst_tstart = srv_resp.bin_tstart
        bin_rqst_tstop = srv_resp.bin_tstop

        # tstop = time.time()
        row_data = [control_pts, tstart , bin_rqst_tstart, bin_rqst_tstop, skel_tstart, 
                    skel_tstop, nn_tstart, nn_tstop, spline_tstart, spline_tstop, tstop]
        writer.writerow(row_data)
        rate.sleep()
    
    f.close()   
         

if __name__ == '__main__':
    main(sys.argv)

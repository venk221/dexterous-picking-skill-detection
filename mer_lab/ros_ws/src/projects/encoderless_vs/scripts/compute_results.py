#!/usr/bin/env python3

import math
import numpy as np
import csv
# import os
import sys
# import rospy
from os.path import expanduser
from datetime import datetime
r = 10


def read_data(file_name):
    
    # Read data
    err_data = []
    with open(file_name,'r') as csvfile:
        plots = csv.reader(csvfile, delimiter = ',')
        for row in plots:
            err_data.append(row)

    # # Post process the list
    err_data = err_data[1:]             
    err_data = [list( map(float,i) ) for i in err_data]
    err_data = np.array(err_data)

    return err_data


def norm(data):
    # Compute norm of error data in each row
    # And return as an np array
    n = []
    for row in data:
        err_norm = math.sqrt(row[0]**2 + row[1]**2)
        n.append(err_norm)
    n = np.array(n)
    # print(n)
    return n


def rt(data):
    time = 0.0

    lower_bound = 0.9*(data[0])
    upper_bound = 0.1*(data[0])
    lf = False      # true when lower bound is set
    uf = False      # true when upper bound is set

    rt_start = 0
    rt_stop = 0

    for i in range(data.size):
        if data[i] <= lower_bound and not lf:
            rt_start = i
            lf = True

        if data[i] <= upper_bound and not uf:
            rt_stop = i
            uf = True
    
    time  = (rt_stop - rt_start)/r


    return time


def st(data):
    time = 0.0
    bound = 0.02*(data[0])
    flag = False
    st_end = 0

    for i in range(data.size):
        if data[i] <= bound and not flag:
            st_end = i
            flag = True
    time = (st_end+1)/r
    return time


def ovsht(data):
    overshoot = 0.0
    overshoot_x = 0.0
    overshoot_y = 0.0

    x_data = data[0:,0]
    y_data = data[0:,1]

    # Compute overshoot in x
    z_cross_x = np.where(np.diff(np.signbit(x_data)))[0]

    if z_cross_x.size>0:
        if z_cross_x.size == 1:
            overshoot_x = 0.0
        elif z_cross_x.size == 2:
            segment = np.abs(x_data[z_cross_x[0]:z_cross_x[1]+1])
            overshoot_x = np.amax(segment)
        else:
            segment1 = np.abs(x_data[z_cross_x[0]:z_cross_x[1]+1])
            segment2 = np.abs(x_data[z_cross_x[1]:z_cross_x[2]+1])
            ov1 = np.amax(segment1)
            ov2 = np.amax(segment2)
            overshoot_x = min(ov1, ov2)

    # Compute overshoot in y
    z_cross_y = np.where(np.diff(np.signbit(y_data)))[0]

    if z_cross_y.size>0:
        if z_cross_y.size == 1:
            overshoot_y = 0.0
        elif z_cross_y.size == 2:
            segment = np.abs(y_data[z_cross_y[0]:z_cross_y[1]+1])
            overshoot_y = np.amax(segment)
        else:
            segment1 = np.abs(y_data[z_cross_y[0]:z_cross_y[1]+1])
            segment2 = np.abs(y_data[z_cross_y[1]:z_cross_y[2]+1])
            ov1 = np.amax(segment1)
            ov2 = np.amax(segment2)
            overshoot_y = min(ov1, ov2)

    # Compute average overshoot
    overshoot = math.sqrt(overshoot_x**2 + overshoot_y**2)
    
    # compute as % of error norm
    overshoot = (overshoot/(math.sqrt(x_data[0]**2 + y_data[0]**2)))*100
    return overshoot


def main(args):

    home = expanduser("~")


    ######## Select Path to experiment folder #############
    folder = "/baseline"
    # folder = "/shape"
    
    exp_folder = "/Pictures/repeatability" + folder
    # exp_folder = "/Pictures/Abhinav 2-18-22" + folder + "/servoing/exps/"
    
    exps = range(1,6)              ######## Add the experiment numbers #########
    
    for exp_no in exps:
        path_to_exp = home + exp_folder + "/"+str(exp_no) + "/err.csv"
        err_data = read_data(path_to_exp)
        # err_data = err_data[0:, 4:6]     ####### Uncomment for shape servoing ########

        # Compute Norm of Error
        err_norm = norm(err_data)

        # Compute rise time
        rise_time = rt(err_norm)

        # Compute settling time
        settling_time = st(err_norm)

        # Compute overshoot %
        overshoot = ovsht(err_data)

        # Format result
        s = ""
        s += f"Experiment No, {exp_no:.0f} \n"
        s += f"Rise time, {rise_time:.3f}s \n"
        s += f"Settling time, {settling_time:.3f}s \n"
        s += f"Overshoot, {overshoot:.2f}% \n"
        s += "\n \n"

        # Write data to file
        time_now = datetime.now()
        format_time = time_now.strftime("%m-%d-%Y-%H-%M-%S")
        file_path = home + "/Desktop/"+folder+format_time+".csv" 
        f = open(file_path, "a")
        f.write(s)
        f.close()
    
    print("Completed")


if __name__ == "__main__":
    main(sys.argv)

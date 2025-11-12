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

    # print(err_data)
    return err_data


def norm(data):
    # Compute norm of error data in each row
    # And return as an np array
    n = []
    o = []
    p = []
    for row in data:
        # print(row[0],row[1],row[2],row[3],row[4],row[5])
        err_norm_1 = math.sqrt(row[0]**2 + row[1]**2)
        n.append(err_norm_1)
        err_norm_2 = math.sqrt(row[2]**2 + row[3]**2)
        o.append(err_norm_2)
        err_norm_3 = math.sqrt(row[4]**2 + row[5]**2)
        p.append(err_norm_3)
    n = np.array(n)
    o = np.array(o)
    p = np.array(p)
    # print(n,o,p)
    return n, o , p


def rt(data):
    time = 0.0
    print(data[0])
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
    overshoot_1 = 0.0
    overshoot_x1 = 0.0
    overshoot_y1 = 0.0

    overshoot_2 = 0.0
    overshoot_x2 = 0.0
    overshoot_y2 = 0.0

    overshoot_3 = 0.0
    overshoot_x3 = 0.0
    overshoot_y3 = 0.0


    x1_data = data[0:,0]
    y1_data = data[0:,1]

    x2_data = data[0:,2]
    y2_data = data[0:,3]

    x3_data = data[0:,4]
    y3_data = data[0:,5]

    # Compute overshoot in x1
    z_cross_x1 = np.where(np.diff(np.signbit(x1_data)))[0]

    # print("np.signbit", np.signbit(x1_data))
    # print("np.diff", np.diff(np.signbit(x1_data)))

    # print(z_cross_x1)

    if z_cross_x1.size>0:
        if z_cross_x1.size == 1:
            segment = np.abs(x1_data[z_cross_x1[0]:])
            overshoot_x1 = np.amax(segment)
        elif z_cross_x1.size == 2:
            segment = np.abs(x1_data[z_cross_x1[0]:z_cross_x1[1]+1])
            overshoot_x1 = np.amax(segment)
        else:
            segment1 = np.abs(x1_data[z_cross_x1[0]:z_cross_x1[1]+1])
            segment2 = np.abs(x1_data[z_cross_x1[1]:z_cross_x1[2]+1])
            ov1 = np.amax(segment1)
            ov2 = np.amax(segment2)
            overshoot_x1 = min(ov1, ov2)

    # Compute overshoot in y1
    z_cross_y1 = np.where(np.diff(np.signbit(y1_data)))[0]

    if z_cross_y1.size>0:
        if z_cross_y1.size == 1:
            segment = np.abs(y1_data[z_cross_y1[0]:])
            overshoot_y1 = np.amax(segment)
        elif z_cross_y1.size == 2:
            segment = np.abs(y1_data[z_cross_y1[0]:z_cross_y1[1]+1])
            overshoot_y1 = np.amax(segment)
        else:
            segment1 = np.abs(y1_data[z_cross_y1[0]:z_cross_y1[1]+1])
            segment2 = np.abs(y1_data[z_cross_y1[1]:z_cross_y1[2]+1])
            ov1 = np.amax(segment1)
            ov2 = np.amax(segment2)
            overshoot_y1 = min(ov1, ov2)
    # print(overshoot_x1)
    # print(overshoot_y1)
    # Compute average overshoot
    overshoot_1 = math.sqrt(overshoot_x1**2 + overshoot_y1**2)
    
    # compute as % of error norm
    overshoot_1 = (overshoot_1/(math.sqrt(x1_data[0]**2 + y1_data[0]**2)))*100

    # Compute overshoot in x2
    z_cross_x2 = np.where(np.diff(np.signbit(x2_data)))[0]

    if z_cross_x2.size>0:
        if z_cross_x2.size == 1:
            segment = np.abs(x2_data[z_cross_x2[0]:])
            overshoot_x2 = np.amax(segment)
        elif z_cross_x2.size == 2:
            segment = np.abs(x2_data[z_cross_x2[0]:z_cross_x2[1]+1])
            overshoot_x2 = np.amax(segment)
        else:
            segment1 = np.abs(x2_data[z_cross_x2[0]:z_cross_x2[1]+1])
            segment2 = np.abs(x2_data[z_cross_x2[1]:z_cross_x2[2]+1])
            ov1 = np.amax(segment1)
            ov2 = np.amax(segment2)
            overshoot_x2 = min(ov1, ov2)

    # Compute overshoot in y2
    z_cross_y2 = np.where(np.diff(np.signbit(y2_data)))[0]

    if z_cross_y2.size>0:
        if z_cross_y2.size == 1:
            segment = np.abs(y2_data[z_cross_y2[0]:])
            overshoot_y2 = np.amax(segment)
        elif z_cross_y2.size == 2:
            segment = np.abs(y2_data[z_cross_y2[0]:z_cross_y2[1]+1])
            overshoot_y2 = np.amax(segment)
        else:
            segment1 = np.abs(y2_data[z_cross_y2[0]:z_cross_y2[1]+1])
            segment2 = np.abs(y2_data[z_cross_y2[1]:z_cross_y2[2]+1])
            ov1 = np.amax(segment1)
            ov2 = np.amax(segment2)
            overshoot_y2 = min(ov1, ov2)

    # Compute average overshoot
    overshoot_2 = math.sqrt(overshoot_x2**2 + overshoot_y2**2)
    
    # compute as % of error norm
    overshoot_2 = (overshoot_2/(math.sqrt(x2_data[0]**2 + y2_data[0]**2)))*100

    # Compute overshoot in x3
    z_cross_x3 = np.where(np.diff(np.signbit(x3_data)))[0]

    if z_cross_x3.size>0:
        if z_cross_x3.size == 1:
            segment = np.abs(x3_data[z_cross_x3[0]:])
            overshoot_x3 = np.amax(segment)
        elif z_cross_x3.size == 2:
            segment = np.abs(x3_data[z_cross_x3[0]:z_cross_x3[1]+1])
            overshoot_x3 = np.amax(segment)
        else:
            segment1 = np.abs(x3_data[z_cross_x3[0]:z_cross_x3[1]+1])
            segment2 = np.abs(x3_data[z_cross_x3[1]:z_cross_x3[2]+1])
            ov1 = np.amax(segment1)
            ov2 = np.amax(segment2)
            overshoot_x3 = min(ov1, ov2)

    # Compute overshoot in y3
    z_cross_y3 = np.where(np.diff(np.signbit(y3_data)))[0]

    if z_cross_y3.size>0:
        if z_cross_y3.size == 1:
            segment = np.abs(y3_data[z_cross_y3[0]:])
            overshoot_y3 = np.amax(segment)
        elif z_cross_y3.size == 2:
            segment = np.abs(y3_data[z_cross_y3[0]:z_cross_y3[1]+1])
            overshoot_y3 = np.amax(segment)
        else:
            segment1 = np.abs(y3_data[z_cross_y3[0]:z_cross_y3[1]+1])
            segment2 = np.abs(y3_data[z_cross_y3[1]:z_cross_y3[2]+1])
            ov1 = np.amax(segment1)
            ov2 = np.amax(segment2)
            overshoot_y3 = min(ov1, ov2)

    # Compute average overshoot
    overshoot_3 = math.sqrt(overshoot_x3**2 + overshoot_y3**2)
    
    # compute as % of error norm
    overshoot_3 = (overshoot_3/(math.sqrt(x3_data[0]**2 + y3_data[0]**2)))*100

    # print(overshoot_1, overshoot_2, overshoot_3)

    return overshoot_1, overshoot_2, overshoot_3


def main(args):

    home = expanduser("~")


    ######## Select Path to experiment folder #############
    # folder = "/baseline"
    # folder = "/shape"
    # folder = "/dl_vs"
    folder = "/shape_vs"
    exp_folder = "/Pictures/Dl_Exps" + folder + "/servoing/exps/"
    # exp_folder = "/Pictures/Abhinav 2-18-22" + folder + "/servoing/exps/"
    
    exps = range(1,21)              ######## Add the experiment numbers #########
    # exps = range(1,2)
    
    for exp_no in exps:
        path_to_exp = home + exp_folder + "/"+str(exp_no) + "/err.csv"
        err_data = read_data(path_to_exp)
        # err_data = err_data[0:, 4:6]     ####### Uncomment for shape servoing ########

        # Compute Norm of Error
        err_norm = norm(err_data)
        # print(err_norm)
        # Compute rise time
        rise_time_1 = rt(err_norm[0])
        rise_time_2 = rt(err_norm[1])
        rise_time_3 = rt(err_norm[2])

        # Compute settling time
        settling_time_1 = st(err_norm[0])
        settling_time_2 = st(err_norm[1])
        settling_time_3 = st(err_norm[2])

        # Compute overshoot %
        overshoot_1 = ovsht(err_data)[0]
        overshoot_2 = ovsht(err_data)[1]
        overshoot_3 = ovsht(err_data)[2]

        # Format result
        s = ""
        s += f"Experiment No, {exp_no:.0f} \n"
        s += f"Rise time 1, {rise_time_1:.3f}s \n"
        s += f"Settling time 1, {settling_time_1:.3f}s \n"
        s += f"Overshoot 1, {overshoot_1:.2f}% \n"
        s += f"Rise time 2, {rise_time_2:.3f}s \n"
        s += f"Settling time 2, {settling_time_2:.3f}s \n"
        s += f"Overshoot 2, {overshoot_2:.2f}% \n"
        s += f"Rise time 3, {rise_time_3:.3f}s \n"
        s += f"Settling time 3, {settling_time_3:.3f}s \n"
        s += f"Overshoot 3, {overshoot_3:.2f}% \n"
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

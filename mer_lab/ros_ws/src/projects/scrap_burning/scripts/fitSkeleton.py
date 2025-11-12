#!/usr/bin/python

import sys
import csv

import rospy
from geometry_msgs.msg import Point
from scrap_burning.srv import CurveFitting
from ros_pcl_manip.srv import Voxelize
from ros_pcl_manip.srv import LoadFile
from ros_pcl_manip.srv import ToFile

# CURRENTLY USED FOR TESTING
def main():
    if len(sys.argv) != 3:
        print("ERROR: Please provide the resolution and filepath")
        return

    rospy.init_node('fit_skeleton_node')

    # Setup service
    curveFitter = rospy.ServiceProxy("fit_curve", CurveFitting)

    loader = rospy.ServiceProxy('load_pcd_file', LoadFile)
    filteredCloud = loader("{0}/filteredGlobal.pcd".format(sys.argv[2]), "panda_link0")
    globalCloud = loader("{0}/globalCloud.pcd".format(sys.argv[2]), "panda_link0")

    print("Obtained both clouds, running curve fitting")

    # Got both clouds, get curve fitting
    res = curveFitter(filteredCloud.cloud, globalCloud.cloud, 0, 0, 0.0, 0.0, 0.0, 120, True, True, True, float(sys.argv[1]))
    print("Fit skeleton to cloud data")

    # Create file
    with open('/tmp/skeletonPointNormals.csv', 'w') as outCSVFile:
        csvWriter = csv.writer(outCSVFile, delimiter=',')
        for idx in range(len(res.sampled_points)):
            csvWriter.writerow([res.sampled_points[idx].x, res.sampled_points[idx].y, res.sampled_points[idx].z])
            csvWriter.writerow([res.sampled_normals[idx].x, res.sampled_normals[idx].y, res.sampled_normals[idx].z])
    print("Finished writing file")
    
if __name__ == '__main__':
    main()

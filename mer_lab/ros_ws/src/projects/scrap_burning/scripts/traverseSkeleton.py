#!/usr/bin/python

import csv
import sys

import rospy

from visualization_msgs.msg import Marker
from scrap_burning.srv import Traverse
from geometry_msgs.msg import Point
from ros_pcl_manip.srv import LoadFile
from scrap_burning.srv import CurveFitting

def dotProd(p1, p2):
    return p1.x * p2.x + p1.y * p2.y + p1.z * p2.z

def inv(p):
    ret = p
    ret.x *= -1
    ret.y *= -1
    ret.z *= -1
    return ret

def main():
    rospy.init_node('traverse_node')
    trav = rospy.ServiceProxy("traverse_path", Traverse)
    fit = rospy.ServiceProxy("fit_curve", CurveFitting)
    markPub = rospy.Publisher("visualization_marker", Marker)

    loader = rospy.ServiceProxy('load_pcd_file', LoadFile)

    loadedFilt = loader("{0}/{1}".format(sys.argv[1], "filteredGlobal.pcd"), "panda_link0")
    loadedGlob = loader("{0}/{1}".format(sys.argv[1], "globalCloud.pcd"), "panda_link0")

    curveFit = fit(loadedFilt.cloud, loadedGlob.cloud, 0, 0, 0, 0, 0, 120, True, False, True, 0.005)

    # Sample subset

    # Position marker
    marker = Marker()
    marker.header.frame_id="/panda_link0"
    marker.id = 0
    marker.type = marker.SPHERE_LIST
    marker.action = marker.ADD
    marker.scale.x = 0.02
    marker.scale.y = 0.02
    marker.scale.z = 0.02
    marker.color.a = 1.0
    marker.pose.orientation.w = 1.0
    for pt in curveFit.sampled_points:
        marker.points.append(pt)
    markPub.publish(marker)
    # Normal marker
    marker.id = 1
    for idp in range(len(curveFit.sampled_normals)):
        if idp > 0 and dotProd(curveFit.sampled_normals[idp], curveFit.sampled_normals[idp - 1]) < 0:
            curveFit.sampled_normals[idp] = inv(curveFit.sampled_normals[idp])
        offset = Point()
        # offset.x = curveFit.sampled_points[idp].x + curveFit.sampled_normals[idp].x
        # offset.y = curveFit.sampled_points[idp].y + curveFit.sampled_normals[idp].y
        # offset.z = curveFit.sampled_points[idp].z + curveFit.sampled_normals[idp].z
        offset.x = curveFit.sampled_normals[idp].x
        offset.y = curveFit.sampled_normals[idp].y
        offset.z = curveFit.sampled_normals[idp].z
        marker.points.append(offset)
    markPub.publish(marker)

    pointDelta = len(curveFit.sampled_points) / 13
    print(len(curveFit.sampled_points))
    pts = []
    norms = []
    for idp in range(0, len(curveFit.sampled_points), pointDelta):
        pts.append(curveFit.sampled_points[idp])
        norms.append(curveFit.sampled_normals[0])

    trav(pts, norms, float(sys.argv[2]))
    


if __name__ == "__main__":
    main()

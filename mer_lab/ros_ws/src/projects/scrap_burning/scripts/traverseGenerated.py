#!/usr/bin/python

import csv
import sys

import rospy

from scrap_burning.srv import Traverse
from geometry_msgs.msg import Point

def main():
    rospy.init_node('traverse_node')
    trav = rospy.ServiceProxy("traverse_path", Traverse)
    with open(sys.argv[1]) as pointFile:
        data = []
        reader = csv.reader(pointFile, delimiter=',')
        for row in reader:
            print(row)
            data.append(Point(float(row[0]), float(row[1]), float(row[2])))
        
        # Alternating point/normal rows
        points = []
        normals = []
        for i in range(0, len(data) - 1, 2):
            points.append(data[i])
            normals.append(data[i + 1])
        points = sorted(points, key=lambda lhs: lhs.z)
        # Send request
        for pt in points:
            print(pt)
        print()
        print(normals)
        trav(points, normals, float(sys.argv[2]))


if __name__ == "__main__":
    main()
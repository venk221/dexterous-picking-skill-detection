#!/usr/bin/python

import rospy

from geometry_msgs.msg import Pose
from scrap_burning.srv import RelaxedPath

def main():
    rospy.init_node('traverse_node')
    ex = rospy.ServiceProxy("execute_relaxed_path", RelaxedPath)

    start = Pose()
    start.position.x = 0.25
    start.position.y = 0.0
    start.position.z = -0.4
    start.orientation.x = 0
    start.orientation.y = 0
    start.orientation.z = 0
    start.orientation.w = 1

    ex([start], 1.0)

if __name__ == "__main__":
    main()

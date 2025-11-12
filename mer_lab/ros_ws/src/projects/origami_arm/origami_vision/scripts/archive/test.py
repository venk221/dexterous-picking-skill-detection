#!/usr/bin/env python3
import rospy
from astar import MyAstar



# Jani added
def test_module():
    print("I think it's working")

#Jani added
def main():

    
    rospy.init_node('test_module')        
    r = rospy.Rate(10)
    astar = MyAstar(1,2,3,4,5)
    while not rospy.is_shutdown():
        test_module()
        r.sleep()
    rospy.spin()
# print(astar)
# print("Ran")

#Jani added
if __name__=='__main__':
    main()

#!/usr/bin/env python

import rospy
import tf
import tf.transformations as tf_trans
from geometry_msgs.msg import PoseStamped

def get_transformation():
    rospy.init_node('tf_listener_node', anonymous=True)

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)  # 10 Hz
    while not rospy.is_shutdown():
        try:
            # Get the transformation between base_link and end_effector.
            (trans, rot) = listener.lookupTransform('panda_link0', 'panda_link8', rospy.Time(0))

            # Print the rotation part which is the quaternion (x, y, z, w)
            rospy.loginfo("Quaternion: x=%f, y=%f, z=%f, w=%f" % (rot[0], rot[1], rot[2], rot[3]))
            euler = tf_trans.euler_from_quaternion(rot)
            euler_degrees = [angle for angle in euler]
            rospy.loginfo("Euler Angles (degrees): roll=%f, pitch=%f, yaw=%f" % tuple(euler_degrees))


            rospy.loginfo("Translation: x=%f, y=%f, z=%f" % (trans[0], trans[1], trans[2]))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("Transform not found!")
            pass

        rate.sleep()

if __name__ == '__main__':
    try:
        get_transformation()
    except rospy.ROSInterruptException:
        pass

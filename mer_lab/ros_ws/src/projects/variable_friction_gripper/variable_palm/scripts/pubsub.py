#!/usr/bin/env python3

import argparse
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray

# write publisher and subscriber for pose and panner

class PubSub():

    def publisher(self, topic_name, publisher_name, msg, queue_size=10, rate=10):
        pub = rospy.Publisher(topic_name, msg.__class__, queue_size=queue_size)
        rospy.init_node(publisher_name, anonymous=True)
        rate = rospy.Rate(rate)

        while not rospy.is_shutdown():
            pub.publish(msg)
            rate.sleep()

    def subscriber(self, topic_name, subscriber_name, msg, callback_fn):
        
        def callback(data, callback_fn):
            callback_fn(data)

        rospy.init_node(subscriber_name, anonymous=True)
        rospy.Subscriber(topic_name, msg.__class__, callback, (callback_fn))
        rospy.spin()



if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--isPub', default=1, type=int)
    
    args = parser.parse_args()

    #! DEMO on how to use the pub-sub class for Float32MultiArray msgs
    pubsub = PubSub()
    isPub = args.isPub
    
    msg = Float32MultiArray()
    if isPub:
        sample_pose = np.random.rand(3,4).astype(np.float32)
        try:
            print('Publishing...')
            msg.data = sample_pose.reshape(-1).tolist()
            print(sample_pose, type(msg.data))
        
            pubsub.publisher('gripper_pose','gripper_pose_pub',msg)
        except rospy.ROSInterruptException:
            pass
    else:
        print('Subscribing...')
        
        def fn(data):
            pose = np.array(data.data).reshape(3,4)
            rospy.loginfo(pose)
        pubsub.subscriber('gripper_pose','gripper_pose_sub',msg,fn)
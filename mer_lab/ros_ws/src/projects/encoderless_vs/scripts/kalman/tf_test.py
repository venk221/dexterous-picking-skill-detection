import tf
import tf2_ros as tf2
import numpy as np
import rospy
from tf2_msgs.msg import TFMessage
import threading
import geometry_msgs.msg
from geometry_msgs.msg import PointStamped

last_update_lock = None
tfBuffer = None
last_update = None
listener = None

def transform():    
    # listener.waitf2orTransform('panda_link1', 'panda_link0', rospy.Time.now(), rospy.Duration(4))
    # point_in_world = listener.transformPoint("world", point_stamped)
    # print(listener.frameExists('panda_link0'))

    tfBuffer = tf2.Buffer()
    listener = tf2.TransformListener(tfBuffer)

    tf_1 = tfBuffer.lookup_transform('panda_link0', 'panda_link1', rospy.Time(0), rospy.Duration(1.0))    
    
    t1 = [tf_1.transform.translation.x, tf_1.transform.translation.y, tf_1.transform.translation.z]
    r1 = [tf_1.transform.rotation.x, tf_1.transform.rotation.y, tf_1.transform.rotation.z, tf_1.transform.rotation.w]

    tf_2 = tfBuffer.lookup_transform('panda_link0', 'panda_link2', rospy.Time(0), rospy.Duration(1.0))    
    
    t2 = [tf_2.transform.translation.x, tf_2.transform.translation.y, tf_2.transform.translation.z]
    r2 = [tf_2.transform.rotation.x, tf_2.transform.rotation.y, tf_2.transform.rotation.z, tf_2.transform.rotation.w]

    tf_3 = tfBuffer.lookup_transform('panda_link0', 'panda_link3', rospy.Time(0), rospy.Duration(1.0))    
    
    t3 = [tf_3.transform.translation.x, tf_3.transform.translation.y, tf_3.transform.translation.z]
    r3 = [tf_3.transform.rotation.x, tf_3.transform.rotation.y, tf_3.transform.rotation.z, tf_3.transform.rotation.w]

    tf_4 = tfBuffer.lookup_transform('panda_link0', 'panda_link4', rospy.Time(0), rospy.Duration(1.0))    
    
    t4 = [tf_4.transform.translation.x, tf_4.transform.translation.y, tf_4.transform.translation.z]
    r4 = [tf_4.transform.rotation.x, tf_4.transform.rotation.y, tf_4.transform.rotation.z, tf_4.transform.rotation.w]

    tf_5 = tfBuffer.lookup_transform('panda_link0', 'panda_link5', rospy.Time(0), rospy.Duration(1.0))    
    
    t5 = [tf_5.transform.translation.x, tf_5.transform.translation.y, tf_5.transform.translation.z]
    r5 = [tf_5.transform.rotation.x, tf_5.transform.rotation.y, tf_5.transform.rotation.z, tf_5.transform.rotation.w]

    tf_6 = tfBuffer.lookup_transform('panda_link0', 'panda_link6', rospy.Time(0), rospy.Duration(1.0))    
    
    t6 = [tf_6.transform.translation.x, tf_6.transform.translation.y, tf_6.transform.translation.z]
    r6 = [tf_6.transform.rotation.x, tf_6.transform.rotation.y, tf_6.transform.rotation.z, tf_6.transform.rotation.w]

    tf_7 = tfBuffer.lookup_transform('panda_link0', 'panda_link7', rospy.Time(0), rospy.Duration(1.0))    
    
    t7 = [tf_7.transform.translation.x, tf_7.transform.translation.y, tf_7.transform.translation.z]
    r7 = [tf_7.transform.rotation.x, tf_7.transform.rotation.y, tf_7.transform.rotation.z, tf_7.transform.rotation.w]

    print(t7, r7)

# def check_for_reset():
#         global last_update
#         # Lock to prevent different threads racing on this test and update.
#         # https://github.com/ros/geometry2/issues/341
#         with last_update_lock:
#             now = rospy.Time.now()
#             if now < last_update:
#                 rospy.logwarn("Detected jump back in time of %fs. Clearing TF buffer." % (last_update - now).to_sec())
#                 tfBuffer.clear()
#             last_update = now

# def callback(data):
#         check_for_reset()
#         who = data._connection_header.get('callerid', "default_authority")
#         print("WHO", who)
#         for transform in data.transforms:
#             tfBuffer.set_transform(transform, who)

# def test_transform():
#     t = tf.Transformer(True, rospy.Duration(10.0))
#     t.getFrameStrings()
#     print(t.getFrameStrings())
#     m = geometry_msgs.msg.TransformStamped()
#     m.header.frame_id = 'panda_link1'
#     m.child_frame_id = 'panda_link2'
#     m.transform.translation.x = 2.71828183
#     m.transform.rotation.w = 1.0
#     t.setTransform(m)
#     print(t.getFrameStrings())
#     print(t.lookupTransform('panda_link1', 'panda_link2', rospy.Time(0)))


def main():
    # global last_update_lock, tfBuffer, last_update
    global listener
    rospy.init_node('tf_test')  
    


    # tfBuffer = tf2.Buffer()
    # listener = tf2.TransformListener(tfBuffer)
    
    # last_update = rospy.Time.now()
    # last_update_lock = threading.Lock()
    # coord_sub = rospy.Subscriber("/tf", TFMessage, callback, queue_size = 1)
    # rospy.Subscriber("/sometopic", PointStamped, transform)

    # listener.waitf2orTransform('panda_link1', 'panda_link0', rospy.Time(), rospy.Duration(4))
    while not rospy.is_shutdown():
        transform()
    rospy.spin()

if __name__ == '__main__':
    main()
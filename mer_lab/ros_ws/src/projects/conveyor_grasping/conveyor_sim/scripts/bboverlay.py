#!/usr/bin/env python
from __future__ import print_function

import math
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from conveyor_sim.msg import MetaImage
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/bounding_box",MetaImage,self.callback)

  def callback(self,data):
    img_data = data.image
    try:
      cv_image = self.bridge.imgmsg_to_cv2(img_data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    for i in range(len(data.boxes)):
      cv2.rectangle(cv_image,
                    (data.boxes[i].x - data.boxes[i].width / 2, data.boxes[i].y - data.boxes[i].height / 2),
                    (data.boxes[i].x + data.boxes[i].width / 2, data.boxes[i].y + data.boxes[i].height / 2),
                    (255, 0, 0))
      cv2.putText(cv_image, data.labels[i], (data.boxes[i].x - data.boxes[i].width / 2, data.boxes[i].y + data.boxes[i].height / 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

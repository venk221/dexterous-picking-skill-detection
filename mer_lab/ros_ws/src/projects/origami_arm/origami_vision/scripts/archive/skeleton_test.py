#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from skimage.morphology import skeletonize
import os

bridge = CvBridge() # bridge object
HOME = os.path.expanduser('~')

def read_img():
    img = cv2.imread(HOME + "/bin1.jpg")
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return img


def main():
    # Read
    binary_img = read_img()

    # Skeletonize
    skeleton = skeletonize(binary_img, method='lee')

    # Display skeleton on image
    skeleton = np.where(skeleton>0, 255, skeleton)
    cv2.imshow("skeleton_img", skeleton)
    cv2.waitKey(0)


if __name__ == "__main__":
    main()
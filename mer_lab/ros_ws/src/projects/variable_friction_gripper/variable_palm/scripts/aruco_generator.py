import numpy as np
import random
import cv2

# get ArUCO dicionary of size 4x4 with 100 unique fiducials
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)

#init tag
tag = np.zeros((300, 300, 1), dtype="uint8")

#TODO: set num of aruco markers to be generated
num_aruco = 7

for aruco_no in np.random.randint(0,100,num_aruco):
    
    cv2.aruco.drawMarker(arucoDict, aruco_no, 300, tag, 1)
    print(aruco_no)
    cv2.imshow("ArUCo Tag", tag)
    cv2.waitKey(0)
    
    # write the generated ArUCo tag
    #Todo: change file path, currently storing in pwd
    cv2.imwrite('aruco_res_'+str(aruco_no)+'.png', tag)
#!/usr/bin/env python3
import cv2

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
arucoParams = cv2.aruco.DetectorParameters_create()

#select the first camera
cap = cv2.VideoCapture(0)

#check if the cam is opened
if not cap.isOpened():
    print('Unable to access camera')

while(True):
    ret, frame = cap.read()
    
    (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict,parameters=arucoParams)

    out = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    # print(out)
    cv2.imshow('aruco_dt', out)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
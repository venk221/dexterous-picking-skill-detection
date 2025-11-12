#!/usr/bin/env python3
import cv2
import numpy as np

# TODO: Always set the CALIB_DIR
CALIB_DIR='ros_ws/src/projects/variable_friction_gripper/variable_palm/calib/'
isDebug = True
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
arucoParams = cv2.aruco.DetectorParameters_create()

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print('Unable to access camera')


def read_camera_file(calib_file_path):
    '''
    the function expects relative path to a .txt file
    parse the K matrix/ distortion values from the calibration file
    '''
    cam_matrix = []
    #read file
    with open(calib_file_path, 'r') as f:
        cam_matrix = f.readlines()
    #create cam_matrix matrix and do type casting
    cam_matrix = np.array([x.split() for x in cam_matrix], dtype=np.float32)
    # print(K)
    return cam_matrix

# get camera intrinsics and distortion
##! default camera intrinsic matrix should be stored in 'K.txt' and distortion in 'distortion.txt'
K = read_camera_file(CALIB_DIR+'K.txt')
distortion = read_camera_file(CALIB_DIR+'distortion.txt')

count = 0
while(True):
    ret, frame = cap.read()
    
    #detect aruco markers in the frame
    (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict,parameters=arucoParams)
    
    # compute pose
    if len(corners) > 0:
        for i in range(0, len(ids)):
            # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, K, distortion)
            if isDebug:
                copy = frame.copy()

                #Draw Aruco markers
                cv2.aruco.drawDetectedMarkers(copy, corners, ids)
                # Draw Axes
                cv2.aruco.drawAxis(copy, K, distortion, rvec, tvec, 0.01)  
                
                cv2.imshow('aruco_dt', cv2.resize(copy, (800,600)))
    
    #save images when 's' is pressed on keyboard
    if(cv2.waitKey(1) == ord('s')):
        count+=1
        cv2.imwrite('calib'+str(count)+'.png', frame)
        print('image captured')
    #quit when 'q' is pressed on keyboard
    if cv2.waitKey(1) == ord('q') or count>10:
        break

cap.release()
cv2.destroyAllWindows()
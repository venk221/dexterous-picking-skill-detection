import numpy as np
import cv2
import glob


class CameraCalib():
    def __init__(self, rows=10, cols=7):
        '''
        Class to read and write camera intrinsics and extrinsics (to a *.txt file)
        Inputs: 
            inner rows and cols of chessboard
            directory with chesboard samples 
        '''
        self.num_rows = rows
        self.num_cols = cols

        #init camera properties
        self.intrinsics = []
        self.extrinsics = []
        self.distortion = []
        
    def calibCam(self, samples_dir):
        '''
        Compute all camera properties including K matrix, extrinsics, distortion (based on Rad-Tan upto 5 degrees)
        inputs: directory containing the chessboard samples from the camera

        Note: the function only takes in *jpg files
        '''
        # termination criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(9,6,0)
        objp = np.hstack([np.mgrid[0:self.num_rows,0:self.num_cols].T.reshape(-1,2), np.zeros((self.num_rows*self.num_cols,1), np.float32)])
        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.
        images = glob.glob(samples_dir+'*.jpeg')

        for fname in images:
            img = cv2.imread(fname)

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (self.num_rows,self.num_cols), 
                            cv2.CALIB_CB_ADAPTIVE_THRESH + 
                            cv2.CALIB_CB_FAST_CHECK)
            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints.append(objp)
                corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
                imgpoints.append(corners2)
                # Draw and display the corners
                # cv2.drawChessboardCorners(img, (7,6), corners2, ret)
                # cv2.imshow('img', img)
                # cv2.waitKey(500)
        
        ret, intrinsics, distortion, rvecs, tvecs = cv2.calibrateCamera(np.array(objpoints, dtype=np.float32), np.array(imgpoints).reshape((len(images),-1,2)), gray.shape[::-1], None, None)

        self.intrinsics = intrinsics
        self.extrinsics = [rvecs, tvecs]
        self.distortion = distortion 
        
        print('Calibration data computed successfully')


    def writeCalibration(self, data, filename='calibration.txt'):
        '''
        Abstract txt file writing function
        Input: 
            data: to be written
            filename: relative file path
        '''
        np.savetxt(filename, data, delimiter=' ')
        print('Calibration data successfully written to file: ', filename)


if __name__ == '__main__':
    
    root_dir='ros_ws/src/projects/variable_friction_gripper/variable_palm/calib/'
    calibration = CameraCalib()
    calibration.calibCam(root_dir)

    calibration.writeCalibration(calibration.intrinsics,root_dir+'K.txt')
    calibration.writeCalibration(calibration.distortion,root_dir+'distortion.txt') 
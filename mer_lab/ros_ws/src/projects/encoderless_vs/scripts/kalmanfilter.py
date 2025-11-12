#!/usr/bin/env python3.8

import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Bool
import sys
# print(sys.path)

class KalmanFilter(object):
    def __init__(self, dt, x_std_meas, y_std_meas, vx_std_meas, vy_std_meas):
        """
        :param dt: sampling time (time for 1 cycle)
        :param u_x: acceleration in x-direction
        :param u_y: acceleration in y-direction
        :param std_acc: process noise magnitude
        :param x_std_meas: standard deviation of the measurement in x-direction
        :param y_std_meas: standard deviation of the measurement in y-direction
        """
        # Counter to predict the first estimate
        self.count = 1
        # flag subscriber to stop correction        
        self.flag_sub = rospy.Subscriber("/feature/Flag", Bool, self.flag_cb, queue_size=1)
        self.do_correction = None        
        # Define sampling time
        self.dt = dt
        # Intial State
        self.X = np.matrix([[320], [240], [0.001], [0.001], [0.01], [0.01]])
        # declare states in prediction
        self.predicted_x = np.zeros(self.X.shape)
        # declare states after correction
        self.corrected_x = np.zeros(self.X.shape)
        # Define the State Transition Matrix A
        self.F = np.array([[1, 0, dt,  0, 0.5*dt**2, 0], 
              [0, 1,  0, dt,         0, 0.5*dt**2], 
              [0, 0,  1,  0,        dt,         0],
              [0, 0,  0,  1,         0,        dt],
              [0, 0,  0,  0,         1,         0],
              [0, 0,  0,  0,         0,         1]])  
        
        # Define Measurement Mapping Matrix
        self.H = np.matrix([[1, 0, 0, 0, 0, 0],
                            [0, 1, 0, 0, 0, 0],
                            [0, 0, 1, 0, 0, 0],
                            [0, 0, 0, 1, 0, 0]])
        #Initial Process Noise Covariance
        self.Q = np.diag([50, 50, 50, 50, 3, 0.1])

        #Initial Measurement Noise Covariance
        self.R = np.matrix([[x_std_meas**2, 0, 0, 0],
                           [0, y_std_meas**2, 0, 0],
                           [0, 0, vx_std_meas**2, 0],
                           [0, 0, 0, vy_std_meas**2]])
        #Initial error Covariance Matrix
        self.P = np.diag([0.01, 0.01, 10, 10, 2, 2])

        # declare lists to average accelaration and velocity in prediction steps
        self.p_vel_list_x = []
        self.p_vel_list_y = []
        self.p_acc_list_x = []
        self.p_acc_list_y = []

        # declare lists to average accelaration and velocity in correction steps
        self.c_vel_list_x = []
        self.c_vel_list_y = []
        self.c_acc_list_x = []
        self.c_acc_list_y = []
        
        self.avg_length = 5
    
    def flag_cb(self, msg):
        self.do_correction = msg.data
        # print("skip correction?", self.do_correction)

    def predict(self):
        # Refer to :Eq.(9) and Eq.(10)  in https://machinelearningspace.com/object-tracking-simple-implementation-of-kalman-filter-in-python/?preview_id=1364&preview_nonce=52f6f1262e&preview=true&_thumbnail_id=1795
        # Update time state
        #x_k =Fx_(k-1)     Eq.(9)        
        self.X = np.dot(self.F, self.X)
        # Calculate error covariance
        # P= A*P*A' + Q               Eq.(10)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        # vx = self.X[2][0,0]
        # vy = self.X[3][0,0]
        # ax = self.X[4][0,0]
        # ay = self.X[5][0,0]
        # print("Corrected velocity before velocity correction", vx, vy)
        # self.p_vel_list_x = np.append(vx, self.p_vel_list_x)
        # self.p_vel_list_x = np.asarray(self.p_vel_list_x)
        # self.p_vel_list_y = np.append(vy, self.p_vel_list_y)
        # self.p_vel_list_y = np.asarray(self.p_vel_list_y)
        # self.p_acc_list_x = np.append(ax, self.p_acc_list_x)
        # self.p_acc_list_x = np.asarray(self.p_acc_list_x)
        # self.p_acc_list_y = np.append(ay, self.p_acc_list_y)
        # self.p_acc_list_y = np.asarray(self.p_acc_list_y)     
        # if len(self.p_vel_list_x)<self.avg_length:
        #     vx_av = np.average(self.p_vel_list_x)
        #     vy_av = np.average(self.p_vel_list_y)
        #     ax_av = np.average(self.p_acc_list_x)
        #     ay_av = np.average(self.p_acc_list_y)
        # else:
        #     # print("1st ten velocities", self.p_vel_list_x[0:10])    
        #     vx_av = np.average(self.p_vel_list_x[0:self.avg_length])
        #     vy_av = np.average(self.p_vel_list_y[0:self.avg_length])
        #     ax_av = np.average(self.p_acc_list_x[0:self.avg_length])
        #     ay_av = np.average(self.p_acc_list_y[0:self.avg_length])

        # self.X[2] = [vx_av]
        # self.X[3] = [vy_av]      
        # self.X[4] = [ax_av]
        # self.X[5] = [ay_av]
        
        # self.count = self.count + 1
        return self.X

    def update(self, z):
        # Refer to :Eq.(11), Eq.(12) and Eq.(13)  in https://machinelearningspace.com/object-tracking-simple-implementation-of-kalman-filter-in-python/?preview_id=1364&preview_nonce=52f6f1262e&preview=true&_thumbnail_id=1795
        # S = H*P*H'+R
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        I = np.eye(self.H.shape[1])
        # Calculate the Kalman Gain
        # K = P * H'* inv(H*P*H'+R)
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.X = np.ceil(self.X + np.dot(K, (z - np.dot(self.H, self.X))))
        self.P = (I - (K * self.H)) * self.P

        # vx = self.X[2][0,0]
        # vy = self.X[3][0,0]
        # ax = self.X[4][0,0]
        # ay = self.X[5][0,0]
        # # print("Corrected velocity before velocity correction", vx, vy)
        # self.c_vel_list_x = np.append(vx, self.c_vel_list_x)
        # self.c_vel_list_x = np.asarray(self.c_vel_list_x)
        # self.c_vel_list_y = np.append(vy, self.c_vel_list_y)
        # self.c_vel_list_y = np.asarray(self.c_vel_list_y)
        # self.c_acc_list_x = np.append(ax, self.c_acc_list_x)
        # self.c_acc_list_x = np.asarray(self.c_acc_list_x)
        # self.c_acc_list_y = np.append(ay, self.c_acc_list_y)
        # self.c_acc_list_y = np.asarray(self.c_acc_list_y) 
        # if len(self.c_vel_list_x)<self.avg_length:
        #     vx_av = np.average(self.c_vel_list_x)
        #     vy_av = np.average(self.c_vel_list_y)
        #     ax_av = np.average(self.c_acc_list_x)
        #     ay_av = np.average(self.c_acc_list_y)
        # else:
        #     # print("1st ten velocities", self.c_vel_list_x[0:10]) 
        #     vx_av = np.average(self.c_vel_list_x[0:self.avg_length])
        #     vy_av = np.average(self.c_vel_list_y[0:self.avg_length])
        #     ax_av = np.average(self.c_acc_list_x[0:self.avg_length])
        #     ay_av = np.average(self.c_acc_list_y[0:self.avg_length])

        # self.X[2] = [vx_av]
        # self.X[3] = [vy_av] 
        # self.X[4] = [ax_av]
        # self.X[5] = [ay_av]

        return self.X
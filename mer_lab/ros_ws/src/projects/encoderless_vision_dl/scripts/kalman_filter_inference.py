
import numpy as np

def kalman_estimate(self, img, feature):

        # print("latest marker flag array", self.marker_flag)

        # print("input in kalman filter function", feature)       

        yellow = (0, 255, 255)
        green = (0, 255, 0)
        red = (0, 0, 255)
        blue = (255, 0, 0)

        # # Detect feature points
        # joint1
        cx1 = feature[0][0]
        cy1 = feature[0][1]

        self.cx1_list = np.append(cx1, self.cx1_list)
        self.cy1_list = np.append(cy1, self.cy1_list)

        dx1 = self.cx1_list[0]-self.cx1_list[1]
        dy1 = self.cy1_list[0]-self.cy1_list[1]

        vx1 = dx1
        vy1 = dy1

        measured1 = np.array([[np.float32(cx1)], [np.float32(cy1)], [np.float32(vx1)], [np.float32(vy1)]])

        predicted1 = kf1.predict()

        if self.marker_flag[0] == True:
            self.updated1 = np.asarray(kf1.update(measured1))
            # print("Updated when True", updated)
            self.measured1_list = np.append(self.updated1, self.measured1_list) 
        elif self.marker_flag[0] == False:
            # rospy.sleep(2)
            new_measured1 = np.array([[np.float32(self.measured1_list[0])], [np.float32(self.measured1_list[1])], [np.float32(self.measured1_list[2])], [np.float32(self.measured1_list[3])]])
            self.updated1 = np.asarray(kf1.update(new_measured1))
            self.measured1_list = np.append(self.updated1, self.measured1_list)
            # self.cx1_list = np.append(self.updated1[0], self.cx1_list)
            # self.cy1_list = np.append(self.updated1[1], self.cy1_list)

        # cv2.circle(img, (int(cx1), int(cy1)), 5, green, -1)

        # joint2
        cx2 = feature[1][0]
        cy2 = feature[1][1]

        self.cx2_list = np.append(cx2, self.cx2_list)
        self.cy2_list = np.append(cy2, self.cy2_list)

        dx2 = self.cx2_list[0]-self.cx2_list[1]
        dy2 = self.cy2_list[0]-self.cy2_list[1]

        vx2 = dx2
        vy2 = dy2

        measured2 = np.array([[np.float32(cx2)], [np.float32(cy2)], [np.float32(vx2)], [np.float32(vy2)]])

        predicted2 = kf2.predict()

        if self.marker_flag[1] == True:
            self.updated2 = np.asarray(kf2.update(measured2))
            # print("Updated when True", updated)
            self.measured2_list = np.append(self.updated2, self.measured2_list) 
        elif self.marker_flag[1] == False:
            # rospy.sleep(2)
            new_measured2 = np.array([[np.float32(self.measured2_list[0])], [np.float32(self.measured2_list[1])], [np.float32(self.measured2_list[2])], [np.float32(self.measured2_list[3])]])
            self.updated2 = np.asarray(kf2.update(new_measured2))
            self.measured2_list = np.append(self.updated2, self.measured2_list)
            # self.cx2_list = np.append(self.updated2[0], self.cx2_list)
            # self.cy2_list = np.append(self.updated2[1], self.cy2_list)

        

        #joint3
        cx3 = feature[2][0]
        cy3 = feature[2][1] 

        self.cx3_list = np.append(cx3, self.cx3_list)
        self.cy3_list = np.append(cy3, self.cy3_list)

        dx3 = self.cx3_list[0]-self.cx3_list[1]
        dy3 = self.cy3_list[0]-self.cy3_list[1]

        vx3 = dx3
        vy3 = dy3

        measured3 = np.array([[np.float32(cx3)], [np.float32(cy3)], [np.float32(vx3)], [np.float32(vy3)]])

        predicted3 = kf3.predict()

        if self.marker_flag[2] == True:
            self.updated3 = np.asarray(kf3.update(measured3))
            # print("Updated when True", updated)
            self.measured3_list = np.append(self.updated3, self.measured3_list) 
        elif self.marker_flag[2] == False:
            # rospy.sleep(2)
            new_measured3 = np.array([[np.float32(self.measured3_list[0])], [np.float32(self.measured3_list[1])], [np.float32(self.measured3_list[2])], [np.float32(self.measured3_list[3])]])
            self.updated3 = np.asarray(kf3.update(new_measured3))
            self.measured3_list = np.append(self.updated3, self.measured3_list)
            # self.cx3_list = np.append(self.updated3[0], self.cx3_list)
            # self.cy3_list = np.append(self.updated3[1], self.cy3_list)
       

        pred_x1, pred_y1 = int(predicted1[0][0]), int(predicted1[1][0])   
        upd_x1, upd_y1 = int(self.updated1[0]), int(self.updated1[1])

        pred_x2, pred_y2 = int(predicted2[0][0]), int(predicted2[1][0])   
        upd_x2, upd_y2 = int(self.updated2[0]), int(self.updated2[1])

        pred_x3, pred_y3 = int(predicted3[0][0]), int(predicted3[1][0])   
        upd_x3, upd_y3 = int(self.updated3[0]), int(self.updated3[1])


        # joint4
        cx4 = feature[3][0]
        cy4 = feature[3][1]

        self.cx4_list = np.append(cx4, self.cx4_list)
        self.cy4_list = np.append(cy4, self.cy4_list)

        dx4 = self.cx4_list[0]-self.cx4_list[1]
        dy4 = self.cy4_list[0]-self.cy4_list[1]

        vx4 = dx4
        vy4 = dy4

        measured4 = np.array([[np.float32(cx4)], [np.float32(cy4)], [np.float32(vx4)], [np.float32(vy4)]])

        predicted4 = kf4.predict()

        if self.marker_flag[3] == True:
            self.updated4 = np.asarray(kf4.update(measured4))
            # print("Updated when True", updated)
            self.measured4_list = np.append(self.updated4, self.measured4_list) 
        elif self.marker_flag[3] == False:
            # rospy.sleep(2)
            new_measured4 = np.array([[np.float32(self.measured4_list[0])], [np.float32(self.measured4_list[1])], [np.float32(self.measured4_list[2])], [np.float32(self.measured4_list[3])]])
            self.updated4 = np.asarray(kf4.update(new_measured4))
            self.measured4_list = np.append(self.updated4, self.measured4_list)
            # self.cx4_list = np.append(self.updated4[0], self.cx4_list)
            # self.cy4_list = np.append(self.updated4[1], self.cy4_list)

        # joint5
        cx5 = feature[4][0]
        cy5 = feature[4][1]

        self.cx5_list = np.append(cx5, self.cx5_list)
        self.cy5_list = np.append(cy5, self.cy5_list)

        dx5 = self.cx5_list[0]-self.cx5_list[1]
        dy5 = self.cy5_list[0]-self.cy5_list[1]

        vx5 = dx5
        vy5 = dy5

        measured5 = np.array([[np.float32(cx5)], [np.float32(cy5)], [np.float32(vx5)], [np.float32(vy5)]])

        predicted5 = kf5.predict()

        if self.marker_flag[4] == True:
            self.updated5 = np.asarray(kf5.update(measured5))
            # print("Updated when True", updated)
            self.measured5_list = np.append(self.updated5, self.measured5_list) 
        elif self.marker_flag[4] == False:
            print("is update function getting called for 5th point")
            print("update output before False", self.measured5_list[0], self.measured5_list[1], self.measured5_list[2], self.measured5_list[3])
            # rospy.sleep(2)
            new_measured5 = np.array([[np.float32(self.measured5_list[0])], [np.float32(self.measured5_list[1])], [np.float32(self.measured5_list[2])], [np.float32(self.measured5_list[3])]])
            self.updated5 = np.asarray(kf5.update(new_measured5))
            self.measured5_list = np.append(self.updated5, self.measured5_list)
            # self.cx5_list = np.append(self.updated5[0], self.cx5_list)
            # self.cy5_list = np.append(self.updated5[1], self.cy5_list)

        # joint6
        cx6 = feature[5][0]
        cy6 = feature[5][1]

        self.cx6_list = np.append(cx6, self.cx6_list)
        self.cy6_list = np.append(cy6, self.cy6_list)

        dx6 = self.cx6_list[0]-self.cx6_list[1]
        dy6 = self.cy6_list[0]-self.cy6_list[1]

        vx6 = dx6
        vy6 = dy6

        measured6 = np.array([[np.float32(cx6)], [np.float32(cy6)], [np.float32(vx6)], [np.float32(vy6)]])

        predicted6 = kf6.predict()

        if self.marker_flag[5] == True:            
            self.updated6 = np.asarray(kf6.update(measured6))
            # print("Updated when True", updated)
            self.measured6_list = np.append(self.updated6, self.measured6_list) 
        elif self.marker_flag[5] == False:
            print("is update function getting called for 6th point")
            print("update output before False", self.measured6_list[0], self.measured6_list[1], self.measured6_list[2], self.measured6_list[3])
            # rospy.sleep(2)
            new_measured6 = np.array([[np.float32(self.measured6_list[0])], [np.float32(self.measured6_list[1])], [np.float32(self.measured6_list[2])], [np.float32(self.measured6_list[3])]])
            print("new measured 6", new_measured6)
            self.updated6 = np.asarray(kf6.update(new_measured6))
            self.measured6_list = np.append(self.updated6, self.measured6_list)
            # self.cx6_list = np.append(self.updated6[0], self.cx6_list)
            # self.cy6_list = np.append(self.updated6[1], self.cy6_list)

        pred_x4, pred_y4 = int(predicted4[0][0]), int(predicted4[1][0])   
        upd_x4, upd_y4 = int(self.updated4[0]), int(self.updated4[1])

        pred_x5, pred_y5 = int(predicted5[0][0]), int(predicted5[1][0])   
        upd_x5, upd_y5 = int(self.updated5[0]), int(self.updated5[1])

        pred_x6, pred_y6 = int(predicted6[0][0]), int(predicted6[1][0])   
        upd_x6, upd_y6 = int(self.updated6[0]), int(self.updated6[1])


        # joint7
        cx7 = feature[6][0]
        cy7 = feature[6][1]

        self.cx7_list = np.append(cx7, self.cx7_list)
        self.cy7_list = np.append(cy7, self.cy7_list)

        dx7 = self.cx7_list[0]-self.cx7_list[1]
        dy7 = self.cy7_list[0]-self.cy7_list[1]

        vx7 = dx7
        vy7 = dy7

        measured7 = np.array([[np.float32(cx7)], [np.float32(cy7)], [np.float32(vx7)], [np.float32(vy7)]])

        predicted7 = kf7.predict()


        if self.marker_flag[6] == True:
            self.updated7 = np.asarray(kf7.update(measured7))
            # print("Updated when True", updated)
            self.measured7_list = np.append(self.updated7, self.measured7_list) 
        elif self.marker_flag[6] == False:
            print("is update function getting called for 7th point")
            print("update output before False", self.measured7_list[0], self.measured7_list[1], self.measured7_list[2], self.measured7_list[3])
            # rospy.sleep(2)
            new_measured7 = np.array([[np.float32(self.measured7_list[0])], [np.float32(self.measured7_list[1])], [np.float32(self.measured7_list[2])], [np.float32(self.measured7_list[3])]])
            self.updated7 = np.asarray(kf7.update(new_measured7))
            self.measured7_list = np.append(self.updated7, self.measured7_list)
            # self.cx7_list = np.append(self.updated7[0], self.cx7_list)
            # self.cy7_list = np.append(self.updated7[1], self.cy7_list)

        pred_x7, pred_y7 = int(predicted7[0][0]), int(predicted7[1][0])   
        upd_x7, upd_y7 = int(self.updated7[0]), int(self.updated7[1])

        font = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 0.4
        org1 = (10, 20)
        org2 = (10,80)
        org3 = (10, 100)
        org4 = (10,140)
        org5 = (10, 160)
        org6 = (10, 180)

        cv2.circle(img, (int(cx1), int(cy1)), 5, blue, -1)        
        cv2.circle(img, (int(cx2), int(cy2)), 5, blue, -1)
        cv2.circle(img, (int(cx3), int(cy3)), 5, blue, -1)
        cv2.circle(img, (int(cx4), int(cy4)), 5, blue, -1)        
        cv2.circle(img, (int(cx5), int(cy5)), 5, blue, -1)
        cv2.circle(img, (int(cx6), int(cy6)), 5, blue, -1)
        cv2.circle(img, (int(cx7), int(cy7)), 5, blue, -1)


        # cv2.circle(img, (pred_x1, pred_y1), 10, yellow, 4)
        cv2.circle(img, (upd_x1, upd_y1), 5, red, 4)

        # cv2.circle(img, (pred_x2, pred_y2), 10, yellow, 4)
        cv2.circle(img, (upd_x2, upd_y2), 5, red, 4)

        # cv2.circle(img, (pred_x3, pred_y3), 10, yellow, 4)
        cv2.circle(img, (upd_x3, upd_y3), 5, red, 4)


        # cv2.circle(img, (pred_x4, pred_y4), 10, yellow, 4)
        cv2.circle(img, (upd_x4, upd_y4), 5, red, 4)

        # cv2.circle(img, (pred_x5, pred_y5), 10, yellow, 4)
        cv2.circle(img, (upd_x5, upd_y5), 5, red, 4)

        # cv2.circle(img, (pred_x6, pred_y6), 10, yellow, 4)
        cv2.circle(img, (upd_x6, upd_y6), 5, red, 4)

        # cv2.circle(img, (pred_x7, pred_y7), 10, yellow, 4)
        cv2.circle(img, (upd_x7, upd_y7), 5, red, 4)

        cv2.putText(img, str(measured5), org1, font, fontScale, green, 1, cv2.LINE_AA)
        cv2.putText(img, str(measured6), org2, font, fontScale, red, 1, cv2.LINE_AA)
        cv2.putText(img, str(measured7), org3, font, fontScale, yellow, 1, cv2.LINE_AA)

        cv2.putText(img, str(self.updated5), org4, font, fontScale, green, 2, cv2.LINE_AA)
        cv2.putText(img, str(self.updated6), org5, font, fontScale, red, 2, cv2.LINE_AA)
        cv2.putText(img, str(self.updated7), org6, font, fontScale, yellow, 2, cv2.LINE_AA)
        # # cv2.circle(img, (int(cx1), int(cy1)), 5, green, -1)
        # cv2.putText(img, str(predicted5[0:4]), org2, font, fontScale, yellow, 1, cv2.LINE_AA)
        # cv2.putText(img, str(predicted5[4:]), org3, font, fontScale, yellow, 1, cv2.LINE_AA)
        # # cv2.circle(img, (pred_x, pred_y), 10, yellow, 4)
        # cv2.putText(img, str(self.updated5[0:4]), org4, font, fontScale, red, 1, cv2.LINE_AA)
        # cv2.putText(img, str(self.updated5[4:]), org5, font, fontScale, red, 1, cv2.LINE_AA)
        # cv2.circle(img, (upd_x, upd_y), 5, red, 4)

        print("next frame"+str(self.j))

        cv2.imwrite("/home/merlab/Pictures/kalman_images/"+str(self.j)+".jpg", img)                 

        self.j= self.j + 1

        latest_corrected_x = np.array([[self.updated1[0][0], self.updated1[1][0]], [self.updated2[0][0], self.updated2[1][0]], [self.updated3[0][0], self.updated3[1][0]], 
                                        [self.updated4[0][0], self.updated4[1][0]], [self.updated5[0][0], self.updated5[1][0]], [self.updated6[0][0], self.updated6[1][0]], 
                                        [self.updated7[0][0], self.updated7[1][0]]])

        return latest_corrected_x
    
    # runs only once, when all 7 joints are identified for the first time
    def first_input_estimate(self, img, key_points):
        print("first filter is called")
        # print("Actual first first key points", key_points)
        self.marker_flag = [True, True, True, True, True, True, True]
        first_corrected_x = dream_ros.dream_kalman_estimate(img, key_points)        
        # print("First Corrected X", first_corrected_x)
        return first_corrected_x        
    
    # this is the function where the missing joints are identified for 
    def input_estimation(self, img, key_points, corrected_x):
        print("filters are called")        
        # print("key points input after the first iteration", key_points)
        # print("corredtec_x after first iteration", corrected_x)

        latest_x = np.array([[-1, -1], [-1,-1], [-1, -1], 
                     [-1, -1], [-1, -1], [-1, -1], [-1, -1]])
        # if len(key_points) == 7 and (155 < self.i < 150):        
        if len(key_points) == 7 and (150 < self.i < 161):
            print("key point 7 but some missing")
            key_points[-1] = [-1, -1]
            key_points[-2] = [-1, -1]
            key_points[-3] = [-1, -1]
            self.marker_flag = [True, True, True, True, False, False, False]
            latest_corrected_x = dream_ros.dream_kalman_estimate(img, key_points)

        elif len(key_points) == 7 and (180 < self.i < 191):
            print("key point 7 but some missing")
            key_points[-1] = [-1, -1]
            key_points[-2] = [-1, -1]
            key_points[-3] = [-1, -1]
            self.marker_flag = [True, True, True, True, False, False, False]
            latest_corrected_x = dream_ros.dream_kalman_estimate(img, key_points)
        
        elif len(key_points) == 7:
            self.marker_flag = [True, True, True, True, True, True, True]
            latest_corrected_x = dream_ros.dream_kalman_estimate(img, key_points)

        else:
            for i in range(len(key_points)):
                distances = np.sqrt((corrected_x[:, 0] - key_points[i][0])**2 + (corrected_x[:, 1] - key_points[i][1])**2)
                nearest_index = np.argmin(distances)
                latest_x[nearest_index] = key_points[i]
                # print("input after the first estimation", latest_x)

            for i in range(len(latest_x)):
                if ([-1,-1] == latest_x[i]).all():
                    # print(([-1,-1] == latest_x[i]).all())
                    self.marker_flag[i] = False
                    # print("marker_flag array", self.marker_flag)
            latest_corrected_x = dream_ros.dream_kalman_estimate(img, latest_x)
            # print("marker_flag array when length less than 7", self.marker_flag)

        # print("latest corrected X in input estimation", latest_corrected_x)
        # rospy.sleep(3)
        # self.k = self.k + 1
        return latest_corrected_x

    
    # image subscriber callback function to generate key_points
    def kp_from_image(self, image):
        font = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 0.4
        org1 = (10, 20)
        org2 = (10,80)
        org3 = (10, 100)
        org4 = (10,140)
        org5 = (10, 160)        
        blue = (255,0,0)
        yellow = (0,255,255)
        print("Service Started")
        if image is not None:
            # rospy.sleep(20)
            self.control_flag = True
            self.cv_image = self.bridge.imgmsg_to_cv2(image, "rgb8") 
            cv2.imwrite("/home/merlab/Pictures/labeled"+str(self.i)+".jpg", self.cv_image)            
            img1 = copy.deepcopy(self.cv_image)
            # cv2.imwrite("/home/merlab/Pictures/orig"+str(self.i)+".jpg", img1)
            img2 = copy.deepcopy(self.cv_image)
            # cv2.imwrite("/home/merlab/Pictures/orig"+str(self.i)+".jpg", img2)
            key_points = self.key_points
            kp_x = []
            kp_y = []
            for i in range(len(key_points)):
                x = np.int64(key_points[i][0])
                y = np.int64(key_points[i][1])
                kp_x.append(x)
                kp_y.append(y)
                cv2.circle(img1,(x, y),10,blue,-1)           

            # print("Key points x and y", kp_x, kp_y)

            ## This part is to use the key points as constarint feature for skeleton and also process the depth image
            # Depth image processing

            # get the key points as a service message
            kp = []
            for i in range(len(kp_x)):
               kp.append(kp_x[i]) 
               kp.append(kp_y[i])
            kp = np.reshape(np.array(kp), (-1, 2))
            # print("length of keypoints", len(kp))     

            cv2.putText(img1, str(kp[-1]), org1, font, fontScale, yellow, 2, cv2.LINE_AA)
            cv2.putText(img1, str(kp[-2]), org2, font, fontScale, yellow, 2, cv2.LINE_AA)
            cv2.putText(img1, str(kp[-3]), org3, font, fontScale, yellow, 2, cv2.LINE_AA)

            cv2.imwrite("/home/merlab/Pictures/dream_kalman_images/"+str(self.i)+".jpg", img1)

            if len(kp) == 7 and not self.executed:
                # print("first key points input", kp)
                self.corrected_x = dream_ros.first_input_estimate(img2, kp)
                self.executed = True
                # print("Corrected_X", self.corrected_x)                       
            
            elif self.executed:
                # print("Executed?", self.executed)               
                self.corrected_x = dream_ros.input_estimation(img2, kp, self.corrected_x) 

                       
            

            self.i = self.i + 1

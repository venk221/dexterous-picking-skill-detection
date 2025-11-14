#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime
import numpy as np
import pyrealsense2 as rs
import threading  

class CameraRecorder:
    def __init__(self):
        self.bridge = CvBridge()
        self.recording = False
        self.video_writer = None
        self.output_folder = os.path.join(os.path.expanduser("~"), "mer_lab/ros_ws/src/projects/dexterous_picking/final_videos")
        self.ensure_folder_exists(self.output_folder)
        self.grasp_id = None
        self.pipeline = None
        self.recording_thread = None  

    def set_grasp_id(self, grasp_id):
        self.grasp_id = grasp_id

    def ensure_folder_exists(self, folder):
        if not os.path.exists(folder):
            os.makedirs(folder)

    def start_recording(self):
        if self.grasp_id is not None and not self.recording:
            self.recording_thread = threading.Thread(target=self._record_video)
            self.recording_thread.start()

    def stop_recording(self):
        self.recording = False

    def _record_video(self):
        self.recording = True
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        video_filename = os.path.join(self.output_folder, f"video_grasp_{self.grasp_id}_{timestamp}.avi")
        fourcc = cv2.VideoWriter_fourcc(*"XVID")

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)  # Set resolution to 1920x1080

        self.pipeline.start(config)

        try:
            self.video_writer = cv2.VideoWriter(video_filename, fourcc, 30.0, (1920, 1080))
            rospy.loginfo("Recording started.")

            while self.recording:
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()

                if not color_frame:
                    continue

                color_image = np.asanyarray(color_frame.get_data())

                self.video_writer.write(color_image)

        except Exception as e:
            rospy.logerr(f"Error while recording: {str(e)}")

        finally:
            self.pipeline.stop()
            if self.video_writer:
                self.video_writer.release()
            rospy.loginfo("Recording stopped.")

if __name__ == '__main__':
    rospy.init_node('camera_recorder_node')
    recorder = CameraRecorder()

    recorder.set_grasp_id(1)  
    recorder.start_recording()

    rospy.sleep(10)  
    recorder.stop_recording()

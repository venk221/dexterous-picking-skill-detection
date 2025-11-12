#!/usr/bin/env python

import rospy
import cv2
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetModelState
import rospkg
from geometry_msgs.msg import Pose
import numpy as np
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_matrix
from cv_bridge import CvBridge

OBJECTS = {
    # Object name: [URDF file path, object ID, [[diagonal corner point 1], [diagonal corner point 2]], [6 region grasp skill code]]
    "racquetball": ["057_racquetball/racquetball.urdf", 1, [[0.9,0.14,1.05], [1.37,0.66,1.05]], [3, 3, 3, 5, 5, 5]],
    "plate": ["029_plate/plate.urdf", 2, [[0.9,0.14,1.05], [1.37,0.66,1.05]], [5, 5, 5, 3, 3, 3]],
    "chips_can": ["001_chips_can/chips_can.urdf", 5, [[0.9,0.14,1.05], [1.37,0.66,1.05]], [3, 3, 3, 5, 5, 5]],
    "mug": ["025_mug/mug.urdf", 3, [[0.9,0.14,1.05], [1.37,0.66,1.05]], [3, 3, 3, 5, 5, 5]],
}
TABLE_CORNER_POINTS = [[0.75,0,1], [1.5,0.8,1]]
POINT_GEN_DIST = 0.1
EXPERIMENT_ID = 1
DATASET_PATH = "/home/robotics/Desktop/dataset"

class DataSetGenerator:
    def __init__(self):
        # Get package path
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path("dexterous_picking")
        self.bridge = CvBridge()

        # Get start counter number
        cnt_start = input("Enter start counter number: ")
        self.counter = int(cnt_start)

        # Initialize variables
        self.camera_k_matrix = None
        self.depth_image = None

        # Initialize tf2 listener
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Subscribe to camera info and depth image topics
        self.sub_camera_info = rospy.Subscriber("/camera/zed2/depth/camera_info", CameraInfo, self.camera_info_callback)
        self.sub_depth_image = rospy.Subscriber("/camera/zed2/depth/depth_registered", Image, self.depth_image_callback)
        rospy.wait_for_message("/camera/zed2/depth/camera_info", CameraInfo)
        rospy.wait_for_message("/camera/zed2/depth/depth_registered", Image)

    def spawn_urdf_model(self, model_name, point):
        # Create pose message
        pose = Pose()
        pose.position.x = point[0]
        pose.position.y = point[1]
        pose.position.z = point[2]
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1
        # Spawn model
        rospy.wait_for_service("/gazebo/spawn_urdf_model")
        urdf_path = self.package_path + "/description/ycb/" + OBJECTS[model_name][0]
        try:
            spawn_model = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
            with open(urdf_path, "r") as urdf_file:
                urdf_content = urdf_file.read()
            spawn_model(model_name, urdf_content, "", pose, "world")
            rospy.loginfo("Spawned model: %s", model_name)
        except rospy.ServiceException as e:
            rospy.logerr("Spawn service call failed: %s", e)

    def delete_model(self, model_name):
        rospy.wait_for_service("/gazebo/delete_model")
        try:
            delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
            delete_model(model_name)
            rospy.loginfo("Deleted model: %s", model_name)
        except rospy.ServiceException as e:
            rospy.logerr("Delete service call failed: %s", e)

    def points_generator(self, diagonal_corner_points):
        # Generate uniform points in the bounding box
        x_min = diagonal_corner_points[0][0]
        x_max = diagonal_corner_points[1][0]
        y_min = diagonal_corner_points[0][1]
        y_max = diagonal_corner_points[1][1]
        z = diagonal_corner_points[0][2]
        x_points = np.linspace(x_min, x_max, num=int((x_max - x_min) / POINT_GEN_DIST)+1)
        y_points = np.linspace(y_min, y_max, num=int((y_max - y_min) / POINT_GEN_DIST)+1)
        points = np.array(np.meshgrid(x_points, y_points, [z])).T.reshape(-1, 3)
        return points
    
    def get_grasp_skill_code(self, model_name, point):
        # Create 6 region diagonal corner points
        x_min = TABLE_CORNER_POINTS[0][0]
        x_max = TABLE_CORNER_POINTS[1][0]
        y_min = TABLE_CORNER_POINTS[0][1]
        y_max = TABLE_CORNER_POINTS[1][1]
        x_points = np.linspace(x_min, x_max, num=4)
        y_points = np.linspace(y_min, y_max, num=3)
        zone1 = [[x_points[0],y_points[0]], [x_points[1],y_points[1]]]
        zone2 = [[x_points[1],y_points[0]], [x_points[2],y_points[1]]]
        zone3 = [[x_points[2],y_points[0]], [x_points[3],y_points[1]]]
        zone4 = [[x_points[0],y_points[1]], [x_points[1],y_points[2]]]
        zone5 = [[x_points[1],y_points[1]], [x_points[2],y_points[2]]]
        zone6 = [[x_points[2],y_points[1]], [x_points[3],y_points[2]]]
        zones = [zone1, zone2, zone3, zone4, zone5, zone6]
        # Get index of the zone that the point is in
        zone_index = None
        for i in range(len(zones)):
            if point[0] >= zones[i][0][0] and point[0] <= zones[i][1][0] and point[1] >= zones[i][0][1] and point[1] <= zones[i][1][1]:
                zone_index = i
                break
        if zone_index == None:
            rospy.logerr("Point is not in any zone")
            return None
        # Get grasp skill code
        grasp_skill_code = OBJECTS[model_name][3][zone_index]
        return grasp_skill_code, zone_index

    def generate_triangle_world(self, model_name, object_location, grasp_skill_code, zone_index):
        pass

    
    def get_model_pose(self, model_name):
        rospy.wait_for_service("/gazebo/get_model_state")
        try:
            get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
            model_state = get_model_state(model_name, "world")
            point = [model_state.pose.position.x, model_state.pose.position.y, model_state.pose.position.z]
            return point
        except rospy.ServiceException as e:
            rospy.logerr("Get model state service call failed: %s", e)
            return None
    
    def world2px_cam(self, pos_world):
        rospy.wait_for_message("/camera/zed2/depth/camera_info", CameraInfo)
        # Convert world coordinates to camera coordinates
        try:
            trans = self.tf_buffer.lookup_transform('zed2_left_camera_optical_frame', 'world', rospy.Time(), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Failed to lookup transform from world to camera")
            return None
        # Create pose stamped message
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "world"
        pose_stamped.pose.position.x = pos_world[0]
        pose_stamped.pose.position.y = pos_world[1]
        pose_stamped.pose.position.z = pos_world[2]
        pose_stamped.pose.orientation.x = 0
        pose_stamped.pose.orientation.y = 0
        pose_stamped.pose.orientation.z = 0
        pose_stamped.pose.orientation.w = 1
        # Transform pose stamped message
        camera_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, trans)
        # Convert camera coordinates to pixel coordinates
        camera_point = np.array([camera_pose.pose.position.x / camera_pose.pose.position.z, camera_pose.pose.position.y /  camera_pose.pose.position.z, 1])
        pos_px = np.matmul(self.camera_k_matrix, camera_point)
        pos_px = pos_px[:2]
        return pos_px
    
    def publish_img_pixel(self, pos_px):
        # Publishes the image with the pixel marked with a red dot
        img = self.depth_image.copy()
        cv2.circle(img, (int(pos_px[0]), int(pos_px[1])), 5, (0, 0, 255), -1)
        # Resize image
        scale_percent = 50
        width = int(img.shape[1] * scale_percent / 100)
        height = int(img.shape[0] * scale_percent / 100)
        img = cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)
        cv2.imshow("Depth Image", img)
        cv2.waitKey(0)
    
    def camera_info_callback(self, msg):
        self.camera_k_matrix = np.array(msg.K).reshape((3, 3))

    def depth_image_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        depth_array = np.array(depth_image, dtype=np.float32)
        depth_array[np.isnan(depth_array)] = 0
        cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
        depth_array = np.abs(depth_array*255)
        depth_array = np.array(depth_array, dtype=np.uint8)
        depth_array = np.invert(depth_array)
        self.depth_image = depth_array

    def save_data(self, depth_image, annotation, model_name):
        if EXPERIMENT_ID == 1:
            folder_name = "/OneObject"
        elif EXPERIMENT_ID == 2:
            folder_name = "/TwoObjects"
        elif EXPERIMENT_ID == 3:
            folder_name = "/ThreeObjects"
        # Save depth image
        depth_image_path = DATASET_PATH + folder_name + "/depth/" + str(EXPERIMENT_ID) + "_" + str(OBJECTS[model_name][1]) + "_" + str(self.counter) + "_depth.png"
        cv2.imwrite(depth_image_path, depth_image)
        # Save annotation
        annotation_path = DATASET_PATH + folder_name + "/annotation/" + str(EXPERIMENT_ID) + "_" + str(OBJECTS[model_name][1]) + "_" + str(self.counter) + ".txt"
        with open(annotation_path, "w") as annotation_file:
            annotation_file.write(str(annotation))
        

if __name__ == "__main__":
    rospy.init_node("spawn_urdf_model_node")
    dataset_generator = DataSetGenerator()
    pose_world = [0.9,0.14,1.05]
    pose_px = dataset_generator.world2px_cam(pose_world)
    print(pose_px)
    dataset_generator.publish_img_pixel(pose_px)
    # model_name = "plate"
    # points = dataset_generator.points_generator(OBJECTS[model_name][2])
    # for point in points:
    #     dataset_generator.spawn_urdf_model(model_name, point)
    #     rospy.sleep(2)
    #     model_pose = dataset_generator.get_model_pose(model_name)
    #     grasp_skill_code = dataset_generator.get_grasp_skill_code(model_name, model_pose)
    #     print(grasp_skill_code)
    #     dataset_generator.delete_model(model_name)
    #     rospy.sleep(2)

    rospy.spin()
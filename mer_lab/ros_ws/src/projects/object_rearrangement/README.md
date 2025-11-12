This package contains various ROS nodes which work in conjugation with the Unity Environment which can be found in UnityEnvironment folder outside the ros_ws folder. It consists of ur3_moveit package which contains various sceneUnderstanding nodes which controls the way the manipulator will push the pile of objects.
### Using this package
Inorder to use this package, please make sure you all the dependencies installed. Incase of any missing dependencies please run the following command in your catkin_ws outside the src folder.
```
rosdep install --from-paths --ignore-src ros_ws -y
```
Use normal building procedures to build the package. 
###Starting the simulation
1. Start UnityHub by running the following command in the terminal (please make sure you are in the correct folder where you have stored the AppImage)
```
./UnityHub.AppImage 
```
2. Add a new project by adding the Unity folder in Unity Hub. After this is done. Open the project.
3. Launch the start.launch file by using:
```
 roslaunch ur3_moveit start.launch
```
This launch file will launch move_group node, server_endpoint node which will help in connecting with Unity ROS Nodes, and robot_state_publisher. 
4. Run the motionplanning node using:
```
rosrun ur3_moveit moveit_planning_node.py 
```
This node takes in start and end poses (with same orientation/Quaternions of end effector) and plans the robot trajectories so that the end effector traverses in a straight line from start pose to end pose.

5. Run the PointCloud Service using:
```
rosrun ur3_moveit pointCloudService.py
```
This node provides communicates with the unity side of ROS node to get a depth map of the scene and then convert this data into world frame of the robot. Since unity uses Left hand coordinate system and ROS uses Right hand coordinate system, this node performs a set of fixed transformation to transform unity's data in left hand coordinate to ROS's right hand coordinate frame. **Please do not move the position of cameras in unity environment. If you move the camera positions, you'll have to change these fixed transformations to get correct pointcloud**

6. Run the simulation in the unity by clicking on the play button
7. Run one of the following sceneunderstanding nodes based on your needs:
```
rosrun ur3_moveit sceneUnderstanding
rosrun ur3_moveit sceneUnderstanding_random
rosrun ur3_moveit sceneUnderstanding_educated
rosrun ur3_moveit sceneUnderstanding_pca
```
Here,
1. sceneUnderstanding node will plan the trajectory such that the robot will sweep once in horizontal and once in vertical direction passing through the center of the scene.
2. sceneUnderstanding_random will choose start and end poses randomly from a list of 8 pairs of start and end poses. These poses are also in horizontal and vertical direction but not passing through origin.
3. sceneUnderstanding_educated will plan trajectory in horizontal and vertical direction passing through the point where the height of pile is maximum.
4. sceneUnderstanding_pca will plan trajectory based on the mean and eigen vector found after performing PCA.

The way all sceneUnderstanding node works is that it calls pointCloudService to get point cloud, processes it (saves the original received cloud, removes the table, calculates mean, performs Principal component analysis etc), assigns start and end poses, sends a request to moveit_planning node to compute the trajectories and then sends request to MoveRobot Service to the unity node to move the robot in unity.

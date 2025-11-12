# Unity-ROS-Environment
1. Install [Unity Hub](https://docs.unity3d.com/2020.1/Documentation/Manual/GettingStartedInstallingHub.html)
2. Install Unity Version 2020.3.37f1 or 2020.3.*
3. Run UnityHub by running the following command in the terminal:
```
./UnityHub.AppImage
```
4. Add a new project by adding the Unity folder in Unity Hub. After this is done. Open the project.
5. Once the Unity Environment is loaded, go to project tab in the unity window to open Assets/Scenes/ObjectRearrangementScene.unity as shown in figure below:

  <p align="center">
    <img src="images/img1.png" width=900 height=300/>
  </p>
  
6. Installing Packages in Unity Environment:
  - From the top menu bar, open Window -> Package Manager. As the name suggests, the Package Manager is where you can download new packages, update or remove existing ones, and access a variety of information and additional actions for each package.
  - Click on the + sign at the top-left corner of the Package Manager window and then choose the option Add package from git URL....
    - Install [ROS TCP Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector)
    - Install [Unity Robotics Visualizations](https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.visualizations)
    - Install [URDF importer](https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer)
  - Enter the package address and click Add.
  After the installation, the package manager will look something like this:
  
  <p align="center">
    <img src="images/img2.png" width=600 height=300/>
  </p>
  
7. To setup ROS settings, go to Robotics -> ROS Settings. You can set up various settings as shown in the figure below:

  <p align="center">
    <img src="images/img3.png" width=500 height=300/>
  </p>
  
  **Make sure that the ROS IP address is same as the one set in /ur3_moveit/config/params.yaml**
  
8. Click on Play button to start the simulation (Before you run the simulation, please make sure to run the launch file "start.launch" which can be found inside **ur3_moveit** packaage)

Sequence of files created
1.	Kinmodeltest – To verify kinematic model of robot
2.	Controlpos – Discontinuous control. Module moves in steps towards target uses encoder feedback
3.	Controlvel – Continuous control. Module moves continuously towards target without using encoder feedback
4.	Controlvelm2 – New Jacobian extending to marker centre introduced. Also has codes utilizing target point moving towards final target. Code for Sliding Mode Control present as well.
5.	Controlvelm2web – Codes using webcam reading instead of OpenMV cam.

Most files have shared codes. In general, subsequent files share codes of previous files in addition to new codes. 
Purpose of individual codes in Controlvelm2 given below.
Codes for interfacing with OpenMV camera
•	Cameracom – To start reading from OpenMV camera.
•	readSerialData – Called every iteration by cameracom. Processes data published in serial terminal by camera.
•	processData – Converts data in more easy to read from. Returns data in the form of an array containing ID and details of the Apriltags found in image as well as the timestamp.
•	Initialize – Run only during beginning of the experiment. To store initial values wrt which all experiment data will be recorded.
•	Track – Reading current data
•	Findbase – Just an auxiliary function run once used to calculate parameters to be used by Transform function.
•	Calcpos – Used in Kinmodeltest file originally to calculate marker position predicted by kinematic model.
•	Transform – Transforming current position such that origin is now base of robot.
•	Tendonlen – Calculating Tendon lengths using position and angle of marker.
•	Tendonlen2 – Calculating Tendon lengths using only position of marker (Using new inverse kinematics).

Codes for interfacing operating module
•	cableLength2MotorCommand, motorCommandConvert – Used for running module
•	operate2 – Main code for initializing and running module with position commands
.
Codes for control
•	Follou3 – Base code for continuous velocity control
•	folvel_ptr1 – Code for transferring points in such a way that target point is shifted towards final target as marker approaches intermediate target. Never tested due to folvel_ptr2 working well.
•	folvel_ptr2 – Code for moving target points along line joining target and initial position.
•	folvel_smc – Code using sliding model control.
•	Jac – Original Jacobian
•	Jac_m2 – Modified Jacobian which extends till marker position.
•	Jac_m2cont – Same as Jac_m2 with different parameters.

Purpose of some extra individual codes in Controlvelm2web given below. Some of the control codes here have some functions modified to enable reading from the webcam
•	Initwebcam – Run only during beginning of the experiment. To store initial values wrt which all experiment data will be recorded.
•	Webcamread - Reading current data


folvel_ptr2, folvel_smc and Follou3 in Controlvelm2 are commented for better understanding. Other control codes can be understood using these commented codes.


HOW TO run sliding mode control

- Open Controlvelm2web folder
- Open operate2.m
	- Run program
	- Run last section with G = [-10 -10 -10] or other suitable value
- Open another Matlab window which will be used for operations relating to camera
- Open Initwebcam.m
	- Run program
	- If setup changed, change resolution variable res and run program again to save.
- Measure length of actuator cable. Run Findbase function with 2 inputs - length of actuator cable, distance from EE to marker center (currently 37).
- Enter values of outputs basex and basey in the loop in webcamread.m
- Run webcamread.m. Now feedback is available.
- Run folvel_smc in the first window with some target position.
- Reset everything after each control loop test.





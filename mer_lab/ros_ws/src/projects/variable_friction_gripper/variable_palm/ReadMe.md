This Package is dedicated to variable palm width based variable friction gripper. This package is written in python.

## File Info

- ```VF_hand.py``` contains the class used to define the motor nodes and low level dynamixel motor services.
- ```VF_controller_functions.py``` contains the high level services like sliding objects, setting friction state, moving finger.
- ```autocalib.py``` contains calibration function for the hand
- ```pose_estimator.py``` contains method to identify pose of the fingers w.r.t. base using AR Tags
- ```pubsub.py``` If you want to create ROS pub subs, please use the class (or improve upon it to incporporate multiple msg types)
- ```demo.py``` runs a demo sequence on an object (preferrably a small cube)
- ```camera_calibration.py``` follows Zhang's autocalib approach using openCV
- ```register_dict.py``` and ```syncRW_XMHandler.py``` handle communication with the motor

## How to Setup VF Hand

Here is the link to check the wiring:

https://docs.google.com/document/d/1IFK4YY8Vx4xYBYpQJpT-LhC3eLctukeXtCATvK6wI7o/edit

Please check all the wiring, make sure all actuator converter lights on


## How to run:

Removed dependency on ROS to run the code.

Use the following commands to run:

1. Navigate to variable palm/scripts using

  ```
  cd mer_lab/ros_ws/src/projects/variable_friction_gripper/variable_palm/scripts
  ```

2. Check the ports that the USBs are connected to using

    ```ls /dev/ttyUSB```
    
    *Note* Don't press enter, press tab to see the options. Identify USB ports.
    example : ....



    Example:

      ![alt text](utils/usb_select.png)
      
   
    #### OR
                                  
    If you have multiple USBs conneted, please use Dynamixel app to identify the USB ports. Run ```Dynamixel Wizard``` and Click on ```Scan``` to identify USB ports

    *NOTE: please close Dynamixel Wizard after identifying the port Ids, before running next steps*
  
3. Run autocalib to check if you are able to access the VF Hand or not (Use the permutation that works)

    ```
    python3 autoCalib.py --hand_port $HAND_PORT --friction_port $FRICTION_PORT
    ```
    Example (referring the image above):

    ```python3 autoCalib.py --hand_port /dev/ttyUSB2 --friction_port /dev/ttyUSB3```
    
    Note: If does not work, swap ttyUSB2 and ttyUSB3

    ```python3 autoCalib.py --hand_port /dev/ttyUSB3 --friction_port /dev/ttyUSB2```
    

4. Update the Identified USB ports in the following command. Similar to step3.

    ```
    python3 demo.py --hand_port $HAND_PORT --friction_port $FRICTION_PORT
    ```
    
    ```python3 demo.py --hand_port /dev/ttyUSB2 --friction_port /dev/ttyUSB3```

    
   

### Files to Read:

#### All the Meeting notes + Presentations
Under Teams --> Dexterous in-hand manipulation --> Files --> Fall 2022 Direct Research_Trip_Gao

#### Most updated CAD files
Under Teams --> Dexterous in-hand manipulation --> Files --> Fall 2022 Direct Research CAD Files

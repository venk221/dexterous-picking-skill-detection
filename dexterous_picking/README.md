# Dexterous Picking
This project investigates the use of a novel approach to pick objects from a cluttered environment using the franka emika panda robot and Model O dexterous hand. The approach is based on detecting the objects and figuring out the best grasping primitive to use for each object. Then the robot executes the grasping primitive and picks the object.

## Setup Instructions:
In order to setup the project, follow the instructions below:
1. Clone the repository

        git clone git@github.com:berkcalli/mer_lab.git

2. Install the dependencies

        cd mer_lab
        rosdep install --from-paths --ignore-src ros_ws -y

3. Build the project

        catkin build dexterous_picking

## Run Instructions:
In order to run the project, follow the instructions below:
1. Launch the project

        roslaunch dexterous_picking dexterous_picking_demo.launch
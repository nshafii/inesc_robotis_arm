# INESC ROBOTIS Package

This repository prepares ROS functionalities to work with Robotis manipulator_h in active control mode.
The driver can be found in the package INESC_Robotis_Driver. This driver prepares the basic functionalities including geometric based inverse and forward kinematics, trajectory planning and some controlling commands. 

This driver is developed based on the ROBOTIS_Framework:
https://github.com/ROBOTIS-GIT/ROBOTIS-Framework

and initially forked from the following repository: 
https://github.com/catppinto/robotnik_arm


## For installing the package

Prerequisite, ros-indigo packages:

1. sudo apt-get install ros-indigo-desktop-full
2. sudo apt-get install ros-indigo-qt-build
3. sudo apt-get install ros-indigo-moveit-full

Building the package:
In the root directory, catkin_make -j1


## For running the driver

source devel/setup.bash

roslaunch INESC_Robotis_Driver robot_init.launch

The ROS services provided by the driver:

setToolPose (Move the tool position to a desired pose using trajectory planning and IK)

homeArm (Move the robot to home position)

stopArm (Stop the robot)

getToolPos (Forward Kinematics)

getJointsValuse (Return the joint space values) 

The more information about the services can be found in the ReadMe of the driver directory



#### For visualizing the robot (optional)

rosrun rviz rvis 

#### For using a GUI to command the robot (optional)

rosrun cat_manipulator_gui cat_manipulator_gui 






### INESC ROBOTIS arm driver

This package contains the driver sources and the implementation of the services can be found in GoToPose_service.cpp.

Different services has been provided, including:

1- /GoToPose_service/getJointValues

Get the current joints values in radians
         
2- /GoToPose_service/getToolPos

Get the pose of the end effectors in Cartesian position related to the base of the arm. The end-effectors rotation in Euler angle (Roll, Pitch, Yaw) in ZYX canonical Euler sequence.

3- /GoToPose_service/setToolPose

Command the trajectory planning and Inverse lnverse kinematics to move the arm to the asked pose with the input arguments:

* pos_x : X position to the base in Meter

* pos_y: Y position to the base in Meter

* pos_z: Z position to the base in Meter

* Yaw_angle_z: Yaw Euler angle in radian for ZYZ rotation

* float64 Pitch_angle_y: Pitch Euler angle in radian for ZYZ rotation

* float64 Roll_angle_x: Roll Euler angle in radian for ZYZ rotation

* Linear_velocity: The translational speed of the end effectors during the trajecotry execution (can be in the range of 0 m/s to 0.12 m/s ) 

4- /GoToPose_service/stopArm : Pause the arm 

5- /GoToPose_service/homeArm : Move the arm to the the home position where all joints angles are Zero radian (the highest manipulability)

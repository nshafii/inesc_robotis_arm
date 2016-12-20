The impelimetntaiton of the serivces provided by the driver can be found in GoToPose_service.cpp.

The are differnt services has been prepared, including:

1 - /GoToPose_service/getJointValues (to get joints values in radian)
         
2 - /GoToPose_service/getToolPos 
to get the pose of the end effector. Cartesian Position realted to base of the Atm. and End-effector rotation in Euler angle (Roll, Pitch, Yaw) in ZYX canonical Euler sequence.            


3- /GoToPose_service/setToolPose  
to Command the thetrajectory planning and Inverse lnverse kinematics to move the arm of the robot to the asked pose:
pos_x : X postion to the base in Meter
pos_y: Y postion to the base in Meter
pos_z: Z postion to the base in Meter
Yaw_angle_z: Yaw euler angle in radian for ZYZ rotation
float64 Pitch_angle_y: Pitch euler angle in radian for ZYZ rotation
float64 Roll_angle_x: Roll euler angle in radian for ZYZ rotation
linear_velocity: The tranlational speed of the end effector during the trajecotry execution (can be in the renge of 0 m/s to 0.12 m/s )


4- /GoToPose_service/stopArm 
Pause the arm 
               
5- /GoToPose_service/homeArm                
Go the the home position of the arm (all joints angle in Zero radian, highest maniupablity)

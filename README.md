# robotnik_arm

obs: source devel/setup.bash

** Init robot and move it **

roscore

roslaunch cat_move_to_target robot_init.launch 

rosrun cat_manipulator_ik_gui cat_manipulator_ik_gui 

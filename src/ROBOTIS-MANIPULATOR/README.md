# ROBOTIS-MANIPULATOR

15/12/10 robotis_manipulator_h_description  uploaded


  URDF model added
  

15/12/10 robotis_manipulator_h_gazebo   uploaded 


  Gazebo simulation added


15/12/17 robotis_manipulator_h_moveit, uploaded


  Moveit setup added


15/12/24 robotis_manipulator_h_calc, robotis_manipulator_gui uploaded


  moveit_node, kinematics_node & gui for kinematics added
         
         
------------------------------------------------------------------------------------

moveit_node

joint space planning using moveit default gui in Rviz (ex. RRT)


    1. for simulation gazebo

      1-1. roslaunch robotis_manipulator_h_gazebo robotis_world.launch
  
      1-2. roslaunch robotis_manipulator_h_moveit moveit_gazebo.launch


          please add industrial_core before execution 
    
          https://github.com/ros-industrial/industrial_core?files=1


      1-3. rosrun robotis_manipulator_h_calc moveit_node _sim:=true
  

    2. ror real robot execution
  
      2-1. roslaunch robotis_manipulator_h_moveit moveit_demo.launch 


          please add industrial_core before execution 
    
          https://github.com/ros-industrial/industrial_core?files=1


      2-2. roslaunch robotis_controller robotis_manager.launch
  

          from ROBOTIS-Framework  
  
          https://github.com/ROBOTIS-GIT/ROBOTIS-Framework/wiki

  
      2-3. rosrun robotis_manipulator_h_calc moveit_node
      
------------------------------------------------------------------------------------

kinematics_node

forward kinematics and inverse kinematics example from moveit interface with user gui


    1. for simulation gazebo

      1-1. roslaunch robotis_manipulator_h_gazebo robotis_world.launch
  
      1-2. roslaunch robotis_manipulator_h_moveit moveit_gazebo.launch


          please add industrial_core before execution 
    
          https://github.com/ros-industrial/industrial_core?files=1


      1-3. rosrun robotis_manipulator_h_calc kinematics_node _sim:=true
      
      1-4. rosrun robotis_manipulator_h_gui robotis_manipulator_h_gui
  

    2. ror real robot execution
  
      2-1. roslaunch robotis_manipulator_h_moveit moveit_demo.launch 


          please add industrial_core before execution 
    
          https://github.com/ros-industrial/industrial_core?files=1


      2-2. roslaunch robotis_controller robotis_manager.launch
  

          from ROBOTIS-Framework  
  
          https://github.com/ROBOTIS-GIT/ROBOTIS-Framework/wiki

  
      2-3. rosrun robotis_manipulator_h_calc kinematics_node
      
      2-4. rosrun robotis_manipulator_h_gui robotis_manipulator_h_gui
      
------------------------------------------------------------------------------------

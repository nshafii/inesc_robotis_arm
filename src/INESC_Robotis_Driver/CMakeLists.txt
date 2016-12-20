cmake_minimum_required(VERSION 2.8.3)
project(INESC_Robotis_Driver)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	rospy
	roscpp
	tf
    tf2_ros
	geometry_msgs
	std_msgs
	pluginlib 
	cmake_modules
	genmsg
    message_generation
)

find_package(Boost REQUIRED system filesystem date_time thread)

add_service_files(FILES GetJointValues.srv SetToolPose.srv Stop.srv HomeArm.srv GetToolPose.srv)

generate_messages(DEPENDENCIES std_msgs)

#Build talker and listener
include_directories(include 
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS}) 

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES INESC_Robotis_Driver
  CATKIN_DEPENDS message_runtime
 )

add_library(INESC_Robotis_Driver
src/IK_Solver.cpp
src/GoToPose_service.cpp
src/TrajectoryGenerator.cpp)

target_link_libraries(INESC_Robotis_Driver ${catkin_LIBRARIES})

## Declare a C++ executable



add_executable(GoToPose_service src/GoToPose_service.cpp src/IK_Solver.cpp src/TrajectoryGenerator.cpp)
target_link_libraries(GoToPose_service ${catkin_LIBRARIES})
add_dependencies(GoToPose_service INESC_Robotis_Driver_gencpp)

#add_executable(gotoxyz_client src/gotoxyz_client.cpp)
#target_link_libraries(gotoxyz_client ${catkin_LIBRARIES})
#add_dependencies(gotoxyz_client INESC_Robotis_Driver_gencpp)



## Specify libraries to link a library or executable target against



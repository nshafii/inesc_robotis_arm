cmake_minimum_required(VERSION 2.8.3)
project(inesc_3_finger_gripper_driver)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
	rospy
	roscpp
	geometry_msgs
	std_msgs
	pluginlib 
	cmake_modules
	genmsg
    message_generation
)

find_package(Boost REQUIRED system filesystem date_time thread)

add_service_files(FILES Close.srv Open.srv Stop.srv)

generate_messages(DEPENDENCIES std_msgs)

#Build talker and listener
include_directories(include 
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS}) 

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES inesc_3_finger_gripper_driver
  CATKIN_DEPENDS message_runtime
 )

add_library(inesc_3_finger_gripper_driver
src/Gripper_driver.cpp
)

target_link_libraries(inesc_3_finger_gripper_driver ${catkin_LIBRARIES})

## Declare a C++ executable



add_executable(Gripper_driver src/Gripper_driver.cpp)
target_link_libraries(Gripper_driver ${catkin_LIBRARIES})
add_dependencies(Gripper_driver inesc_3_finger_gripper_driver_gencpp)



## Specify libraries to link a library or executable target against



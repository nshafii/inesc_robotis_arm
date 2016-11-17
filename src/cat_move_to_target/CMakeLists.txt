cmake_minimum_required(VERSION 2.8.3)
project(cat_move_to_target)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	rospy
	roscpp
	cat_common
	apriltags
	tf
        tf2_ros
	geometry_msgs
	std_msgs
	moveit_core 
	moveit_ros_planning 
	moveit_ros_planning_interface 
	pluginlib 
	cmake_modules
	genmsg
        message_generation
)
find_package(Boost REQUIRED system filesystem date_time thread)

add_service_files(FILES GetJointValues.srv GetTagPose.srv)

generate_messages(DEPENDENCIES std_msgs)

#Build talker and listener
include_directories(include 
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS}) 

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cat_move_to_target
  CATKIN_DEPENDS message_runtime
 #   roscpp
 #   geometry_msgs
 #   tf
 #   tf2_ros
 #   moveit_core
 #   moveit_ros_planning_interface
 #   interactive_markers
)

add_library(cat_move_to_target
src/ApriltagsClass.cpp
src/SimpleTag.cpp
src/RobotState.cpp
src/TwoDotFiveDVS.cpp
src/gotoxyz_service.cpp)

target_link_libraries(cat_move_to_target ${catkin_LIBRARIES})

## Declare a C++ executable

add_executable(cat_move_manipulator src/move_to_target.cpp  src/pick.cpp src/RobotState.cpp)
target_link_libraries(cat_move_manipulator
   ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(cat_move_manipulator cat_move_to_target_gencpp)

add_executable(cat_detect_tags src/detect_tags.cpp src/ApriltagsClass.cpp)
 target_link_libraries(cat_detect_tags
   ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(cat_detect_tags cat_move_to_target_gencpp)

add_executable(cat_ik_manipulator src/ik_manipulator.cpp src/RobotState.cpp)
target_link_libraries(cat_ik_manipulator
   ${catkin_LIBRARIES})
add_dependencies(cat_ik_manipulator cat_move_to_target_gencpp)

add_executable(gettagpose_service src/gettagpose_service.cpp src/ApriltagsClass.cpp)
target_link_libraries(gettagpose_service ${catkin_LIBRARIES})
add_dependencies(gettagpose_service cat_move_to_target_gencpp)

add_executable(gotoxyz_service src/gotoxyz_service.cpp src/RobotState.cpp)
target_link_libraries(gotoxyz_service ${catkin_LIBRARIES})
add_dependencies(gotoxyz_service cat_move_to_target_gencpp)

add_executable(gotoxyz_client src/gotoxyz_client.cpp)
target_link_libraries(gotoxyz_client ${catkin_LIBRARIES})
add_dependencies(gotoxyz_client cat_move_to_target_gencpp)



## Specify libraries to link a library or executable target against



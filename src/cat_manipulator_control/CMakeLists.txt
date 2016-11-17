cmake_minimum_required(VERSION 2.8.3)
project(cat_manipulator_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  control_msgs
  robotis_controller_msgs
  sensor_msgs
  std_msgs
  geometry_msgs
  moveit_msgs
  moveit_core 
  moveit_ros_planning 
  moveit_ros_planning_interface
  pluginlib 
  cmake_modules
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(Eigen REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robotis_manipulator_h_calc
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  roscpp
  control_msgs 
robotis_controller_msgs
  sensor_msgs 
  std_msgs 
  geometry_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
  interactive_markers
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)


add_executable(robotis_moveit_node 
	src/main_moveit.cpp
	src/base/callback.cpp
	src/base/jointstate.cpp
	src/base/process.cpp
	src/base/trajectory.cpp
)

add_executable(robotis_kinematics_node 
	src/main_kinematics.cpp
	src/base/callback.cpp
	src/base/jointstate.cpp
	src/base/process.cpp
	src/base/trajectory.cpp
)

add_executable(my_kinematics_node_gazebo
        src/my_main_kinematics_gazebo.cpp
        src/base/my_callback.cpp
        src/base/jointstate.cpp
        src/base/process.cpp
        src/base/trajectory.cpp
)

add_executable(my_kinematics_node
        src/my_main_kinematics.cpp
        src/base/my_callback.cpp
        src/base/jointstate.cpp
        src/base/process.cpp
        src/base/trajectory.cpp
)

add_executable(my_init_robot
        src/my_init_robot.cpp
)

add_executable(my_init_robot_gazebo
        src/my_init_robot_gazebo.cpp
)

add_executable(kin_tests
        src/kin_tests.cpp
)


add_dependencies(robotis_moveit_node main_generate_messages_cpp)
add_dependencies(robotis_kinematics_node main_generate_messages_cpp)
add_dependencies(my_kinematics_node main_generate_messages_cpp)
add_dependencies(my_kinematics_node_gazebo main_generate_messages_cpp)
add_dependencies(my_init_robot main_generate_messages_cpp)
add_dependencies(my_init_robot_gazebo main_generate_messages_cpp)
add_dependencies(kin_tests main_generate_messages_cpp)


target_link_libraries(robotis_moveit_node
  ${catkin_LIBRARIES}
  ${ROSRT_PLATFORM_LDFLAGS}
)

target_link_libraries(robotis_kinematics_node
  ${catkin_LIBRARIES}
  ${ROSRT_PLATFORM_LDFLAGS}
)

target_link_libraries(my_kinematics_node_gazebo
  ${catkin_LIBRARIES}
  ${ROSRT_PLATFORM_LDFLAGS}
)

target_link_libraries(my_kinematics_node
  ${catkin_LIBRARIES}
  ${ROSRT_PLATFORM_LDFLAGS}
)

target_link_libraries(my_init_robot
  ${catkin_LIBRARIES}
  ${ROSRT_PLATFORM_LDFLAGS}
)

target_link_libraries(my_init_robot_gazebo
  ${catkin_LIBRARIES}
  ${ROSRT_PLATFORM_LDFLAGS}
)

target_link_libraries(kin_tests
  ${catkin_LIBRARIES}
  ${ROSRT_PLATFORM_LDFLAGS}
)




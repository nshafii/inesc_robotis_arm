cmake_minimum_required(VERSION 2.8.3)
project(robotis_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  robotis_controller_msgs
  message_generation
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   ControlWrite.msg
#   ControlTorque.msg
#   PublishPosition.msg
#    Message1.msg
#    Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   dynamixel_msgs#   sensor_msgs#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES robotis_controller
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(robotis_controller
  src/RobotisController.cpp
  src/device/GenericDevice.cpp
  src/handler/PortHandler.cpp
  src/handler/GroupHandler.cpp
  src/handler/PacketHandler.cpp
  src/handler/PacketProtocol1.cpp
  src/handler/PacketProtocol2.cpp
)

## Declare a cpp executable
add_executable(robotis_manager
  src/RobotisController.cpp
  src/device/GenericDevice.cpp
  src/handler/PortHandler.cpp
  src/handler/GroupHandler.cpp
  src/handler/PacketHandler.cpp
  src/handler/PacketProtocol1.cpp
  src/handler/PacketProtocol2.cpp
  src/RobotisManager.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(robotis_manager robotis_manager_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(robotis_controller
  yaml-cpp
  ${catkin_LIBRARIES}
)
target_link_libraries(robotis_manager
  ${catkin_LIBRARIES}
)

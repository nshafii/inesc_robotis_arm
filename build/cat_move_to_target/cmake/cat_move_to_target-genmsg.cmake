# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "cat_move_to_target: 0 messages, 2 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(cat_move_to_target_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/cat/robotnik_arm/src/cat_move_to_target/srv/GetTagPose.srv" NAME_WE)
add_custom_target(_cat_move_to_target_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cat_move_to_target" "/home/cat/robotnik_arm/src/cat_move_to_target/srv/GetTagPose.srv" ""
)

get_filename_component(_filename "/home/cat/robotnik_arm/src/cat_move_to_target/srv/GetJointValues.srv" NAME_WE)
add_custom_target(_cat_move_to_target_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cat_move_to_target" "/home/cat/robotnik_arm/src/cat_move_to_target/srv/GetJointValues.srv" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(cat_move_to_target
  "/home/cat/robotnik_arm/src/cat_move_to_target/srv/GetTagPose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cat_move_to_target
)
_generate_srv_cpp(cat_move_to_target
  "/home/cat/robotnik_arm/src/cat_move_to_target/srv/GetJointValues.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cat_move_to_target
)

### Generating Module File
_generate_module_cpp(cat_move_to_target
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cat_move_to_target
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(cat_move_to_target_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(cat_move_to_target_generate_messages cat_move_to_target_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cat/robotnik_arm/src/cat_move_to_target/srv/GetTagPose.srv" NAME_WE)
add_dependencies(cat_move_to_target_generate_messages_cpp _cat_move_to_target_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cat/robotnik_arm/src/cat_move_to_target/srv/GetJointValues.srv" NAME_WE)
add_dependencies(cat_move_to_target_generate_messages_cpp _cat_move_to_target_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cat_move_to_target_gencpp)
add_dependencies(cat_move_to_target_gencpp cat_move_to_target_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cat_move_to_target_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(cat_move_to_target
  "/home/cat/robotnik_arm/src/cat_move_to_target/srv/GetTagPose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cat_move_to_target
)
_generate_srv_lisp(cat_move_to_target
  "/home/cat/robotnik_arm/src/cat_move_to_target/srv/GetJointValues.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cat_move_to_target
)

### Generating Module File
_generate_module_lisp(cat_move_to_target
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cat_move_to_target
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(cat_move_to_target_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(cat_move_to_target_generate_messages cat_move_to_target_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cat/robotnik_arm/src/cat_move_to_target/srv/GetTagPose.srv" NAME_WE)
add_dependencies(cat_move_to_target_generate_messages_lisp _cat_move_to_target_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cat/robotnik_arm/src/cat_move_to_target/srv/GetJointValues.srv" NAME_WE)
add_dependencies(cat_move_to_target_generate_messages_lisp _cat_move_to_target_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cat_move_to_target_genlisp)
add_dependencies(cat_move_to_target_genlisp cat_move_to_target_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cat_move_to_target_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(cat_move_to_target
  "/home/cat/robotnik_arm/src/cat_move_to_target/srv/GetTagPose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cat_move_to_target
)
_generate_srv_py(cat_move_to_target
  "/home/cat/robotnik_arm/src/cat_move_to_target/srv/GetJointValues.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cat_move_to_target
)

### Generating Module File
_generate_module_py(cat_move_to_target
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cat_move_to_target
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(cat_move_to_target_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(cat_move_to_target_generate_messages cat_move_to_target_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cat/robotnik_arm/src/cat_move_to_target/srv/GetTagPose.srv" NAME_WE)
add_dependencies(cat_move_to_target_generate_messages_py _cat_move_to_target_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cat/robotnik_arm/src/cat_move_to_target/srv/GetJointValues.srv" NAME_WE)
add_dependencies(cat_move_to_target_generate_messages_py _cat_move_to_target_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cat_move_to_target_genpy)
add_dependencies(cat_move_to_target_genpy cat_move_to_target_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cat_move_to_target_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cat_move_to_target)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cat_move_to_target
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(cat_move_to_target_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cat_move_to_target)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cat_move_to_target
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(cat_move_to_target_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cat_move_to_target)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cat_move_to_target\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cat_move_to_target
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(cat_move_to_target_generate_messages_py std_msgs_generate_messages_py)

# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/cat/robotnik_arm/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cat/robotnik_arm/build

# Include any dependencies generated for this target.
include cat_manipulator_control/CMakeFiles/my_init_robot.dir/depend.make

# Include the progress variables for this target.
include cat_manipulator_control/CMakeFiles/my_init_robot.dir/progress.make

# Include the compile flags for this target's objects.
include cat_manipulator_control/CMakeFiles/my_init_robot.dir/flags.make

cat_manipulator_control/CMakeFiles/my_init_robot.dir/src/my_init_robot.cpp.o: cat_manipulator_control/CMakeFiles/my_init_robot.dir/flags.make
cat_manipulator_control/CMakeFiles/my_init_robot.dir/src/my_init_robot.cpp.o: /home/cat/robotnik_arm/src/cat_manipulator_control/src/my_init_robot.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/cat/robotnik_arm/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object cat_manipulator_control/CMakeFiles/my_init_robot.dir/src/my_init_robot.cpp.o"
	cd /home/cat/robotnik_arm/build/cat_manipulator_control && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/my_init_robot.dir/src/my_init_robot.cpp.o -c /home/cat/robotnik_arm/src/cat_manipulator_control/src/my_init_robot.cpp

cat_manipulator_control/CMakeFiles/my_init_robot.dir/src/my_init_robot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/my_init_robot.dir/src/my_init_robot.cpp.i"
	cd /home/cat/robotnik_arm/build/cat_manipulator_control && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/cat/robotnik_arm/src/cat_manipulator_control/src/my_init_robot.cpp > CMakeFiles/my_init_robot.dir/src/my_init_robot.cpp.i

cat_manipulator_control/CMakeFiles/my_init_robot.dir/src/my_init_robot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/my_init_robot.dir/src/my_init_robot.cpp.s"
	cd /home/cat/robotnik_arm/build/cat_manipulator_control && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/cat/robotnik_arm/src/cat_manipulator_control/src/my_init_robot.cpp -o CMakeFiles/my_init_robot.dir/src/my_init_robot.cpp.s

cat_manipulator_control/CMakeFiles/my_init_robot.dir/src/my_init_robot.cpp.o.requires:
.PHONY : cat_manipulator_control/CMakeFiles/my_init_robot.dir/src/my_init_robot.cpp.o.requires

cat_manipulator_control/CMakeFiles/my_init_robot.dir/src/my_init_robot.cpp.o.provides: cat_manipulator_control/CMakeFiles/my_init_robot.dir/src/my_init_robot.cpp.o.requires
	$(MAKE) -f cat_manipulator_control/CMakeFiles/my_init_robot.dir/build.make cat_manipulator_control/CMakeFiles/my_init_robot.dir/src/my_init_robot.cpp.o.provides.build
.PHONY : cat_manipulator_control/CMakeFiles/my_init_robot.dir/src/my_init_robot.cpp.o.provides

cat_manipulator_control/CMakeFiles/my_init_robot.dir/src/my_init_robot.cpp.o.provides.build: cat_manipulator_control/CMakeFiles/my_init_robot.dir/src/my_init_robot.cpp.o

# Object files for target my_init_robot
my_init_robot_OBJECTS = \
"CMakeFiles/my_init_robot.dir/src/my_init_robot.cpp.o"

# External object files for target my_init_robot
my_init_robot_EXTERNAL_OBJECTS =

/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: cat_manipulator_control/CMakeFiles/my_init_robot.dir/src/my_init_robot.cpp.o
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: cat_manipulator_control/CMakeFiles/my_init_robot.dir/build.make
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmoveit_common_planning_interface_objects.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmoveit_planning_scene_interface.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmoveit_move_group_interface.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmoveit_warehouse.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libwarehouse_ros.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmoveit_pick_place_planner.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmoveit_move_group_capabilities_base.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmoveit_rdf_loader.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmoveit_kinematics_plugin_loader.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmoveit_robot_model_loader.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmoveit_constraint_sampler_manager_loader.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmoveit_planning_pipeline.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmoveit_trajectory_execution_manager.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmoveit_plan_execution.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmoveit_planning_scene_monitor.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmoveit_lazy_free_space_updater.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmoveit_point_containment_filter.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmoveit_occupancy_map_monitor.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmoveit_pointcloud_octomap_updater_core.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmoveit_semantic_world.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmoveit_exceptions.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmoveit_background_processing.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmoveit_kinematics_base.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmoveit_robot_model.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmoveit_transforms.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmoveit_robot_state.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmoveit_robot_trajectory.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmoveit_planning_interface.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmoveit_collision_detection.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmoveit_collision_detection_fcl.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmoveit_kinematic_constraints.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmoveit_planning_scene.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmoveit_constraint_samplers.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmoveit_planning_request_adapter.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmoveit_profiler.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmoveit_trajectory_processing.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmoveit_distance_field.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmoveit_kinematics_metrics.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmoveit_dynamics_solver.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libgeometric_shapes.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/liboctomap.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/liboctomath.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libeigen_conversions.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/librandom_numbers.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libkdl_parser.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/liborocos-kdl.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/liborocos-kdl.so.1.3.0
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/liburdf.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /usr/lib/x86_64-linux-gnu/liburdfdom_sensor.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /usr/lib/x86_64-linux-gnu/liburdfdom_model_state.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /usr/lib/x86_64-linux-gnu/liburdfdom_model.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /usr/lib/x86_64-linux-gnu/liburdfdom_world.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/librosconsole_bridge.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libsrdfdom.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libimage_transport.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libmessage_filters.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libroscpp.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libclass_loader.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /usr/lib/libPocoFoundation.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /usr/lib/x86_64-linux-gnu/libdl.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/librosconsole.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /usr/lib/liblog4cxx.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/librostime.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libcpp_common.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: /opt/ros/indigo/lib/libroslib.so
/home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot: cat_manipulator_control/CMakeFiles/my_init_robot.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot"
	cd /home/cat/robotnik_arm/build/cat_manipulator_control && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/my_init_robot.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
cat_manipulator_control/CMakeFiles/my_init_robot.dir/build: /home/cat/robotnik_arm/devel/lib/cat_manipulator_control/my_init_robot
.PHONY : cat_manipulator_control/CMakeFiles/my_init_robot.dir/build

cat_manipulator_control/CMakeFiles/my_init_robot.dir/requires: cat_manipulator_control/CMakeFiles/my_init_robot.dir/src/my_init_robot.cpp.o.requires
.PHONY : cat_manipulator_control/CMakeFiles/my_init_robot.dir/requires

cat_manipulator_control/CMakeFiles/my_init_robot.dir/clean:
	cd /home/cat/robotnik_arm/build/cat_manipulator_control && $(CMAKE_COMMAND) -P CMakeFiles/my_init_robot.dir/cmake_clean.cmake
.PHONY : cat_manipulator_control/CMakeFiles/my_init_robot.dir/clean

cat_manipulator_control/CMakeFiles/my_init_robot.dir/depend:
	cd /home/cat/robotnik_arm/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cat/robotnik_arm/src /home/cat/robotnik_arm/src/cat_manipulator_control /home/cat/robotnik_arm/build /home/cat/robotnik_arm/build/cat_manipulator_control /home/cat/robotnik_arm/build/cat_manipulator_control/CMakeFiles/my_init_robot.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : cat_manipulator_control/CMakeFiles/my_init_robot.dir/depend


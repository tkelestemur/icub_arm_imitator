# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/tarik/ros_ws/src/icub_ros/iCubSim

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tarik/ros_ws/src/icub_ros/iCubSim/build

# Include any dependencies generated for this target.
include CMakeFiles/jointSub.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/jointSub.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/jointSub.dir/flags.make

CMakeFiles/jointSub.dir/src/jointSub.cpp.o: CMakeFiles/jointSub.dir/flags.make
CMakeFiles/jointSub.dir/src/jointSub.cpp.o: ../src/jointSub.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tarik/ros_ws/src/icub_ros/iCubSim/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/jointSub.dir/src/jointSub.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/jointSub.dir/src/jointSub.cpp.o -c /home/tarik/ros_ws/src/icub_ros/iCubSim/src/jointSub.cpp

CMakeFiles/jointSub.dir/src/jointSub.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/jointSub.dir/src/jointSub.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tarik/ros_ws/src/icub_ros/iCubSim/src/jointSub.cpp > CMakeFiles/jointSub.dir/src/jointSub.cpp.i

CMakeFiles/jointSub.dir/src/jointSub.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/jointSub.dir/src/jointSub.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tarik/ros_ws/src/icub_ros/iCubSim/src/jointSub.cpp -o CMakeFiles/jointSub.dir/src/jointSub.cpp.s

CMakeFiles/jointSub.dir/src/jointSub.cpp.o.requires:

.PHONY : CMakeFiles/jointSub.dir/src/jointSub.cpp.o.requires

CMakeFiles/jointSub.dir/src/jointSub.cpp.o.provides: CMakeFiles/jointSub.dir/src/jointSub.cpp.o.requires
	$(MAKE) -f CMakeFiles/jointSub.dir/build.make CMakeFiles/jointSub.dir/src/jointSub.cpp.o.provides.build
.PHONY : CMakeFiles/jointSub.dir/src/jointSub.cpp.o.provides

CMakeFiles/jointSub.dir/src/jointSub.cpp.o.provides.build: CMakeFiles/jointSub.dir/src/jointSub.cpp.o


# Object files for target jointSub
jointSub_OBJECTS = \
"CMakeFiles/jointSub.dir/src/jointSub.cpp.o"

# External object files for target jointSub
jointSub_EXTERNAL_OBJECTS =

jointSub: CMakeFiles/jointSub.dir/src/jointSub.cpp.o
jointSub: CMakeFiles/jointSub.dir/build.make
jointSub: /usr/local/lib/libYARP_math.so.2.3.65
jointSub: /usr/local/lib/libYARP_dev.so.2.3.65
jointSub: /usr/local/lib/libYARP_init.so.2.3.65
jointSub: /usr/local/lib/libYARP_name.so.2.3.65
jointSub: /usr/local/lib/libYARP_sig.so.2.3.65
jointSub: /usr/local/lib/libYARP_OS.so.2.3.65
jointSub: CMakeFiles/jointSub.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/tarik/ros_ws/src/icub_ros/iCubSim/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable jointSub"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/jointSub.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/jointSub.dir/build: jointSub

.PHONY : CMakeFiles/jointSub.dir/build

CMakeFiles/jointSub.dir/requires: CMakeFiles/jointSub.dir/src/jointSub.cpp.o.requires

.PHONY : CMakeFiles/jointSub.dir/requires

CMakeFiles/jointSub.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/jointSub.dir/cmake_clean.cmake
.PHONY : CMakeFiles/jointSub.dir/clean

CMakeFiles/jointSub.dir/depend:
	cd /home/tarik/ros_ws/src/icub_ros/iCubSim/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tarik/ros_ws/src/icub_ros/iCubSim /home/tarik/ros_ws/src/icub_ros/iCubSim /home/tarik/ros_ws/src/icub_ros/iCubSim/build /home/tarik/ros_ws/src/icub_ros/iCubSim/build /home/tarik/ros_ws/src/icub_ros/iCubSim/build/CMakeFiles/jointSub.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/jointSub.dir/depend

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
CMAKE_SOURCE_DIR = /nfshome/ewf985/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /nfshome/ewf985/catkin_ws/build

# Include any dependencies generated for this target.
include counter/CMakeFiles/counter.dir/depend.make

# Include the progress variables for this target.
include counter/CMakeFiles/counter.dir/progress.make

# Include the compile flags for this target's objects.
include counter/CMakeFiles/counter.dir/flags.make

counter/CMakeFiles/counter.dir/src/publisher.cpp.o: counter/CMakeFiles/counter.dir/flags.make
counter/CMakeFiles/counter.dir/src/publisher.cpp.o: /nfshome/ewf985/catkin_ws/src/counter/src/publisher.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/nfshome/ewf985/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object counter/CMakeFiles/counter.dir/src/publisher.cpp.o"
	cd /nfshome/ewf985/catkin_ws/build/counter && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/counter.dir/src/publisher.cpp.o -c /nfshome/ewf985/catkin_ws/src/counter/src/publisher.cpp

counter/CMakeFiles/counter.dir/src/publisher.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/counter.dir/src/publisher.cpp.i"
	cd /nfshome/ewf985/catkin_ws/build/counter && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /nfshome/ewf985/catkin_ws/src/counter/src/publisher.cpp > CMakeFiles/counter.dir/src/publisher.cpp.i

counter/CMakeFiles/counter.dir/src/publisher.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/counter.dir/src/publisher.cpp.s"
	cd /nfshome/ewf985/catkin_ws/build/counter && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /nfshome/ewf985/catkin_ws/src/counter/src/publisher.cpp -o CMakeFiles/counter.dir/src/publisher.cpp.s

counter/CMakeFiles/counter.dir/src/publisher.cpp.o.requires:

.PHONY : counter/CMakeFiles/counter.dir/src/publisher.cpp.o.requires

counter/CMakeFiles/counter.dir/src/publisher.cpp.o.provides: counter/CMakeFiles/counter.dir/src/publisher.cpp.o.requires
	$(MAKE) -f counter/CMakeFiles/counter.dir/build.make counter/CMakeFiles/counter.dir/src/publisher.cpp.o.provides.build
.PHONY : counter/CMakeFiles/counter.dir/src/publisher.cpp.o.provides

counter/CMakeFiles/counter.dir/src/publisher.cpp.o.provides.build: counter/CMakeFiles/counter.dir/src/publisher.cpp.o


counter/CMakeFiles/counter.dir/src/subscriber.cpp.o: counter/CMakeFiles/counter.dir/flags.make
counter/CMakeFiles/counter.dir/src/subscriber.cpp.o: /nfshome/ewf985/catkin_ws/src/counter/src/subscriber.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/nfshome/ewf985/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object counter/CMakeFiles/counter.dir/src/subscriber.cpp.o"
	cd /nfshome/ewf985/catkin_ws/build/counter && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/counter.dir/src/subscriber.cpp.o -c /nfshome/ewf985/catkin_ws/src/counter/src/subscriber.cpp

counter/CMakeFiles/counter.dir/src/subscriber.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/counter.dir/src/subscriber.cpp.i"
	cd /nfshome/ewf985/catkin_ws/build/counter && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /nfshome/ewf985/catkin_ws/src/counter/src/subscriber.cpp > CMakeFiles/counter.dir/src/subscriber.cpp.i

counter/CMakeFiles/counter.dir/src/subscriber.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/counter.dir/src/subscriber.cpp.s"
	cd /nfshome/ewf985/catkin_ws/build/counter && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /nfshome/ewf985/catkin_ws/src/counter/src/subscriber.cpp -o CMakeFiles/counter.dir/src/subscriber.cpp.s

counter/CMakeFiles/counter.dir/src/subscriber.cpp.o.requires:

.PHONY : counter/CMakeFiles/counter.dir/src/subscriber.cpp.o.requires

counter/CMakeFiles/counter.dir/src/subscriber.cpp.o.provides: counter/CMakeFiles/counter.dir/src/subscriber.cpp.o.requires
	$(MAKE) -f counter/CMakeFiles/counter.dir/build.make counter/CMakeFiles/counter.dir/src/subscriber.cpp.o.provides.build
.PHONY : counter/CMakeFiles/counter.dir/src/subscriber.cpp.o.provides

counter/CMakeFiles/counter.dir/src/subscriber.cpp.o.provides.build: counter/CMakeFiles/counter.dir/src/subscriber.cpp.o


# Object files for target counter
counter_OBJECTS = \
"CMakeFiles/counter.dir/src/publisher.cpp.o" \
"CMakeFiles/counter.dir/src/subscriber.cpp.o"

# External object files for target counter
counter_EXTERNAL_OBJECTS =

/nfshome/ewf985/catkin_ws/devel/lib/counter/counter: counter/CMakeFiles/counter.dir/src/publisher.cpp.o
/nfshome/ewf985/catkin_ws/devel/lib/counter/counter: counter/CMakeFiles/counter.dir/src/subscriber.cpp.o
/nfshome/ewf985/catkin_ws/devel/lib/counter/counter: counter/CMakeFiles/counter.dir/build.make
/nfshome/ewf985/catkin_ws/devel/lib/counter/counter: /opt/ros/kinetic/lib/libroscpp.so
/nfshome/ewf985/catkin_ws/devel/lib/counter/counter: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/nfshome/ewf985/catkin_ws/devel/lib/counter/counter: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/nfshome/ewf985/catkin_ws/devel/lib/counter/counter: /opt/ros/kinetic/lib/librosconsole.so
/nfshome/ewf985/catkin_ws/devel/lib/counter/counter: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/nfshome/ewf985/catkin_ws/devel/lib/counter/counter: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/nfshome/ewf985/catkin_ws/devel/lib/counter/counter: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/nfshome/ewf985/catkin_ws/devel/lib/counter/counter: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/nfshome/ewf985/catkin_ws/devel/lib/counter/counter: /opt/ros/kinetic/lib/libroscpp_serialization.so
/nfshome/ewf985/catkin_ws/devel/lib/counter/counter: /opt/ros/kinetic/lib/libxmlrpcpp.so
/nfshome/ewf985/catkin_ws/devel/lib/counter/counter: /opt/ros/kinetic/lib/librostime.so
/nfshome/ewf985/catkin_ws/devel/lib/counter/counter: /opt/ros/kinetic/lib/libcpp_common.so
/nfshome/ewf985/catkin_ws/devel/lib/counter/counter: /usr/lib/x86_64-linux-gnu/libboost_system.so
/nfshome/ewf985/catkin_ws/devel/lib/counter/counter: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/nfshome/ewf985/catkin_ws/devel/lib/counter/counter: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/nfshome/ewf985/catkin_ws/devel/lib/counter/counter: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/nfshome/ewf985/catkin_ws/devel/lib/counter/counter: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/nfshome/ewf985/catkin_ws/devel/lib/counter/counter: /usr/lib/x86_64-linux-gnu/libpthread.so
/nfshome/ewf985/catkin_ws/devel/lib/counter/counter: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/nfshome/ewf985/catkin_ws/devel/lib/counter/counter: counter/CMakeFiles/counter.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/nfshome/ewf985/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable /nfshome/ewf985/catkin_ws/devel/lib/counter/counter"
	cd /nfshome/ewf985/catkin_ws/build/counter && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/counter.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
counter/CMakeFiles/counter.dir/build: /nfshome/ewf985/catkin_ws/devel/lib/counter/counter

.PHONY : counter/CMakeFiles/counter.dir/build

counter/CMakeFiles/counter.dir/requires: counter/CMakeFiles/counter.dir/src/publisher.cpp.o.requires
counter/CMakeFiles/counter.dir/requires: counter/CMakeFiles/counter.dir/src/subscriber.cpp.o.requires

.PHONY : counter/CMakeFiles/counter.dir/requires

counter/CMakeFiles/counter.dir/clean:
	cd /nfshome/ewf985/catkin_ws/build/counter && $(CMAKE_COMMAND) -P CMakeFiles/counter.dir/cmake_clean.cmake
.PHONY : counter/CMakeFiles/counter.dir/clean

counter/CMakeFiles/counter.dir/depend:
	cd /nfshome/ewf985/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /nfshome/ewf985/catkin_ws/src /nfshome/ewf985/catkin_ws/src/counter /nfshome/ewf985/catkin_ws/build /nfshome/ewf985/catkin_ws/build/counter /nfshome/ewf985/catkin_ws/build/counter/CMakeFiles/counter.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : counter/CMakeFiles/counter.dir/depend


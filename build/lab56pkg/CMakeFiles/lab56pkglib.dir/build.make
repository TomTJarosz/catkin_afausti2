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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/youbot/catkin_afausti2/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/youbot/catkin_afausti2/build

# Include any dependencies generated for this target.
include lab56pkg/CMakeFiles/lab56pkglib.dir/depend.make

# Include the progress variables for this target.
include lab56pkg/CMakeFiles/lab56pkglib.dir/progress.make

# Include the compile flags for this target's objects.
include lab56pkg/CMakeFiles/lab56pkglib.dir/flags.make

lab56pkg/CMakeFiles/lab56pkglib.dir/src/lab56func.cpp.o: lab56pkg/CMakeFiles/lab56pkglib.dir/flags.make
lab56pkg/CMakeFiles/lab56pkglib.dir/src/lab56func.cpp.o: /home/youbot/catkin_afausti2/src/lab56pkg/src/lab56func.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/youbot/catkin_afausti2/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object lab56pkg/CMakeFiles/lab56pkglib.dir/src/lab56func.cpp.o"
	cd /home/youbot/catkin_afausti2/build/lab56pkg && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/lab56pkglib.dir/src/lab56func.cpp.o -c /home/youbot/catkin_afausti2/src/lab56pkg/src/lab56func.cpp

lab56pkg/CMakeFiles/lab56pkglib.dir/src/lab56func.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lab56pkglib.dir/src/lab56func.cpp.i"
	cd /home/youbot/catkin_afausti2/build/lab56pkg && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/youbot/catkin_afausti2/src/lab56pkg/src/lab56func.cpp > CMakeFiles/lab56pkglib.dir/src/lab56func.cpp.i

lab56pkg/CMakeFiles/lab56pkglib.dir/src/lab56func.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lab56pkglib.dir/src/lab56func.cpp.s"
	cd /home/youbot/catkin_afausti2/build/lab56pkg && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/youbot/catkin_afausti2/src/lab56pkg/src/lab56func.cpp -o CMakeFiles/lab56pkglib.dir/src/lab56func.cpp.s

lab56pkg/CMakeFiles/lab56pkglib.dir/src/lab56func.cpp.o.requires:
.PHONY : lab56pkg/CMakeFiles/lab56pkglib.dir/src/lab56func.cpp.o.requires

lab56pkg/CMakeFiles/lab56pkglib.dir/src/lab56func.cpp.o.provides: lab56pkg/CMakeFiles/lab56pkglib.dir/src/lab56func.cpp.o.requires
	$(MAKE) -f lab56pkg/CMakeFiles/lab56pkglib.dir/build.make lab56pkg/CMakeFiles/lab56pkglib.dir/src/lab56func.cpp.o.provides.build
.PHONY : lab56pkg/CMakeFiles/lab56pkglib.dir/src/lab56func.cpp.o.provides

lab56pkg/CMakeFiles/lab56pkglib.dir/src/lab56func.cpp.o.provides.build: lab56pkg/CMakeFiles/lab56pkglib.dir/src/lab56func.cpp.o

# Object files for target lab56pkglib
lab56pkglib_OBJECTS = \
"CMakeFiles/lab56pkglib.dir/src/lab56func.cpp.o"

# External object files for target lab56pkglib
lab56pkglib_EXTERNAL_OBJECTS =

/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: lab56pkg/CMakeFiles/lab56pkglib.dir/src/lab56func.cpp.o
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: lab56pkg/CMakeFiles/lab56pkglib.dir/build.make
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /opt/ros/indigo/lib/libcv_bridge.so
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /usr/lib/i386-linux-gnu/libopencv_videostab.so.2.4.8
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /usr/lib/i386-linux-gnu/libopencv_video.so.2.4.8
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /usr/lib/i386-linux-gnu/libopencv_superres.so.2.4.8
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /usr/lib/i386-linux-gnu/libopencv_stitching.so.2.4.8
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /usr/lib/i386-linux-gnu/libopencv_photo.so.2.4.8
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /usr/lib/i386-linux-gnu/libopencv_ocl.so.2.4.8
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /usr/lib/i386-linux-gnu/libopencv_objdetect.so.2.4.8
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /usr/lib/i386-linux-gnu/libopencv_ml.so.2.4.8
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /usr/lib/i386-linux-gnu/libopencv_legacy.so.2.4.8
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /usr/lib/i386-linux-gnu/libopencv_imgproc.so.2.4.8
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /usr/lib/i386-linux-gnu/libopencv_highgui.so.2.4.8
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /usr/lib/i386-linux-gnu/libopencv_gpu.so.2.4.8
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /usr/lib/i386-linux-gnu/libopencv_flann.so.2.4.8
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /usr/lib/i386-linux-gnu/libopencv_features2d.so.2.4.8
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /usr/lib/i386-linux-gnu/libopencv_core.so.2.4.8
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /usr/lib/i386-linux-gnu/libopencv_contrib.so.2.4.8
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /usr/lib/i386-linux-gnu/libopencv_calib3d.so.2.4.8
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /opt/ros/indigo/lib/libimage_transport.so
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /opt/ros/indigo/lib/libmessage_filters.so
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /usr/lib/i386-linux-gnu/libtinyxml.so
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /opt/ros/indigo/lib/libclass_loader.so
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /usr/lib/libPocoFoundation.so
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /usr/lib/i386-linux-gnu/libdl.so
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /opt/ros/indigo/lib/libroslib.so
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /home/youbot/catkin_afausti2/devel/lib/liblab4pkglib.so
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /home/youbot/catkin_afausti2/devel/lib/liblab3pkglib.so
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /opt/ros/indigo/lib/libroscpp.so
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /usr/lib/i386-linux-gnu/libboost_signals.so
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /usr/lib/i386-linux-gnu/libboost_filesystem.so
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /opt/ros/indigo/lib/librosconsole.so
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /usr/lib/liblog4cxx.so
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /usr/lib/i386-linux-gnu/libboost_regex.so
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /opt/ros/indigo/lib/librostime.so
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /usr/lib/i386-linux-gnu/libboost_date_time.so
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /opt/ros/indigo/lib/libcpp_common.so
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /usr/lib/i386-linux-gnu/libboost_system.so
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /usr/lib/i386-linux-gnu/libboost_thread.so
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /usr/lib/i386-linux-gnu/libpthread.so
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: /usr/lib/i386-linux-gnu/libconsole_bridge.so
/home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so: lab56pkg/CMakeFiles/lab56pkglib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library /home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so"
	cd /home/youbot/catkin_afausti2/build/lab56pkg && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lab56pkglib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
lab56pkg/CMakeFiles/lab56pkglib.dir/build: /home/youbot/catkin_afausti2/devel/lib/liblab56pkglib.so
.PHONY : lab56pkg/CMakeFiles/lab56pkglib.dir/build

lab56pkg/CMakeFiles/lab56pkglib.dir/requires: lab56pkg/CMakeFiles/lab56pkglib.dir/src/lab56func.cpp.o.requires
.PHONY : lab56pkg/CMakeFiles/lab56pkglib.dir/requires

lab56pkg/CMakeFiles/lab56pkglib.dir/clean:
	cd /home/youbot/catkin_afausti2/build/lab56pkg && $(CMAKE_COMMAND) -P CMakeFiles/lab56pkglib.dir/cmake_clean.cmake
.PHONY : lab56pkg/CMakeFiles/lab56pkglib.dir/clean

lab56pkg/CMakeFiles/lab56pkglib.dir/depend:
	cd /home/youbot/catkin_afausti2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/youbot/catkin_afausti2/src /home/youbot/catkin_afausti2/src/lab56pkg /home/youbot/catkin_afausti2/build /home/youbot/catkin_afausti2/build/lab56pkg /home/youbot/catkin_afausti2/build/lab56pkg/CMakeFiles/lab56pkglib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : lab56pkg/CMakeFiles/lab56pkglib.dir/depend


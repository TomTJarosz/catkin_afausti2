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

# Utility rule file for run_tests_cv_camera_rostest_test_no_yaml.test.

# Include the progress variables for this target.
include drivers/cv_camera/CMakeFiles/run_tests_cv_camera_rostest_test_no_yaml.test.dir/progress.make

drivers/cv_camera/CMakeFiles/run_tests_cv_camera_rostest_test_no_yaml.test:
	cd /home/youbot/catkin_afausti2/build/drivers/cv_camera && ../../catkin_generated/env_cached.sh /usr/bin/python /opt/ros/indigo/share/catkin/cmake/test/run_tests.py /home/youbot/catkin_afausti2/build/test_results/cv_camera/rostest-test_no_yaml.xml /opt/ros/indigo/share/rostest/cmake/../../../bin/rostest\ --pkgdir=/home/youbot/catkin_afausti2/src/drivers/cv_camera\ --package=cv_camera\ --results-filename\ test_no_yaml.xml\ --results-base-dir\ "/home/youbot/catkin_afausti2/build/test_results"\ /home/youbot/catkin_afausti2/src/drivers/cv_camera/test/no_yaml.test\ 

run_tests_cv_camera_rostest_test_no_yaml.test: drivers/cv_camera/CMakeFiles/run_tests_cv_camera_rostest_test_no_yaml.test
run_tests_cv_camera_rostest_test_no_yaml.test: drivers/cv_camera/CMakeFiles/run_tests_cv_camera_rostest_test_no_yaml.test.dir/build.make
.PHONY : run_tests_cv_camera_rostest_test_no_yaml.test

# Rule to build all files generated by this target.
drivers/cv_camera/CMakeFiles/run_tests_cv_camera_rostest_test_no_yaml.test.dir/build: run_tests_cv_camera_rostest_test_no_yaml.test
.PHONY : drivers/cv_camera/CMakeFiles/run_tests_cv_camera_rostest_test_no_yaml.test.dir/build

drivers/cv_camera/CMakeFiles/run_tests_cv_camera_rostest_test_no_yaml.test.dir/clean:
	cd /home/youbot/catkin_afausti2/build/drivers/cv_camera && $(CMAKE_COMMAND) -P CMakeFiles/run_tests_cv_camera_rostest_test_no_yaml.test.dir/cmake_clean.cmake
.PHONY : drivers/cv_camera/CMakeFiles/run_tests_cv_camera_rostest_test_no_yaml.test.dir/clean

drivers/cv_camera/CMakeFiles/run_tests_cv_camera_rostest_test_no_yaml.test.dir/depend:
	cd /home/youbot/catkin_afausti2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/youbot/catkin_afausti2/src /home/youbot/catkin_afausti2/src/drivers/cv_camera /home/youbot/catkin_afausti2/build /home/youbot/catkin_afausti2/build/drivers/cv_camera /home/youbot/catkin_afausti2/build/drivers/cv_camera/CMakeFiles/run_tests_cv_camera_rostest_test_no_yaml.test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : drivers/cv_camera/CMakeFiles/run_tests_cv_camera_rostest_test_no_yaml.test.dir/depend


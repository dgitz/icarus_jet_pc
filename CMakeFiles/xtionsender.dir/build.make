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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/linaro/catkin_ws/src/icarus_jet_pc

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/linaro/catkin_ws/src/icarus_jet_pc

# Include any dependencies generated for this target.
include CMakeFiles/xtionsender.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/xtionsender.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/xtionsender.dir/flags.make

CMakeFiles/xtionsender.dir/nodes/xtionsender.cpp.o: CMakeFiles/xtionsender.dir/flags.make
CMakeFiles/xtionsender.dir/nodes/xtionsender.cpp.o: nodes/xtionsender.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/linaro/catkin_ws/src/icarus_jet_pc/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/xtionsender.dir/nodes/xtionsender.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/xtionsender.dir/nodes/xtionsender.cpp.o -c /home/linaro/catkin_ws/src/icarus_jet_pc/nodes/xtionsender.cpp

CMakeFiles/xtionsender.dir/nodes/xtionsender.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/xtionsender.dir/nodes/xtionsender.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/linaro/catkin_ws/src/icarus_jet_pc/nodes/xtionsender.cpp > CMakeFiles/xtionsender.dir/nodes/xtionsender.cpp.i

CMakeFiles/xtionsender.dir/nodes/xtionsender.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/xtionsender.dir/nodes/xtionsender.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/linaro/catkin_ws/src/icarus_jet_pc/nodes/xtionsender.cpp -o CMakeFiles/xtionsender.dir/nodes/xtionsender.cpp.s

CMakeFiles/xtionsender.dir/nodes/xtionsender.cpp.o.requires:
.PHONY : CMakeFiles/xtionsender.dir/nodes/xtionsender.cpp.o.requires

CMakeFiles/xtionsender.dir/nodes/xtionsender.cpp.o.provides: CMakeFiles/xtionsender.dir/nodes/xtionsender.cpp.o.requires
	$(MAKE) -f CMakeFiles/xtionsender.dir/build.make CMakeFiles/xtionsender.dir/nodes/xtionsender.cpp.o.provides.build
.PHONY : CMakeFiles/xtionsender.dir/nodes/xtionsender.cpp.o.provides

CMakeFiles/xtionsender.dir/nodes/xtionsender.cpp.o.provides.build: CMakeFiles/xtionsender.dir/nodes/xtionsender.cpp.o

# Object files for target xtionsender
xtionsender_OBJECTS = \
"CMakeFiles/xtionsender.dir/nodes/xtionsender.cpp.o"

# External object files for target xtionsender
xtionsender_EXTERNAL_OBJECTS =

devel/lib/icarus_jet_pc/xtionsender: CMakeFiles/xtionsender.dir/nodes/xtionsender.cpp.o
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libroscpp.so
devel/lib/icarus_jet_pc/xtionsender: /usr/lib/arm-linux-gnueabihf/libpthread.so
devel/lib/icarus_jet_pc/xtionsender: /usr/lib/libboost_signals-mt.so
devel/lib/icarus_jet_pc/xtionsender: /usr/lib/libboost_filesystem-mt.so
devel/lib/icarus_jet_pc/xtionsender: /usr/lib/libboost_system-mt.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libcpp_common.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libroscpp_serialization.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/librostime.so
devel/lib/icarus_jet_pc/xtionsender: /usr/lib/libboost_date_time-mt.so
devel/lib/icarus_jet_pc/xtionsender: /usr/lib/libboost_thread-mt.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/librosconsole.so
devel/lib/icarus_jet_pc/xtionsender: /usr/lib/libboost_regex-mt.so
devel/lib/icarus_jet_pc/xtionsender: /usr/lib/liblog4cxx.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libxmlrpcpp.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libcv_bridge.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libopencv_calib3d.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libopencv_contrib.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libopencv_core.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libopencv_features2d.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libopencv_flann.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libopencv_gpu.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libopencv_highgui.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libopencv_imgproc.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libopencv_legacy.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libopencv_ml.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libopencv_nonfree.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libopencv_objdetect.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libopencv_photo.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libopencv_stitching.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libopencv_ts.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libopencv_video.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libopencv_videostab.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libimage_transport.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libmessage_filters.so
devel/lib/icarus_jet_pc/xtionsender: /usr/lib/libtinyxml.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libclass_loader.so
devel/lib/icarus_jet_pc/xtionsender: /usr/lib/libPocoFoundation.so
devel/lib/icarus_jet_pc/xtionsender: /usr/lib/arm-linux-gnueabihf/libdl.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libconsole_bridge.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libroslib.so
devel/lib/icarus_jet_pc/xtionsender: /home/linaro/OpenNI-Linux-Arm-2.2/Redist/libOpenNI2.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libopencv_calib3d.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libopencv_contrib.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libopencv_core.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libopencv_features2d.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libopencv_flann.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libopencv_gpu.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libopencv_highgui.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libopencv_imgproc.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libopencv_legacy.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libopencv_ml.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libopencv_nonfree.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libopencv_objdetect.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libopencv_photo.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libopencv_stitching.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libopencv_video.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libopencv_videostab.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libopencv_ts.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libimage_transport.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libmessage_filters.so
devel/lib/icarus_jet_pc/xtionsender: /usr/lib/libtinyxml.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libclass_loader.so
devel/lib/icarus_jet_pc/xtionsender: /usr/lib/libPocoFoundation.so
devel/lib/icarus_jet_pc/xtionsender: /usr/lib/arm-linux-gnueabihf/libdl.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libconsole_bridge.so
devel/lib/icarus_jet_pc/xtionsender: /opt/ros/groovy/lib/libroslib.so
devel/lib/icarus_jet_pc/xtionsender: /home/linaro/OpenNI-Linux-Arm-2.2/Redist/libOpenNI2.so
devel/lib/icarus_jet_pc/xtionsender: CMakeFiles/xtionsender.dir/build.make
devel/lib/icarus_jet_pc/xtionsender: CMakeFiles/xtionsender.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable devel/lib/icarus_jet_pc/xtionsender"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/xtionsender.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/xtionsender.dir/build: devel/lib/icarus_jet_pc/xtionsender
.PHONY : CMakeFiles/xtionsender.dir/build

CMakeFiles/xtionsender.dir/requires: CMakeFiles/xtionsender.dir/nodes/xtionsender.cpp.o.requires
.PHONY : CMakeFiles/xtionsender.dir/requires

CMakeFiles/xtionsender.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/xtionsender.dir/cmake_clean.cmake
.PHONY : CMakeFiles/xtionsender.dir/clean

CMakeFiles/xtionsender.dir/depend:
	cd /home/linaro/catkin_ws/src/icarus_jet_pc && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/linaro/catkin_ws/src/icarus_jet_pc /home/linaro/catkin_ws/src/icarus_jet_pc /home/linaro/catkin_ws/src/icarus_jet_pc /home/linaro/catkin_ws/src/icarus_jet_pc /home/linaro/catkin_ws/src/icarus_jet_pc/CMakeFiles/xtionsender.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/xtionsender.dir/depend


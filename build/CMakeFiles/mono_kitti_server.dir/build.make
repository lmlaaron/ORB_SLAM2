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
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/rose/dev/ORB_SLAM2_NEW

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rose/dev/ORB_SLAM2_NEW/build

# Include any dependencies generated for this target.
include CMakeFiles/mono_kitti_server.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/mono_kitti_server.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/mono_kitti_server.dir/flags.make

CMakeFiles/mono_kitti_server.dir/Examples/Monocular/mono_kitti_server.cc.o: CMakeFiles/mono_kitti_server.dir/flags.make
CMakeFiles/mono_kitti_server.dir/Examples/Monocular/mono_kitti_server.cc.o: ../Examples/Monocular/mono_kitti_server.cc
	$(CMAKE_COMMAND) -E cmake_progress_report /home/rose/dev/ORB_SLAM2_NEW/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/mono_kitti_server.dir/Examples/Monocular/mono_kitti_server.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/mono_kitti_server.dir/Examples/Monocular/mono_kitti_server.cc.o -c /home/rose/dev/ORB_SLAM2_NEW/Examples/Monocular/mono_kitti_server.cc

CMakeFiles/mono_kitti_server.dir/Examples/Monocular/mono_kitti_server.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mono_kitti_server.dir/Examples/Monocular/mono_kitti_server.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/rose/dev/ORB_SLAM2_NEW/Examples/Monocular/mono_kitti_server.cc > CMakeFiles/mono_kitti_server.dir/Examples/Monocular/mono_kitti_server.cc.i

CMakeFiles/mono_kitti_server.dir/Examples/Monocular/mono_kitti_server.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mono_kitti_server.dir/Examples/Monocular/mono_kitti_server.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/rose/dev/ORB_SLAM2_NEW/Examples/Monocular/mono_kitti_server.cc -o CMakeFiles/mono_kitti_server.dir/Examples/Monocular/mono_kitti_server.cc.s

CMakeFiles/mono_kitti_server.dir/Examples/Monocular/mono_kitti_server.cc.o.requires:
.PHONY : CMakeFiles/mono_kitti_server.dir/Examples/Monocular/mono_kitti_server.cc.o.requires

CMakeFiles/mono_kitti_server.dir/Examples/Monocular/mono_kitti_server.cc.o.provides: CMakeFiles/mono_kitti_server.dir/Examples/Monocular/mono_kitti_server.cc.o.requires
	$(MAKE) -f CMakeFiles/mono_kitti_server.dir/build.make CMakeFiles/mono_kitti_server.dir/Examples/Monocular/mono_kitti_server.cc.o.provides.build
.PHONY : CMakeFiles/mono_kitti_server.dir/Examples/Monocular/mono_kitti_server.cc.o.provides

CMakeFiles/mono_kitti_server.dir/Examples/Monocular/mono_kitti_server.cc.o.provides.build: CMakeFiles/mono_kitti_server.dir/Examples/Monocular/mono_kitti_server.cc.o

# Object files for target mono_kitti_server
mono_kitti_server_OBJECTS = \
"CMakeFiles/mono_kitti_server.dir/Examples/Monocular/mono_kitti_server.cc.o"

# External object files for target mono_kitti_server
mono_kitti_server_EXTERNAL_OBJECTS =

../Examples/Monocular/mono_kitti_server: CMakeFiles/mono_kitti_server.dir/Examples/Monocular/mono_kitti_server.cc.o
../Examples/Monocular/mono_kitti_server: CMakeFiles/mono_kitti_server.dir/build.make
../Examples/Monocular/mono_kitti_server: ../lib/libORB_SLAM2.so
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libopencv_ts.so.2.4.8
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
../Examples/Monocular/mono_kitti_server: /home/rose/Pangolin/build/src/libpangolin.so
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libGLU.so
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libGL.so
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libSM.so
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libICE.so
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libX11.so
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libXext.so
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libGLEW.so
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libSM.so
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libICE.so
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libX11.so
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libXext.so
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libGLEW.so
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libpython2.7.so
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libdc1394.so
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libavcodec.so
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libavformat.so
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libavutil.so
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libswscale.so
../Examples/Monocular/mono_kitti_server: /usr/lib/libOpenNI.so
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libpng.so
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libz.so
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libjpeg.so
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libtiff.so
../Examples/Monocular/mono_kitti_server: /usr/lib/x86_64-linux-gnu/libIlmImf.so
../Examples/Monocular/mono_kitti_server: ../Thirdparty/DBoW2/lib/libDBoW2.so
../Examples/Monocular/mono_kitti_server: ../Thirdparty/g2o/lib/libg2o.so
../Examples/Monocular/mono_kitti_server: /usr/local/lib/libgrpc.so
../Examples/Monocular/mono_kitti_server: /usr/local/lib/libgrpc++.so
../Examples/Monocular/mono_kitti_server: /usr/local/lib/libprotobuf.so
../Examples/Monocular/mono_kitti_server: CMakeFiles/mono_kitti_server.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../Examples/Monocular/mono_kitti_server"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mono_kitti_server.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/mono_kitti_server.dir/build: ../Examples/Monocular/mono_kitti_server
.PHONY : CMakeFiles/mono_kitti_server.dir/build

CMakeFiles/mono_kitti_server.dir/requires: CMakeFiles/mono_kitti_server.dir/Examples/Monocular/mono_kitti_server.cc.o.requires
.PHONY : CMakeFiles/mono_kitti_server.dir/requires

CMakeFiles/mono_kitti_server.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/mono_kitti_server.dir/cmake_clean.cmake
.PHONY : CMakeFiles/mono_kitti_server.dir/clean

CMakeFiles/mono_kitti_server.dir/depend:
	cd /home/rose/dev/ORB_SLAM2_NEW/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rose/dev/ORB_SLAM2_NEW /home/rose/dev/ORB_SLAM2_NEW /home/rose/dev/ORB_SLAM2_NEW/build /home/rose/dev/ORB_SLAM2_NEW/build /home/rose/dev/ORB_SLAM2_NEW/build/CMakeFiles/mono_kitti_server.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/mono_kitti_server.dir/depend


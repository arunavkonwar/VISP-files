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
CMAKE_SOURCE_DIR = /home/arunav/code-new/source

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/arunav/code-new/source

# Include any dependencies generated for this target.
include CMakeFiles/example-executable.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/example-executable.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/example-executable.dir/flags.make

CMakeFiles/example-executable.dir/example.cpp.o: CMakeFiles/example-executable.dir/flags.make
CMakeFiles/example-executable.dir/example.cpp.o: example.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/arunav/code-new/source/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/example-executable.dir/example.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/example-executable.dir/example.cpp.o -c /home/arunav/code-new/source/example.cpp

CMakeFiles/example-executable.dir/example.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/example-executable.dir/example.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/arunav/code-new/source/example.cpp > CMakeFiles/example-executable.dir/example.cpp.i

CMakeFiles/example-executable.dir/example.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/example-executable.dir/example.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/arunav/code-new/source/example.cpp -o CMakeFiles/example-executable.dir/example.cpp.s

CMakeFiles/example-executable.dir/example.cpp.o.requires:

.PHONY : CMakeFiles/example-executable.dir/example.cpp.o.requires

CMakeFiles/example-executable.dir/example.cpp.o.provides: CMakeFiles/example-executable.dir/example.cpp.o.requires
	$(MAKE) -f CMakeFiles/example-executable.dir/build.make CMakeFiles/example-executable.dir/example.cpp.o.provides.build
.PHONY : CMakeFiles/example-executable.dir/example.cpp.o.provides

CMakeFiles/example-executable.dir/example.cpp.o.provides.build: CMakeFiles/example-executable.dir/example.cpp.o


# Object files for target example-executable
example__executable_OBJECTS = \
"CMakeFiles/example-executable.dir/example.cpp.o"

# External object files for target example-executable
example__executable_EXTERNAL_OBJECTS =

example-executable: CMakeFiles/example-executable.dir/example.cpp.o
example-executable: CMakeFiles/example-executable.dir/build.make
example-executable: /usr/local/lib/libvisp_vs.so.3.2.0
example-executable: /usr/local/lib/libvisp_tt_mi.so.3.2.0
example-executable: /usr/local/lib/libvisp_tt.so.3.2.0
example-executable: /usr/local/lib/libvisp_mbt.so.3.2.0
example-executable: /usr/local/lib/libvisp_klt.so.3.2.0
example-executable: /usr/local/lib/libvisp_robot.so.3.2.0
example-executable: /usr/local/lib/libvisp_imgproc.so.3.2.0
example-executable: /usr/local/lib/libvisp_gui.so.3.2.0
example-executable: /usr/local/lib/libvisp_detection.so.3.2.0
example-executable: /usr/local/lib/libvisp_ar.so.3.2.0
example-executable: /usr/local/lib/libvisp_sensor.so.3.2.0
example-executable: /usr/lib/x86_64-linux-gnu/libv4l2.so
example-executable: /usr/lib/x86_64-linux-gnu/libv4lconvert.so
example-executable: /usr/lib/x86_64-linux-gnu/libdc1394.so
example-executable: /usr/lib/x86_64-linux-gnu/libfreenect.so
example-executable: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
example-executable: /usr/lib/x86_64-linux-gnu/libSM.so
example-executable: /usr/lib/x86_64-linux-gnu/libICE.so
example-executable: /usr/lib/x86_64-linux-gnu/libX11.so
example-executable: /usr/lib/x86_64-linux-gnu/libXext.so
example-executable: /usr/lib/x86_64-linux-gnu/libboost_thread.so
example-executable: /usr/lib/x86_64-linux-gnu/libboost_system.so
example-executable: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
example-executable: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
example-executable: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
example-executable: /usr/lib/x86_64-linux-gnu/libOgreMain.so
example-executable: /usr/lib/x86_64-linux-gnu/libboost_thread.so
example-executable: /usr/lib/x86_64-linux-gnu/libboost_system.so
example-executable: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
example-executable: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
example-executable: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
example-executable: /usr/lib/x86_64-linux-gnu/libOgreMain.so
example-executable: /usr/lib/x86_64-linux-gnu/libOIS.so
example-executable: /usr/lib/x86_64-linux-gnu/libGLU.so
example-executable: /usr/lib/x86_64-linux-gnu/libGL.so
example-executable: /usr/lib/x86_64-linux-gnu/libCoin.so
example-executable: /usr/local/lib/libvisp_vision.so.3.2.0
example-executable: /usr/local/lib/libvisp_visual_features.so.3.2.0
example-executable: /usr/local/lib/libvisp_me.so.3.2.0
example-executable: /usr/local/lib/libvisp_blob.so.3.2.0
example-executable: /usr/local/lib/libvisp_io.so.3.2.0
example-executable: /usr/lib/x86_64-linux-gnu/libjpeg.so
example-executable: /usr/lib/x86_64-linux-gnu/libpng.so
example-executable: /usr/local/lib/libvisp_core.so.3.2.0
example-executable: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.9
example-executable: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.9
example-executable: /usr/lib/x86_64-linux-gnu/libopencv_ts.so.2.4.9
example-executable: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.9
example-executable: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.9
example-executable: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.9
example-executable: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.9
example-executable: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.9
example-executable: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.9
example-executable: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.9
example-executable: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.9
example-executable: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.9
example-executable: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.9
example-executable: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.9
example-executable: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.9
example-executable: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.9
example-executable: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.9
example-executable: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.9
example-executable: /usr/lib/x86_64-linux-gnu/libgsl.so
example-executable: /usr/lib/x86_64-linux-gnu/libgslcblas.so
example-executable: /usr/lib/liblapack.so
example-executable: /usr/lib/libblas.so
example-executable: /usr/lib/x86_64-linux-gnu/libxml2.so
example-executable: /usr/lib/x86_64-linux-gnu/libz.so
example-executable: /usr/lib/x86_64-linux-gnu/libpthread.so
example-executable: /usr/lib/x86_64-linux-gnu/libzbar.so
example-executable: /usr/lib/x86_64-linux-gnu/libdmtx.so
example-executable: /usr/lib/x86_64-linux-gnu/libm.so
example-executable: /usr/lib/x86_64-linux-gnu/libnsl.so
example-executable: CMakeFiles/example-executable.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/arunav/code-new/source/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable example-executable"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/example-executable.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/example-executable.dir/build: example-executable

.PHONY : CMakeFiles/example-executable.dir/build

CMakeFiles/example-executable.dir/requires: CMakeFiles/example-executable.dir/example.cpp.o.requires

.PHONY : CMakeFiles/example-executable.dir/requires

CMakeFiles/example-executable.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/example-executable.dir/cmake_clean.cmake
.PHONY : CMakeFiles/example-executable.dir/clean

CMakeFiles/example-executable.dir/depend:
	cd /home/arunav/code-new/source && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/arunav/code-new/source /home/arunav/code-new/source /home/arunav/code-new/source /home/arunav/code-new/source /home/arunav/code-new/source/CMakeFiles/example-executable.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/example-executable.dir/depend


# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.19

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/cui/cmake-install/bin/cmake

# The command to remove a file.
RM = /home/cui/cmake-install/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/cui/ROS_workspaces/ws_curb/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cui/ROS_workspaces/ws_curb/build

# Include any dependencies generated for this target.
include curb_features_extract/src/star_shaped/CMakeFiles/star_shaped.dir/depend.make

# Include the progress variables for this target.
include curb_features_extract/src/star_shaped/CMakeFiles/star_shaped.dir/progress.make

# Include the compile flags for this target's objects.
include curb_features_extract/src/star_shaped/CMakeFiles/star_shaped.dir/flags.make

curb_features_extract/src/star_shaped/CMakeFiles/star_shaped.dir/star_shaped.cpp.o: curb_features_extract/src/star_shaped/CMakeFiles/star_shaped.dir/flags.make
curb_features_extract/src/star_shaped/CMakeFiles/star_shaped.dir/star_shaped.cpp.o: /home/cui/ROS_workspaces/ws_curb/src/curb_features_extract/src/star_shaped/star_shaped.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cui/ROS_workspaces/ws_curb/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object curb_features_extract/src/star_shaped/CMakeFiles/star_shaped.dir/star_shaped.cpp.o"
	cd /home/cui/ROS_workspaces/ws_curb/build/curb_features_extract/src/star_shaped && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/star_shaped.dir/star_shaped.cpp.o -c /home/cui/ROS_workspaces/ws_curb/src/curb_features_extract/src/star_shaped/star_shaped.cpp

curb_features_extract/src/star_shaped/CMakeFiles/star_shaped.dir/star_shaped.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/star_shaped.dir/star_shaped.cpp.i"
	cd /home/cui/ROS_workspaces/ws_curb/build/curb_features_extract/src/star_shaped && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cui/ROS_workspaces/ws_curb/src/curb_features_extract/src/star_shaped/star_shaped.cpp > CMakeFiles/star_shaped.dir/star_shaped.cpp.i

curb_features_extract/src/star_shaped/CMakeFiles/star_shaped.dir/star_shaped.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/star_shaped.dir/star_shaped.cpp.s"
	cd /home/cui/ROS_workspaces/ws_curb/build/curb_features_extract/src/star_shaped && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cui/ROS_workspaces/ws_curb/src/curb_features_extract/src/star_shaped/star_shaped.cpp -o CMakeFiles/star_shaped.dir/star_shaped.cpp.s

# Object files for target star_shaped
star_shaped_OBJECTS = \
"CMakeFiles/star_shaped.dir/star_shaped.cpp.o"

# External object files for target star_shaped
star_shaped_EXTERNAL_OBJECTS =

/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: curb_features_extract/src/star_shaped/CMakeFiles/star_shaped.dir/star_shaped.cpp.o
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: curb_features_extract/src/star_shaped/CMakeFiles/star_shaped.dir/build.make
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
/home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so: curb_features_extract/src/star_shaped/CMakeFiles/star_shaped.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cui/ROS_workspaces/ws_curb/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so"
	cd /home/cui/ROS_workspaces/ws_curb/build/curb_features_extract/src/star_shaped && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/star_shaped.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
curb_features_extract/src/star_shaped/CMakeFiles/star_shaped.dir/build: /home/cui/ROS_workspaces/ws_curb/devel/lib/libstar_shaped.so

.PHONY : curb_features_extract/src/star_shaped/CMakeFiles/star_shaped.dir/build

curb_features_extract/src/star_shaped/CMakeFiles/star_shaped.dir/clean:
	cd /home/cui/ROS_workspaces/ws_curb/build/curb_features_extract/src/star_shaped && $(CMAKE_COMMAND) -P CMakeFiles/star_shaped.dir/cmake_clean.cmake
.PHONY : curb_features_extract/src/star_shaped/CMakeFiles/star_shaped.dir/clean

curb_features_extract/src/star_shaped/CMakeFiles/star_shaped.dir/depend:
	cd /home/cui/ROS_workspaces/ws_curb/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cui/ROS_workspaces/ws_curb/src /home/cui/ROS_workspaces/ws_curb/src/curb_features_extract/src/star_shaped /home/cui/ROS_workspaces/ws_curb/build /home/cui/ROS_workspaces/ws_curb/build/curb_features_extract/src/star_shaped /home/cui/ROS_workspaces/ws_curb/build/curb_features_extract/src/star_shaped/CMakeFiles/star_shaped.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : curb_features_extract/src/star_shaped/CMakeFiles/star_shaped.dir/depend


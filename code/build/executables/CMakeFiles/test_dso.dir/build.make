# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.23

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
CMAKE_COMMAND = /snap/cmake/1070/bin/cmake

# The command to remove a file.
RM = /snap/cmake/1070/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/manu/Desktop/thesis_refactoring/code

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/manu/Desktop/thesis_refactoring/code/build

# Include any dependencies generated for this target.
include executables/CMakeFiles/test_dso.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include executables/CMakeFiles/test_dso.dir/compiler_depend.make

# Include the progress variables for this target.
include executables/CMakeFiles/test_dso.dir/progress.make

# Include the compile flags for this target's objects.
include executables/CMakeFiles/test_dso.dir/flags.make

executables/CMakeFiles/test_dso.dir/test_dso.cpp.o: executables/CMakeFiles/test_dso.dir/flags.make
executables/CMakeFiles/test_dso.dir/test_dso.cpp.o: ../executables/test_dso.cpp
executables/CMakeFiles/test_dso.dir/test_dso.cpp.o: executables/CMakeFiles/test_dso.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/manu/Desktop/thesis_refactoring/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object executables/CMakeFiles/test_dso.dir/test_dso.cpp.o"
	cd /home/manu/Desktop/thesis_refactoring/code/build/executables && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT executables/CMakeFiles/test_dso.dir/test_dso.cpp.o -MF CMakeFiles/test_dso.dir/test_dso.cpp.o.d -o CMakeFiles/test_dso.dir/test_dso.cpp.o -c /home/manu/Desktop/thesis_refactoring/code/executables/test_dso.cpp

executables/CMakeFiles/test_dso.dir/test_dso.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_dso.dir/test_dso.cpp.i"
	cd /home/manu/Desktop/thesis_refactoring/code/build/executables && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/manu/Desktop/thesis_refactoring/code/executables/test_dso.cpp > CMakeFiles/test_dso.dir/test_dso.cpp.i

executables/CMakeFiles/test_dso.dir/test_dso.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_dso.dir/test_dso.cpp.s"
	cd /home/manu/Desktop/thesis_refactoring/code/build/executables && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/manu/Desktop/thesis_refactoring/code/executables/test_dso.cpp -o CMakeFiles/test_dso.dir/test_dso.cpp.s

executables/CMakeFiles/test_dso.dir/__/src/image.cpp.o: executables/CMakeFiles/test_dso.dir/flags.make
executables/CMakeFiles/test_dso.dir/__/src/image.cpp.o: ../src/image.cpp
executables/CMakeFiles/test_dso.dir/__/src/image.cpp.o: executables/CMakeFiles/test_dso.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/manu/Desktop/thesis_refactoring/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object executables/CMakeFiles/test_dso.dir/__/src/image.cpp.o"
	cd /home/manu/Desktop/thesis_refactoring/code/build/executables && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT executables/CMakeFiles/test_dso.dir/__/src/image.cpp.o -MF CMakeFiles/test_dso.dir/__/src/image.cpp.o.d -o CMakeFiles/test_dso.dir/__/src/image.cpp.o -c /home/manu/Desktop/thesis_refactoring/code/src/image.cpp

executables/CMakeFiles/test_dso.dir/__/src/image.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_dso.dir/__/src/image.cpp.i"
	cd /home/manu/Desktop/thesis_refactoring/code/build/executables && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/manu/Desktop/thesis_refactoring/code/src/image.cpp > CMakeFiles/test_dso.dir/__/src/image.cpp.i

executables/CMakeFiles/test_dso.dir/__/src/image.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_dso.dir/__/src/image.cpp.s"
	cd /home/manu/Desktop/thesis_refactoring/code/build/executables && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/manu/Desktop/thesis_refactoring/code/src/image.cpp -o CMakeFiles/test_dso.dir/__/src/image.cpp.s

executables/CMakeFiles/test_dso.dir/__/src/CamerasContainer.cpp.o: executables/CMakeFiles/test_dso.dir/flags.make
executables/CMakeFiles/test_dso.dir/__/src/CamerasContainer.cpp.o: ../src/CamerasContainer.cpp
executables/CMakeFiles/test_dso.dir/__/src/CamerasContainer.cpp.o: executables/CMakeFiles/test_dso.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/manu/Desktop/thesis_refactoring/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object executables/CMakeFiles/test_dso.dir/__/src/CamerasContainer.cpp.o"
	cd /home/manu/Desktop/thesis_refactoring/code/build/executables && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT executables/CMakeFiles/test_dso.dir/__/src/CamerasContainer.cpp.o -MF CMakeFiles/test_dso.dir/__/src/CamerasContainer.cpp.o.d -o CMakeFiles/test_dso.dir/__/src/CamerasContainer.cpp.o -c /home/manu/Desktop/thesis_refactoring/code/src/CamerasContainer.cpp

executables/CMakeFiles/test_dso.dir/__/src/CamerasContainer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_dso.dir/__/src/CamerasContainer.cpp.i"
	cd /home/manu/Desktop/thesis_refactoring/code/build/executables && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/manu/Desktop/thesis_refactoring/code/src/CamerasContainer.cpp > CMakeFiles/test_dso.dir/__/src/CamerasContainer.cpp.i

executables/CMakeFiles/test_dso.dir/__/src/CamerasContainer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_dso.dir/__/src/CamerasContainer.cpp.s"
	cd /home/manu/Desktop/thesis_refactoring/code/build/executables && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/manu/Desktop/thesis_refactoring/code/src/CamerasContainer.cpp -o CMakeFiles/test_dso.dir/__/src/CamerasContainer.cpp.s

executables/CMakeFiles/test_dso.dir/__/src/CameraForMapping.cpp.o: executables/CMakeFiles/test_dso.dir/flags.make
executables/CMakeFiles/test_dso.dir/__/src/CameraForMapping.cpp.o: ../src/CameraForMapping.cpp
executables/CMakeFiles/test_dso.dir/__/src/CameraForMapping.cpp.o: executables/CMakeFiles/test_dso.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/manu/Desktop/thesis_refactoring/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object executables/CMakeFiles/test_dso.dir/__/src/CameraForMapping.cpp.o"
	cd /home/manu/Desktop/thesis_refactoring/code/build/executables && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT executables/CMakeFiles/test_dso.dir/__/src/CameraForMapping.cpp.o -MF CMakeFiles/test_dso.dir/__/src/CameraForMapping.cpp.o.d -o CMakeFiles/test_dso.dir/__/src/CameraForMapping.cpp.o -c /home/manu/Desktop/thesis_refactoring/code/src/CameraForMapping.cpp

executables/CMakeFiles/test_dso.dir/__/src/CameraForMapping.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_dso.dir/__/src/CameraForMapping.cpp.i"
	cd /home/manu/Desktop/thesis_refactoring/code/build/executables && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/manu/Desktop/thesis_refactoring/code/src/CameraForMapping.cpp > CMakeFiles/test_dso.dir/__/src/CameraForMapping.cpp.i

executables/CMakeFiles/test_dso.dir/__/src/CameraForMapping.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_dso.dir/__/src/CameraForMapping.cpp.s"
	cd /home/manu/Desktop/thesis_refactoring/code/build/executables && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/manu/Desktop/thesis_refactoring/code/src/CameraForMapping.cpp -o CMakeFiles/test_dso.dir/__/src/CameraForMapping.cpp.s

executables/CMakeFiles/test_dso.dir/__/src/dso.cpp.o: executables/CMakeFiles/test_dso.dir/flags.make
executables/CMakeFiles/test_dso.dir/__/src/dso.cpp.o: ../src/dso.cpp
executables/CMakeFiles/test_dso.dir/__/src/dso.cpp.o: executables/CMakeFiles/test_dso.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/manu/Desktop/thesis_refactoring/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object executables/CMakeFiles/test_dso.dir/__/src/dso.cpp.o"
	cd /home/manu/Desktop/thesis_refactoring/code/build/executables && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT executables/CMakeFiles/test_dso.dir/__/src/dso.cpp.o -MF CMakeFiles/test_dso.dir/__/src/dso.cpp.o.d -o CMakeFiles/test_dso.dir/__/src/dso.cpp.o -c /home/manu/Desktop/thesis_refactoring/code/src/dso.cpp

executables/CMakeFiles/test_dso.dir/__/src/dso.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_dso.dir/__/src/dso.cpp.i"
	cd /home/manu/Desktop/thesis_refactoring/code/build/executables && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/manu/Desktop/thesis_refactoring/code/src/dso.cpp > CMakeFiles/test_dso.dir/__/src/dso.cpp.i

executables/CMakeFiles/test_dso.dir/__/src/dso.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_dso.dir/__/src/dso.cpp.s"
	cd /home/manu/Desktop/thesis_refactoring/code/build/executables && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/manu/Desktop/thesis_refactoring/code/src/dso.cpp -o CMakeFiles/test_dso.dir/__/src/dso.cpp.s

executables/CMakeFiles/test_dso.dir/__/src/camera.cpp.o: executables/CMakeFiles/test_dso.dir/flags.make
executables/CMakeFiles/test_dso.dir/__/src/camera.cpp.o: ../src/camera.cpp
executables/CMakeFiles/test_dso.dir/__/src/camera.cpp.o: executables/CMakeFiles/test_dso.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/manu/Desktop/thesis_refactoring/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object executables/CMakeFiles/test_dso.dir/__/src/camera.cpp.o"
	cd /home/manu/Desktop/thesis_refactoring/code/build/executables && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT executables/CMakeFiles/test_dso.dir/__/src/camera.cpp.o -MF CMakeFiles/test_dso.dir/__/src/camera.cpp.o.d -o CMakeFiles/test_dso.dir/__/src/camera.cpp.o -c /home/manu/Desktop/thesis_refactoring/code/src/camera.cpp

executables/CMakeFiles/test_dso.dir/__/src/camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_dso.dir/__/src/camera.cpp.i"
	cd /home/manu/Desktop/thesis_refactoring/code/build/executables && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/manu/Desktop/thesis_refactoring/code/src/camera.cpp > CMakeFiles/test_dso.dir/__/src/camera.cpp.i

executables/CMakeFiles/test_dso.dir/__/src/camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_dso.dir/__/src/camera.cpp.s"
	cd /home/manu/Desktop/thesis_refactoring/code/build/executables && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/manu/Desktop/thesis_refactoring/code/src/camera.cpp -o CMakeFiles/test_dso.dir/__/src/camera.cpp.s

executables/CMakeFiles/test_dso.dir/__/src/Pyramid.cpp.o: executables/CMakeFiles/test_dso.dir/flags.make
executables/CMakeFiles/test_dso.dir/__/src/Pyramid.cpp.o: ../src/Pyramid.cpp
executables/CMakeFiles/test_dso.dir/__/src/Pyramid.cpp.o: executables/CMakeFiles/test_dso.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/manu/Desktop/thesis_refactoring/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object executables/CMakeFiles/test_dso.dir/__/src/Pyramid.cpp.o"
	cd /home/manu/Desktop/thesis_refactoring/code/build/executables && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT executables/CMakeFiles/test_dso.dir/__/src/Pyramid.cpp.o -MF CMakeFiles/test_dso.dir/__/src/Pyramid.cpp.o.d -o CMakeFiles/test_dso.dir/__/src/Pyramid.cpp.o -c /home/manu/Desktop/thesis_refactoring/code/src/Pyramid.cpp

executables/CMakeFiles/test_dso.dir/__/src/Pyramid.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_dso.dir/__/src/Pyramid.cpp.i"
	cd /home/manu/Desktop/thesis_refactoring/code/build/executables && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/manu/Desktop/thesis_refactoring/code/src/Pyramid.cpp > CMakeFiles/test_dso.dir/__/src/Pyramid.cpp.i

executables/CMakeFiles/test_dso.dir/__/src/Pyramid.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_dso.dir/__/src/Pyramid.cpp.s"
	cd /home/manu/Desktop/thesis_refactoring/code/build/executables && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/manu/Desktop/thesis_refactoring/code/src/Pyramid.cpp -o CMakeFiles/test_dso.dir/__/src/Pyramid.cpp.s

executables/CMakeFiles/test_dso.dir/__/src/environment.cpp.o: executables/CMakeFiles/test_dso.dir/flags.make
executables/CMakeFiles/test_dso.dir/__/src/environment.cpp.o: ../src/environment.cpp
executables/CMakeFiles/test_dso.dir/__/src/environment.cpp.o: executables/CMakeFiles/test_dso.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/manu/Desktop/thesis_refactoring/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object executables/CMakeFiles/test_dso.dir/__/src/environment.cpp.o"
	cd /home/manu/Desktop/thesis_refactoring/code/build/executables && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT executables/CMakeFiles/test_dso.dir/__/src/environment.cpp.o -MF CMakeFiles/test_dso.dir/__/src/environment.cpp.o.d -o CMakeFiles/test_dso.dir/__/src/environment.cpp.o -c /home/manu/Desktop/thesis_refactoring/code/src/environment.cpp

executables/CMakeFiles/test_dso.dir/__/src/environment.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_dso.dir/__/src/environment.cpp.i"
	cd /home/manu/Desktop/thesis_refactoring/code/build/executables && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/manu/Desktop/thesis_refactoring/code/src/environment.cpp > CMakeFiles/test_dso.dir/__/src/environment.cpp.i

executables/CMakeFiles/test_dso.dir/__/src/environment.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_dso.dir/__/src/environment.cpp.s"
	cd /home/manu/Desktop/thesis_refactoring/code/build/executables && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/manu/Desktop/thesis_refactoring/code/src/environment.cpp -o CMakeFiles/test_dso.dir/__/src/environment.cpp.s

executables/CMakeFiles/test_dso.dir/__/src/Tracker.cpp.o: executables/CMakeFiles/test_dso.dir/flags.make
executables/CMakeFiles/test_dso.dir/__/src/Tracker.cpp.o: ../src/Tracker.cpp
executables/CMakeFiles/test_dso.dir/__/src/Tracker.cpp.o: executables/CMakeFiles/test_dso.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/manu/Desktop/thesis_refactoring/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object executables/CMakeFiles/test_dso.dir/__/src/Tracker.cpp.o"
	cd /home/manu/Desktop/thesis_refactoring/code/build/executables && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT executables/CMakeFiles/test_dso.dir/__/src/Tracker.cpp.o -MF CMakeFiles/test_dso.dir/__/src/Tracker.cpp.o.d -o CMakeFiles/test_dso.dir/__/src/Tracker.cpp.o -c /home/manu/Desktop/thesis_refactoring/code/src/Tracker.cpp

executables/CMakeFiles/test_dso.dir/__/src/Tracker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_dso.dir/__/src/Tracker.cpp.i"
	cd /home/manu/Desktop/thesis_refactoring/code/build/executables && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/manu/Desktop/thesis_refactoring/code/src/Tracker.cpp > CMakeFiles/test_dso.dir/__/src/Tracker.cpp.i

executables/CMakeFiles/test_dso.dir/__/src/Tracker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_dso.dir/__/src/Tracker.cpp.s"
	cd /home/manu/Desktop/thesis_refactoring/code/build/executables && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/manu/Desktop/thesis_refactoring/code/src/Tracker.cpp -o CMakeFiles/test_dso.dir/__/src/Tracker.cpp.s

executables/CMakeFiles/test_dso.dir/__/src/utils.cpp.o: executables/CMakeFiles/test_dso.dir/flags.make
executables/CMakeFiles/test_dso.dir/__/src/utils.cpp.o: ../src/utils.cpp
executables/CMakeFiles/test_dso.dir/__/src/utils.cpp.o: executables/CMakeFiles/test_dso.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/manu/Desktop/thesis_refactoring/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object executables/CMakeFiles/test_dso.dir/__/src/utils.cpp.o"
	cd /home/manu/Desktop/thesis_refactoring/code/build/executables && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT executables/CMakeFiles/test_dso.dir/__/src/utils.cpp.o -MF CMakeFiles/test_dso.dir/__/src/utils.cpp.o.d -o CMakeFiles/test_dso.dir/__/src/utils.cpp.o -c /home/manu/Desktop/thesis_refactoring/code/src/utils.cpp

executables/CMakeFiles/test_dso.dir/__/src/utils.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_dso.dir/__/src/utils.cpp.i"
	cd /home/manu/Desktop/thesis_refactoring/code/build/executables && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/manu/Desktop/thesis_refactoring/code/src/utils.cpp > CMakeFiles/test_dso.dir/__/src/utils.cpp.i

executables/CMakeFiles/test_dso.dir/__/src/utils.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_dso.dir/__/src/utils.cpp.s"
	cd /home/manu/Desktop/thesis_refactoring/code/build/executables && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/manu/Desktop/thesis_refactoring/code/src/utils.cpp -o CMakeFiles/test_dso.dir/__/src/utils.cpp.s

executables/CMakeFiles/test_dso.dir/__/src/KeyframeHandler.cpp.o: executables/CMakeFiles/test_dso.dir/flags.make
executables/CMakeFiles/test_dso.dir/__/src/KeyframeHandler.cpp.o: ../src/KeyframeHandler.cpp
executables/CMakeFiles/test_dso.dir/__/src/KeyframeHandler.cpp.o: executables/CMakeFiles/test_dso.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/manu/Desktop/thesis_refactoring/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object executables/CMakeFiles/test_dso.dir/__/src/KeyframeHandler.cpp.o"
	cd /home/manu/Desktop/thesis_refactoring/code/build/executables && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT executables/CMakeFiles/test_dso.dir/__/src/KeyframeHandler.cpp.o -MF CMakeFiles/test_dso.dir/__/src/KeyframeHandler.cpp.o.d -o CMakeFiles/test_dso.dir/__/src/KeyframeHandler.cpp.o -c /home/manu/Desktop/thesis_refactoring/code/src/KeyframeHandler.cpp

executables/CMakeFiles/test_dso.dir/__/src/KeyframeHandler.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_dso.dir/__/src/KeyframeHandler.cpp.i"
	cd /home/manu/Desktop/thesis_refactoring/code/build/executables && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/manu/Desktop/thesis_refactoring/code/src/KeyframeHandler.cpp > CMakeFiles/test_dso.dir/__/src/KeyframeHandler.cpp.i

executables/CMakeFiles/test_dso.dir/__/src/KeyframeHandler.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_dso.dir/__/src/KeyframeHandler.cpp.s"
	cd /home/manu/Desktop/thesis_refactoring/code/build/executables && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/manu/Desktop/thesis_refactoring/code/src/KeyframeHandler.cpp -o CMakeFiles/test_dso.dir/__/src/KeyframeHandler.cpp.s

executables/CMakeFiles/test_dso.dir/__/src/initializer.cpp.o: executables/CMakeFiles/test_dso.dir/flags.make
executables/CMakeFiles/test_dso.dir/__/src/initializer.cpp.o: ../src/initializer.cpp
executables/CMakeFiles/test_dso.dir/__/src/initializer.cpp.o: executables/CMakeFiles/test_dso.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/manu/Desktop/thesis_refactoring/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object executables/CMakeFiles/test_dso.dir/__/src/initializer.cpp.o"
	cd /home/manu/Desktop/thesis_refactoring/code/build/executables && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT executables/CMakeFiles/test_dso.dir/__/src/initializer.cpp.o -MF CMakeFiles/test_dso.dir/__/src/initializer.cpp.o.d -o CMakeFiles/test_dso.dir/__/src/initializer.cpp.o -c /home/manu/Desktop/thesis_refactoring/code/src/initializer.cpp

executables/CMakeFiles/test_dso.dir/__/src/initializer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_dso.dir/__/src/initializer.cpp.i"
	cd /home/manu/Desktop/thesis_refactoring/code/build/executables && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/manu/Desktop/thesis_refactoring/code/src/initializer.cpp > CMakeFiles/test_dso.dir/__/src/initializer.cpp.i

executables/CMakeFiles/test_dso.dir/__/src/initializer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_dso.dir/__/src/initializer.cpp.s"
	cd /home/manu/Desktop/thesis_refactoring/code/build/executables && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/manu/Desktop/thesis_refactoring/code/src/initializer.cpp -o CMakeFiles/test_dso.dir/__/src/initializer.cpp.s

# Object files for target test_dso
test_dso_OBJECTS = \
"CMakeFiles/test_dso.dir/test_dso.cpp.o" \
"CMakeFiles/test_dso.dir/__/src/image.cpp.o" \
"CMakeFiles/test_dso.dir/__/src/CamerasContainer.cpp.o" \
"CMakeFiles/test_dso.dir/__/src/CameraForMapping.cpp.o" \
"CMakeFiles/test_dso.dir/__/src/dso.cpp.o" \
"CMakeFiles/test_dso.dir/__/src/camera.cpp.o" \
"CMakeFiles/test_dso.dir/__/src/Pyramid.cpp.o" \
"CMakeFiles/test_dso.dir/__/src/environment.cpp.o" \
"CMakeFiles/test_dso.dir/__/src/Tracker.cpp.o" \
"CMakeFiles/test_dso.dir/__/src/utils.cpp.o" \
"CMakeFiles/test_dso.dir/__/src/KeyframeHandler.cpp.o" \
"CMakeFiles/test_dso.dir/__/src/initializer.cpp.o"

# External object files for target test_dso
test_dso_EXTERNAL_OBJECTS =

executables/test_dso: executables/CMakeFiles/test_dso.dir/test_dso.cpp.o
executables/test_dso: executables/CMakeFiles/test_dso.dir/__/src/image.cpp.o
executables/test_dso: executables/CMakeFiles/test_dso.dir/__/src/CamerasContainer.cpp.o
executables/test_dso: executables/CMakeFiles/test_dso.dir/__/src/CameraForMapping.cpp.o
executables/test_dso: executables/CMakeFiles/test_dso.dir/__/src/dso.cpp.o
executables/test_dso: executables/CMakeFiles/test_dso.dir/__/src/camera.cpp.o
executables/test_dso: executables/CMakeFiles/test_dso.dir/__/src/Pyramid.cpp.o
executables/test_dso: executables/CMakeFiles/test_dso.dir/__/src/environment.cpp.o
executables/test_dso: executables/CMakeFiles/test_dso.dir/__/src/Tracker.cpp.o
executables/test_dso: executables/CMakeFiles/test_dso.dir/__/src/utils.cpp.o
executables/test_dso: executables/CMakeFiles/test_dso.dir/__/src/KeyframeHandler.cpp.o
executables/test_dso: executables/CMakeFiles/test_dso.dir/__/src/initializer.cpp.o
executables/test_dso: executables/CMakeFiles/test_dso.dir/build.make
executables/test_dso: /usr/local/lib/libopencv_gapi.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_stitching.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_alphamat.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_aruco.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_bgsegm.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_bioinspired.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_ccalib.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_cudabgsegm.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_cudafeatures2d.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_cudaobjdetect.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_cudastereo.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_dnn_objdetect.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_dnn_superres.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_dpm.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_face.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_freetype.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_fuzzy.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_hdf.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_hfs.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_img_hash.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_intensity_transform.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_line_descriptor.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_mcc.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_quality.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_rapid.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_reg.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_rgbd.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_saliency.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_sfm.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_stereo.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_structured_light.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_superres.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_surface_matching.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_tracking.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_videostab.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_wechat_qrcode.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_xfeatures2d.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_xobjdetect.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_xphoto.so.4.5.2
executables/test_dso: /usr/lib/x86_64-linux-gnu/libpython3.6m.so
executables/test_dso: /usr/local/lib/libopencv_shape.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_highgui.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_datasets.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_plot.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_text.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_ml.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_phase_unwrapping.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_videoio.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_cudaoptflow.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_cudalegacy.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_cudawarping.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_optflow.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_ximgproc.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_video.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_dnn.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_imgcodecs.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_objdetect.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_calib3d.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_features2d.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_flann.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_photo.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_cudaimgproc.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_cudafilters.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_imgproc.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_cudaarithm.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_core.so.4.5.2
executables/test_dso: /usr/local/lib/libopencv_cudev.so.4.5.2
executables/test_dso: executables/CMakeFiles/test_dso.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/manu/Desktop/thesis_refactoring/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Linking CXX executable test_dso"
	cd /home/manu/Desktop/thesis_refactoring/code/build/executables && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_dso.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
executables/CMakeFiles/test_dso.dir/build: executables/test_dso
.PHONY : executables/CMakeFiles/test_dso.dir/build

executables/CMakeFiles/test_dso.dir/clean:
	cd /home/manu/Desktop/thesis_refactoring/code/build/executables && $(CMAKE_COMMAND) -P CMakeFiles/test_dso.dir/cmake_clean.cmake
.PHONY : executables/CMakeFiles/test_dso.dir/clean

executables/CMakeFiles/test_dso.dir/depend:
	cd /home/manu/Desktop/thesis_refactoring/code/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/manu/Desktop/thesis_refactoring/code /home/manu/Desktop/thesis_refactoring/code/executables /home/manu/Desktop/thesis_refactoring/code/build /home/manu/Desktop/thesis_refactoring/code/build/executables /home/manu/Desktop/thesis_refactoring/code/build/executables/CMakeFiles/test_dso.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : executables/CMakeFiles/test_dso.dir/depend


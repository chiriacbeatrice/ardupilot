# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/beatricec/LicentaBetty/ardupilot/build/px4-v3/modules/PX4Firmware

# Utility rule file for submodule_clean.

# Include the progress variables for this target.
include CMakeFiles/submodule_clean.dir/progress.make

CMakeFiles/submodule_clean:
	cd /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware && git submodule deinit -f .
	cd /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware && rm -rf .git/modules/*

submodule_clean: CMakeFiles/submodule_clean
submodule_clean: CMakeFiles/submodule_clean.dir/build.make

.PHONY : submodule_clean

# Rule to build all files generated by this target.
CMakeFiles/submodule_clean.dir/build: submodule_clean

.PHONY : CMakeFiles/submodule_clean.dir/build

CMakeFiles/submodule_clean.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/submodule_clean.dir/cmake_clean.cmake
.PHONY : CMakeFiles/submodule_clean.dir/clean

CMakeFiles/submodule_clean.dir/depend:
	cd /home/beatricec/LicentaBetty/ardupilot/build/px4-v3/modules/PX4Firmware && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware /home/beatricec/LicentaBetty/ardupilot/build/px4-v3/modules/PX4Firmware /home/beatricec/LicentaBetty/ardupilot/build/px4-v3/modules/PX4Firmware /home/beatricec/LicentaBetty/ardupilot/build/px4-v3/modules/PX4Firmware/CMakeFiles/submodule_clean.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/submodule_clean.dir/depend


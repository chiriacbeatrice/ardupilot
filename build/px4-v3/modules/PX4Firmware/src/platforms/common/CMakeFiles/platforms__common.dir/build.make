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

# Include any dependencies generated for this target.
include src/platforms/common/CMakeFiles/platforms__common.dir/depend.make

# Include the progress variables for this target.
include src/platforms/common/CMakeFiles/platforms__common.dir/progress.make

# Include the compile flags for this target's objects.
include src/platforms/common/CMakeFiles/platforms__common.dir/flags.make

src/platforms/common/CMakeFiles/platforms__common.dir/px4_getopt.c.obj: src/platforms/common/CMakeFiles/platforms__common.dir/flags.make
src/platforms/common/CMakeFiles/platforms__common.dir/px4_getopt.c.obj: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/platforms/common/px4_getopt.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/beatricec/LicentaBetty/ardupilot/build/px4-v3/modules/PX4Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object src/platforms/common/CMakeFiles/platforms__common.dir/px4_getopt.c.obj"
	cd /home/beatricec/LicentaBetty/ardupilot/build/px4-v3/modules/PX4Firmware/src/platforms/common && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/platforms__common.dir/px4_getopt.c.obj   -c /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/platforms/common/px4_getopt.c

src/platforms/common/CMakeFiles/platforms__common.dir/px4_getopt.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/platforms__common.dir/px4_getopt.c.i"
	cd /home/beatricec/LicentaBetty/ardupilot/build/px4-v3/modules/PX4Firmware/src/platforms/common && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/platforms/common/px4_getopt.c > CMakeFiles/platforms__common.dir/px4_getopt.c.i

src/platforms/common/CMakeFiles/platforms__common.dir/px4_getopt.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/platforms__common.dir/px4_getopt.c.s"
	cd /home/beatricec/LicentaBetty/ardupilot/build/px4-v3/modules/PX4Firmware/src/platforms/common && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/platforms/common/px4_getopt.c -o CMakeFiles/platforms__common.dir/px4_getopt.c.s

src/platforms/common/CMakeFiles/platforms__common.dir/px4_getopt.c.obj.requires:

.PHONY : src/platforms/common/CMakeFiles/platforms__common.dir/px4_getopt.c.obj.requires

src/platforms/common/CMakeFiles/platforms__common.dir/px4_getopt.c.obj.provides: src/platforms/common/CMakeFiles/platforms__common.dir/px4_getopt.c.obj.requires
	$(MAKE) -f src/platforms/common/CMakeFiles/platforms__common.dir/build.make src/platforms/common/CMakeFiles/platforms__common.dir/px4_getopt.c.obj.provides.build
.PHONY : src/platforms/common/CMakeFiles/platforms__common.dir/px4_getopt.c.obj.provides

src/platforms/common/CMakeFiles/platforms__common.dir/px4_getopt.c.obj.provides.build: src/platforms/common/CMakeFiles/platforms__common.dir/px4_getopt.c.obj


# Object files for target platforms__common
platforms__common_OBJECTS = \
"CMakeFiles/platforms__common.dir/px4_getopt.c.obj"

# External object files for target platforms__common
platforms__common_EXTERNAL_OBJECTS =

src/platforms/common/libplatforms__common.a: src/platforms/common/CMakeFiles/platforms__common.dir/px4_getopt.c.obj
src/platforms/common/libplatforms__common.a: src/platforms/common/CMakeFiles/platforms__common.dir/build.make
src/platforms/common/libplatforms__common.a: src/platforms/common/CMakeFiles/platforms__common.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/beatricec/LicentaBetty/ardupilot/build/px4-v3/modules/PX4Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C static library libplatforms__common.a"
	cd /home/beatricec/LicentaBetty/ardupilot/build/px4-v3/modules/PX4Firmware/src/platforms/common && $(CMAKE_COMMAND) -P CMakeFiles/platforms__common.dir/cmake_clean_target.cmake
	cd /home/beatricec/LicentaBetty/ardupilot/build/px4-v3/modules/PX4Firmware/src/platforms/common && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/platforms__common.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/platforms/common/CMakeFiles/platforms__common.dir/build: src/platforms/common/libplatforms__common.a

.PHONY : src/platforms/common/CMakeFiles/platforms__common.dir/build

src/platforms/common/CMakeFiles/platforms__common.dir/requires: src/platforms/common/CMakeFiles/platforms__common.dir/px4_getopt.c.obj.requires

.PHONY : src/platforms/common/CMakeFiles/platforms__common.dir/requires

src/platforms/common/CMakeFiles/platforms__common.dir/clean:
	cd /home/beatricec/LicentaBetty/ardupilot/build/px4-v3/modules/PX4Firmware/src/platforms/common && $(CMAKE_COMMAND) -P CMakeFiles/platforms__common.dir/cmake_clean.cmake
.PHONY : src/platforms/common/CMakeFiles/platforms__common.dir/clean

src/platforms/common/CMakeFiles/platforms__common.dir/depend:
	cd /home/beatricec/LicentaBetty/ardupilot/build/px4-v3/modules/PX4Firmware && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/platforms/common /home/beatricec/LicentaBetty/ardupilot/build/px4-v3/modules/PX4Firmware /home/beatricec/LicentaBetty/ardupilot/build/px4-v3/modules/PX4Firmware/src/platforms/common /home/beatricec/LicentaBetty/ardupilot/build/px4-v3/modules/PX4Firmware/src/platforms/common/CMakeFiles/platforms__common.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/platforms/common/CMakeFiles/platforms__common.dir/depend


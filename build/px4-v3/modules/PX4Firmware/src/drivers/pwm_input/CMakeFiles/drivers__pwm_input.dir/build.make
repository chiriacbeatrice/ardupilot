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
CMAKE_SOURCE_DIR = /home/beatrice/Licenta_FisiereComplete/ardupilot/modules/PX4Firmware

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware

# Include any dependencies generated for this target.
include src/drivers/pwm_input/CMakeFiles/drivers__pwm_input.dir/depend.make

# Include the progress variables for this target.
include src/drivers/pwm_input/CMakeFiles/drivers__pwm_input.dir/progress.make

# Include the compile flags for this target's objects.
include src/drivers/pwm_input/CMakeFiles/drivers__pwm_input.dir/flags.make

src/drivers/pwm_input/CMakeFiles/drivers__pwm_input.dir/pwm_input.cpp.obj: src/drivers/pwm_input/CMakeFiles/drivers__pwm_input.dir/flags.make
src/drivers/pwm_input/CMakeFiles/drivers__pwm_input.dir/pwm_input.cpp.obj: /home/beatrice/Licenta_FisiereComplete/ardupilot/modules/PX4Firmware/src/drivers/pwm_input/pwm_input.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/drivers/pwm_input/CMakeFiles/drivers__pwm_input.dir/pwm_input.cpp.obj"
	cd /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/src/drivers/pwm_input && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__pwm_input.dir/pwm_input.cpp.obj -c /home/beatrice/Licenta_FisiereComplete/ardupilot/modules/PX4Firmware/src/drivers/pwm_input/pwm_input.cpp

src/drivers/pwm_input/CMakeFiles/drivers__pwm_input.dir/pwm_input.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__pwm_input.dir/pwm_input.cpp.i"
	cd /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/src/drivers/pwm_input && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/beatrice/Licenta_FisiereComplete/ardupilot/modules/PX4Firmware/src/drivers/pwm_input/pwm_input.cpp > CMakeFiles/drivers__pwm_input.dir/pwm_input.cpp.i

src/drivers/pwm_input/CMakeFiles/drivers__pwm_input.dir/pwm_input.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__pwm_input.dir/pwm_input.cpp.s"
	cd /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/src/drivers/pwm_input && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/beatrice/Licenta_FisiereComplete/ardupilot/modules/PX4Firmware/src/drivers/pwm_input/pwm_input.cpp -o CMakeFiles/drivers__pwm_input.dir/pwm_input.cpp.s

src/drivers/pwm_input/CMakeFiles/drivers__pwm_input.dir/pwm_input.cpp.obj.requires:

.PHONY : src/drivers/pwm_input/CMakeFiles/drivers__pwm_input.dir/pwm_input.cpp.obj.requires

src/drivers/pwm_input/CMakeFiles/drivers__pwm_input.dir/pwm_input.cpp.obj.provides: src/drivers/pwm_input/CMakeFiles/drivers__pwm_input.dir/pwm_input.cpp.obj.requires
	$(MAKE) -f src/drivers/pwm_input/CMakeFiles/drivers__pwm_input.dir/build.make src/drivers/pwm_input/CMakeFiles/drivers__pwm_input.dir/pwm_input.cpp.obj.provides.build
.PHONY : src/drivers/pwm_input/CMakeFiles/drivers__pwm_input.dir/pwm_input.cpp.obj.provides

src/drivers/pwm_input/CMakeFiles/drivers__pwm_input.dir/pwm_input.cpp.obj.provides.build: src/drivers/pwm_input/CMakeFiles/drivers__pwm_input.dir/pwm_input.cpp.obj


# Object files for target drivers__pwm_input
drivers__pwm_input_OBJECTS = \
"CMakeFiles/drivers__pwm_input.dir/pwm_input.cpp.obj"

# External object files for target drivers__pwm_input
drivers__pwm_input_EXTERNAL_OBJECTS =

src/drivers/pwm_input/libdrivers__pwm_input.a: src/drivers/pwm_input/CMakeFiles/drivers__pwm_input.dir/pwm_input.cpp.obj
src/drivers/pwm_input/libdrivers__pwm_input.a: src/drivers/pwm_input/CMakeFiles/drivers__pwm_input.dir/build.make
src/drivers/pwm_input/libdrivers__pwm_input.a: src/drivers/pwm_input/CMakeFiles/drivers__pwm_input.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libdrivers__pwm_input.a"
	cd /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/src/drivers/pwm_input && $(CMAKE_COMMAND) -P CMakeFiles/drivers__pwm_input.dir/cmake_clean_target.cmake
	cd /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/src/drivers/pwm_input && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/drivers__pwm_input.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/drivers/pwm_input/CMakeFiles/drivers__pwm_input.dir/build: src/drivers/pwm_input/libdrivers__pwm_input.a

.PHONY : src/drivers/pwm_input/CMakeFiles/drivers__pwm_input.dir/build

src/drivers/pwm_input/CMakeFiles/drivers__pwm_input.dir/requires: src/drivers/pwm_input/CMakeFiles/drivers__pwm_input.dir/pwm_input.cpp.obj.requires

.PHONY : src/drivers/pwm_input/CMakeFiles/drivers__pwm_input.dir/requires

src/drivers/pwm_input/CMakeFiles/drivers__pwm_input.dir/clean:
	cd /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/src/drivers/pwm_input && $(CMAKE_COMMAND) -P CMakeFiles/drivers__pwm_input.dir/cmake_clean.cmake
.PHONY : src/drivers/pwm_input/CMakeFiles/drivers__pwm_input.dir/clean

src/drivers/pwm_input/CMakeFiles/drivers__pwm_input.dir/depend:
	cd /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/beatrice/Licenta_FisiereComplete/ardupilot/modules/PX4Firmware /home/beatrice/Licenta_FisiereComplete/ardupilot/modules/PX4Firmware/src/drivers/pwm_input /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/src/drivers/pwm_input /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/src/drivers/pwm_input/CMakeFiles/drivers__pwm_input.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/drivers/pwm_input/CMakeFiles/drivers__pwm_input.dir/depend


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

# Utility rule file for fw_io.

# Include the progress variables for this target.
include src/modules/px4iofirmware/CMakeFiles/fw_io.dir/progress.make

src/modules/px4iofirmware/CMakeFiles/fw_io: src/modules/px4iofirmware/px4io-v2.bin


src/modules/px4iofirmware/px4io-v2.bin: src/modules/px4iofirmware/px4io-v2
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating px4io-v2.bin"
	cd /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/src/modules/px4iofirmware && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-objcopy -O binary px4io-v2 px4io-v2.bin

fw_io: src/modules/px4iofirmware/CMakeFiles/fw_io
fw_io: src/modules/px4iofirmware/px4io-v2.bin
fw_io: src/modules/px4iofirmware/CMakeFiles/fw_io.dir/build.make

.PHONY : fw_io

# Rule to build all files generated by this target.
src/modules/px4iofirmware/CMakeFiles/fw_io.dir/build: fw_io

.PHONY : src/modules/px4iofirmware/CMakeFiles/fw_io.dir/build

src/modules/px4iofirmware/CMakeFiles/fw_io.dir/clean:
	cd /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/src/modules/px4iofirmware && $(CMAKE_COMMAND) -P CMakeFiles/fw_io.dir/cmake_clean.cmake
.PHONY : src/modules/px4iofirmware/CMakeFiles/fw_io.dir/clean

src/modules/px4iofirmware/CMakeFiles/fw_io.dir/depend:
	cd /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/beatrice/Licenta_FisiereComplete/ardupilot/modules/PX4Firmware /home/beatrice/Licenta_FisiereComplete/ardupilot/modules/PX4Firmware/src/modules/px4iofirmware /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/src/modules/px4iofirmware /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/src/modules/px4iofirmware/CMakeFiles/fw_io.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/modules/px4iofirmware/CMakeFiles/fw_io.dir/depend


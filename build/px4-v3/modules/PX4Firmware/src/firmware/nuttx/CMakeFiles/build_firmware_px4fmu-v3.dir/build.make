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

# Utility rule file for build_firmware_px4fmu-v3.

# Include the progress variables for this target.
include src/firmware/nuttx/CMakeFiles/build_firmware_px4fmu-v3.dir/progress.make

src/firmware/nuttx/CMakeFiles/build_firmware_px4fmu-v3: src/firmware/nuttx/nuttx-px4fmu-v3-apm.px4


src/firmware/nuttx/nuttx-px4fmu-v3-apm.px4: src/firmware/nuttx/firmware_nuttx
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating nuttx-px4fmu-v3-apm.px4"
	cd /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/src/firmware/nuttx && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-objcopy -O binary /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/src/firmware/nuttx/firmware_nuttx /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/src/firmware/nuttx/firmware_nuttx.bin
	cd /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/src/firmware/nuttx && /usr/bin/python /home/beatrice/Licenta_FisiereComplete/ardupilot/modules/PX4Firmware/Tools/px_mkfw.py --prototype /home/beatrice/Licenta_FisiereComplete/ardupilot/modules/PX4Firmware/Images/px4fmu-v3.prototype --git_identity /home/beatrice/Licenta_FisiereComplete/ardupilot/modules/PX4Firmware --parameter_xml /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/parameters.xml --airframe_xml /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/airframes.xml --image /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/src/firmware/nuttx/firmware_nuttx.bin > /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/src/firmware/nuttx/nuttx-px4fmu-v3-apm.px4

build_firmware_px4fmu-v3: src/firmware/nuttx/CMakeFiles/build_firmware_px4fmu-v3
build_firmware_px4fmu-v3: src/firmware/nuttx/nuttx-px4fmu-v3-apm.px4
build_firmware_px4fmu-v3: src/firmware/nuttx/CMakeFiles/build_firmware_px4fmu-v3.dir/build.make

.PHONY : build_firmware_px4fmu-v3

# Rule to build all files generated by this target.
src/firmware/nuttx/CMakeFiles/build_firmware_px4fmu-v3.dir/build: build_firmware_px4fmu-v3

.PHONY : src/firmware/nuttx/CMakeFiles/build_firmware_px4fmu-v3.dir/build

src/firmware/nuttx/CMakeFiles/build_firmware_px4fmu-v3.dir/clean:
	cd /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/src/firmware/nuttx && $(CMAKE_COMMAND) -P CMakeFiles/build_firmware_px4fmu-v3.dir/cmake_clean.cmake
.PHONY : src/firmware/nuttx/CMakeFiles/build_firmware_px4fmu-v3.dir/clean

src/firmware/nuttx/CMakeFiles/build_firmware_px4fmu-v3.dir/depend:
	cd /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/beatrice/Licenta_FisiereComplete/ardupilot/modules/PX4Firmware /home/beatrice/Licenta_FisiereComplete/ardupilot/modules/PX4Firmware/src/firmware/nuttx /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/src/firmware/nuttx /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/src/firmware/nuttx/CMakeFiles/build_firmware_px4fmu-v3.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/firmware/nuttx/CMakeFiles/build_firmware_px4fmu-v3.dir/depend


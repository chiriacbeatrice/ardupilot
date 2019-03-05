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

# Utility rule file for nuttx_export_px4io-v2.

# Include the progress variables for this target.
include src/modules/px4iofirmware/CMakeFiles/nuttx_export_px4io-v2.dir/progress.make

src/modules/px4iofirmware/CMakeFiles/nuttx_export_px4io-v2: src/modules/px4iofirmware/nuttx_export_px4io-v2.stamp


src/modules/px4iofirmware/nuttx_export_px4io-v2.stamp: px4io-v2.export
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating nuttx_export_px4io-v2.stamp"
	cd /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/src/modules/px4iofirmware && /bin/rm -rf /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/px4io-v2/NuttX/nuttx-export
	cd /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/src/modules/px4iofirmware && /usr/bin/unzip -q /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/px4io-v2.export -d /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/px4io-v2/NuttX
	cd /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/src/modules/px4iofirmware && /usr/bin/touch nuttx_export_px4io-v2.stamp

px4io-v2.export: /home/beatrice/Licenta_FisiereComplete/ardupilot/modules/PX4Firmware/nuttx-configs/px4io-v2/include/board.h
px4io-v2.export: /home/beatrice/Licenta_FisiereComplete/ardupilot/modules/PX4Firmware/nuttx-configs/px4io-v2/nsh/Make.defs
px4io-v2.export: /home/beatrice/Licenta_FisiereComplete/ardupilot/modules/PX4Firmware/nuttx-configs/px4io-v2/nsh/appconfig
px4io-v2.export: /home/beatrice/Licenta_FisiereComplete/ardupilot/modules/PX4Firmware/nuttx-configs/px4io-v2/nsh/defconfig
px4io-v2.export: /home/beatrice/Licenta_FisiereComplete/ardupilot/modules/PX4Firmware/nuttx-configs/px4io-v2/nsh/setenv.sh
px4io-v2.export: /home/beatrice/Licenta_FisiereComplete/ardupilot/modules/PX4Firmware/nuttx-configs/px4io-v2/scripts/ld.script
px4io-v2.export: /home/beatrice/Licenta_FisiereComplete/ardupilot/modules/PX4Firmware/nuttx-configs/px4io-v2/src/Makefile
px4io-v2.export: /home/beatrice/Licenta_FisiereComplete/ardupilot/modules/PX4Firmware/nuttx-configs/px4io-v2/src/empty.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating ../../../px4io-v2.export"
	cd /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/src/modules/px4iofirmware && /bin/echo Configuring NuttX for px4io-v2
	cd /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/src/modules/px4iofirmware && /usr/bin/make --no-print-directory -C/home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/px4io-v2/NuttX/nuttx -r --quiet distclean
	cd /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/src/modules/px4iofirmware && /bin/cp -r /home/beatrice/Licenta_FisiereComplete/ardupilot/modules/PX4Firmware/nuttx-configs/px4io-v2 /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/px4io-v2/NuttX/nuttx/configs
	cd /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/src/modules/px4iofirmware && cd /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/px4io-v2/NuttX/nuttx/tools && /bin/sh ./configure.sh px4io-v2/nsh
	cd /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/src/modules/px4iofirmware && /bin/echo Exporting NuttX for px4io-v2
	cd /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/src/modules/px4iofirmware && /usr/bin/make --no-print-directory --quiet -C /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/px4io-v2/NuttX/nuttx -j4 -r CONFIG_ARCH_BOARD=px4io-v2 export > /dev/null
	cd /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/src/modules/px4iofirmware && /bin/cp -r /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/px4io-v2/NuttX/nuttx/nuttx-export.zip /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/px4io-v2.export

nuttx_export_px4io-v2: src/modules/px4iofirmware/CMakeFiles/nuttx_export_px4io-v2
nuttx_export_px4io-v2: src/modules/px4iofirmware/nuttx_export_px4io-v2.stamp
nuttx_export_px4io-v2: px4io-v2.export
nuttx_export_px4io-v2: src/modules/px4iofirmware/CMakeFiles/nuttx_export_px4io-v2.dir/build.make

.PHONY : nuttx_export_px4io-v2

# Rule to build all files generated by this target.
src/modules/px4iofirmware/CMakeFiles/nuttx_export_px4io-v2.dir/build: nuttx_export_px4io-v2

.PHONY : src/modules/px4iofirmware/CMakeFiles/nuttx_export_px4io-v2.dir/build

src/modules/px4iofirmware/CMakeFiles/nuttx_export_px4io-v2.dir/clean:
	cd /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/src/modules/px4iofirmware && $(CMAKE_COMMAND) -P CMakeFiles/nuttx_export_px4io-v2.dir/cmake_clean.cmake
.PHONY : src/modules/px4iofirmware/CMakeFiles/nuttx_export_px4io-v2.dir/clean

src/modules/px4iofirmware/CMakeFiles/nuttx_export_px4io-v2.dir/depend:
	cd /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/beatrice/Licenta_FisiereComplete/ardupilot/modules/PX4Firmware /home/beatrice/Licenta_FisiereComplete/ardupilot/modules/PX4Firmware/src/modules/px4iofirmware /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/src/modules/px4iofirmware /home/beatrice/Licenta_FisiereComplete/ardupilot/build/px4-v3/modules/PX4Firmware/src/modules/px4iofirmware/CMakeFiles/nuttx_export_px4io-v2.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/modules/px4iofirmware/CMakeFiles/nuttx_export_px4io-v2.dir/depend


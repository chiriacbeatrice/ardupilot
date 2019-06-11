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

# Utility rule file for xml_gen.

# Include the progress variables for this target.
include CMakeFiles/xml_gen.dir/progress.make

CMakeFiles/xml_gen: parameters.xml
CMakeFiles/xml_gen: airframes.xml


parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/drivers/camera_trigger/camera_trigger_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/drivers/gimbal/gimbal_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/drivers/mkblctrl/mkblctrl_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/drivers/px4fmu/px4fmu_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/drivers/px4io/px4io_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/drivers/rgbled/rgbled_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/examples/fixedwing_control/params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/examples/rover_steering_control/params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/examples/subscriber/subscriber_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/lib/launchdetection/launchdetection_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/lib/runway_takeoff/runway_takeoff_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/modules/attitude_estimator_ekf/attitude_estimator_ekf_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/modules/attitude_estimator_q/attitude_estimator_q_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/modules/bottle_drop/bottle_drop_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/modules/commander/commander_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/modules/controllib/test_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/modules/ekf2/ekf2_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/modules/ekf_att_pos_estimator/ekf_att_pos_estimator_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/modules/fw_att_control/fw_att_control_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/modules/fw_pos_control_l1/fw_pos_control_l1_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/modules/fw_pos_control_l1/mtecs/mTecs_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/modules/land_detector/land_detector_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/modules/local_position_estimator/params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/modules/mavlink/mavlink_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/modules/mc_att_control/mc_att_control_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/modules/mc_att_control_multiplatform/mc_att_control_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/modules/mc_pos_control/mc_pos_control_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/modules/mc_pos_control_multiplatform/mc_pos_control_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/modules/navigator/datalinkloss_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/modules/navigator/geofence_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/modules/navigator/gpsfailure_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/modules/navigator/mission_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/modules/navigator/navigator_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/modules/navigator/rcloss_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/modules/navigator/rtl_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/modules/sdlog2/params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/modules/segway/params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/modules/sensors/sensor_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/modules/systemlib/circuit_breaker_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/modules/systemlib/system_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/modules/uavcan/uavcan_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/modules/vtol_att_control/standard_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/modules/vtol_att_control/tailsitter_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/modules/vtol_att_control/tiltrotor_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/modules/vtol_att_control/vtol_att_control_params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/platforms/qurt/px4_layer/params.c
parameters.xml: /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src/systemcmds/tests/test_params.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/beatricec/LicentaBetty/ardupilot/build/px4-v3/modules/PX4Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating parameters.xml"
	/usr/bin/python /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/Tools/px_process_params.py -s /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/src --board CONFIG_ARCH_px4fmu-v3 --xml --inject-xml

airframes.xml:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/beatricec/LicentaBetty/ardupilot/build/px4-v3/modules/PX4Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating airframes.xml"
	/usr/bin/python /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/Tools/px_process_airframes.py -a /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware/ROMFS/px4fmu_common/init.d --board CONFIG_ARCH_BOARD_px4fmu-v3 --xml

xml_gen: CMakeFiles/xml_gen
xml_gen: parameters.xml
xml_gen: airframes.xml
xml_gen: CMakeFiles/xml_gen.dir/build.make

.PHONY : xml_gen

# Rule to build all files generated by this target.
CMakeFiles/xml_gen.dir/build: xml_gen

.PHONY : CMakeFiles/xml_gen.dir/build

CMakeFiles/xml_gen.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/xml_gen.dir/cmake_clean.cmake
.PHONY : CMakeFiles/xml_gen.dir/clean

CMakeFiles/xml_gen.dir/depend:
	cd /home/beatricec/LicentaBetty/ardupilot/build/px4-v3/modules/PX4Firmware && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware /home/beatricec/LicentaBetty/ardupilot/modules/PX4Firmware /home/beatricec/LicentaBetty/ardupilot/build/px4-v3/modules/PX4Firmware /home/beatricec/LicentaBetty/ardupilot/build/px4-v3/modules/PX4Firmware /home/beatricec/LicentaBetty/ardupilot/build/px4-v3/modules/PX4Firmware/CMakeFiles/xml_gen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/xml_gen.dir/depend


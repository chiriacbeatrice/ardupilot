/****************************************************************************
 *
 *   Copyright (C) 2013-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* Auto-generated by genmsg_cpp from file /home/beatrice/Licenta_FisiereComplete/ardupilot/modules/PX4Firmware/msg/position_setpoint.msg */


#pragma once

#include <stdint.h>
#include <uORB/uORB.h>


#ifndef __cplusplus
#define SETPOINT_TYPE_POSITION 0
#define SETPOINT_TYPE_VELOCITY 1
#define SETPOINT_TYPE_LOITER 2
#define SETPOINT_TYPE_TAKEOFF 3
#define SETPOINT_TYPE_LAND 4
#define SETPOINT_TYPE_IDLE 5
#define SETPOINT_TYPE_OFFBOARD 6
#define SETPOINT_TYPE_FOLLOW_TARGET 7

#endif

/**
 * @addtogroup topics
 * @{
 */


#ifdef __cplusplus
struct __EXPORT position_setpoint_s {
#else
struct position_setpoint_s {
#endif
	bool valid;
	uint8_t type;
	float x;
	float y;
	float z;
	bool position_valid;
	float vx;
	float vy;
	float vz;
	bool velocity_valid;
	double lat;
	double lon;
	float alt;
	float yaw;
	bool yaw_valid;
	bool disable_mc_yaw_control;
	float yawspeed;
	bool yawspeed_valid;
	float loiter_radius;
	int8_t loiter_direction;
	float pitch_min;
	float a_x;
	float a_y;
	float a_z;
	bool acceleration_valid;
	bool acceleration_is_force;
	float acceptance_radius;
	float cruising_speed;
#ifdef __cplusplus
	static const uint8_t SETPOINT_TYPE_POSITION = 0;
	static const uint8_t SETPOINT_TYPE_VELOCITY = 1;
	static const uint8_t SETPOINT_TYPE_LOITER = 2;
	static const uint8_t SETPOINT_TYPE_TAKEOFF = 3;
	static const uint8_t SETPOINT_TYPE_LAND = 4;
	static const uint8_t SETPOINT_TYPE_IDLE = 5;
	static const uint8_t SETPOINT_TYPE_OFFBOARD = 6;
	static const uint8_t SETPOINT_TYPE_FOLLOW_TARGET = 7;

#endif
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(position_setpoint);

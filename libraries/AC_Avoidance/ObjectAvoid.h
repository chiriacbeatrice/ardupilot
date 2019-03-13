/*
 * ObjectAvoid.h
 *
 *  Created on: Mar 11, 2019
 *      Author: beatrice
 */
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>     // AHRS library
#include <AC_AttitudeControl/AC_AttitudeControl.h> // Attitude controller library for sqrt controller
#include <AC_Fence/AC_Fence.h>         // Failsafe fence library
#include <AP_Proximity/AP_Proximity.h>
#include <AP_Beacon/AP_Beacon.h>
#include "Obstacle.h"
#include <vector>
#ifndef LIBRARIES_AC_AVOIDANCE_OBJECTAVOID_H_
#define LIBRARIES_AC_AVOIDANCE_OBJECTAVOID_H_

class ObjectAvoid{

public:
    virtual void adjust_velocity_object(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt);
    virtual void adjust_velocity_object(float kP, float accel_cmss, Vector3f &desired_vel_cms, float dt);

protected:
    std::vector<Obstacle> listOfObstacle;

};




#endif /* LIBRARIES_AC_AVOIDANCE_OBJECTAVOID_H_ */

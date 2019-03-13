/*
 * Obstacle.h
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
#include "AC_Avoid.h"
//#include "ObjectAvoid.h"

#ifndef LIBRARIES_AC_AVOIDANCE_OBSTACLE_H_
#define LIBRARIES_AC_AVOIDANCE_OBSTACLE_H_

class Obstacle {

public:

    Obstacle(const AC_Avoid& avoid):
        _avoid(avoid){}
    /* Do not allow copies */
    Obstacle(const Obstacle &other) = delete;
    Obstacle &operator=(const Obstacle&) = delete;
    virtual ~Obstacle();

    virtual void adjust_velocity_object(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt) = 0;
    //pentru vectorul cu 3 dimensiuni(Vector3f) se poate folosi functia de adjust_velosity din Avoid.cpp


protected:

   const AC_Avoid& _avoid;


};





#endif /* LIBRARIES_AC_AVOIDANCE_OBSTACLE_H_ */

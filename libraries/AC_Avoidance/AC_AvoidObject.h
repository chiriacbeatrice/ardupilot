/*
 * AC_AvoidObject.h
 *
 *  Created on: Mar 5, 2019
 *      Author: beatrice
 */
#include "AC_Avoid.h"

#ifndef LIBRARIES_AC_AVOIDANCE_AC_AVOIDOBJECT_H_
#define LIBRARIES_AC_AVOIDANCE_AC_AVOIDOBJECT_H_

class AC_AvoidObject:public AC_Avoid{
public:
   AC_AvoidObject(const AP_AHRS& ahrs, const AC_Fence& fence,
                   const AP_Proximity& proximity, const AP_Beacon* beacon): AC_Avoid(ahrs,fence,proximity,beacon){}

   void adjust_velocity(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt);

    /* Do not allow copies */

};

#endif /* LIBRARIES_AC_AVOIDANCE_AC_AVOIDOBJECT_H_ */

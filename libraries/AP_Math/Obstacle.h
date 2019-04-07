/*
 * Obstacle.h
 *
 *  Created on: Apr 6, 2019
 *      Author: beatrice
 */

#ifndef LIBRARIES_AP_MATH_OBSTACLE_H_
#define LIBRARIES_AP_MATH_OBSTACLE_H_


//#include "AP_Math/vector2.h"
#include "AP_Math/AP_Math.h"
//#include "AP_Vehicle/AP_Vehicle_Type.h"
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h> // Attitude controller library for sqrt controller
#include <AC_Avoidance/AC_Avoid.h>

class Obstacle {

public:
//
//    Obstacle(const AC_Avoid& avoid):
//        _avoid(avoid){}
//    /* Do not allow copies */
//    Obstacle(const Obstacle &other) = default;
//    Obstacle &operator=(const Obstacle&) = default;
      virtual ~Obstacle(){};

   // virtual void adjust_velocity_object(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt){};
    //pentru vectorul cu 3 dimensiuni(Vector3f) se poate folosi functia de adjust_velosity din Avoid.cpp
      virtual Vector2f adjust_velocity(float kP,Vector2f &currentP,float accel_cmss,
                                       Vector2f &desired_vel_cms, float dt);
protected:
      enum BehaviourTypeObstacle {
          BEHAVIOR_SLIDE_OBSTACLE = 0,
          BEHAVIOR_STOP_OBSTACLE = 1
      };
};



#endif /* LIBRARIES_AP_MATH_OBSTACLE_H_ */

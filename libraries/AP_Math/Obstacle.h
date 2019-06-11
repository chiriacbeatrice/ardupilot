/*
 * Obstacle.h
 *
 *  Created on: Jun 11, 2019
 *      Author: beatricec
 */
#include "AC_Avoidance/AC_Avoid.h"
#ifndef LIBRARIES_AP_MATH_OBSTACLE_H_
#define LIBRARIES_AP_MATH_OBSTACLE_H_

//#include "AP_Math/vector2.h"
#include "AP_Math/AP_Math.h"
//#include "AP_Vehicle/AP_Vehicle_Type.h"
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h> // Attitude controller library for sqrt controller


class Obstacle {

public:
//
//    Obstacle(const AC_Avoid& avoid):
//        _avoid(avoid){}
//    /* Do not allow copies */

    Obstacle(){}
    Obstacle(const Obstacle &other) = default;
    Obstacle &operator=(const Obstacle&) = default;

      virtual ~Obstacle(){};

   // virtual void adjust_velocity_object(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt){};
    //pentru vectorul cu 3 dimensiuni(Vector3f) se poate folosi functia de adjust_velosity din Avoid.cpp
      virtual void adjust_velocity(float kP,Vector2f &currentP,float accel_cmss,
                                       Vector2f &desired_vel_cms, float dt) = 0;
      float get_margin();
      AC_Avoid::BehaviourType get_behavior();
      void setBehaviourStop()
      {
          behaviour = AC_Avoid::BehaviourType::BEHAVIOR_STOP;
      }
      void setBehaviourSlide()
      {
          behaviour = AC_Avoid::BehaviourType::BEHAVIOR_SLIDE;
      }

     virtual Vector2f getStoppingPoint(float kP,float accel_cmss,Vector2f &currentP,
                                        Vector2f &desired_vel_cms);


//protected:

       static AC_Avoid::BehaviourType behaviour;
};



#endif /* LIBRARIES_AP_MATH_OBSTACLE_H_ */

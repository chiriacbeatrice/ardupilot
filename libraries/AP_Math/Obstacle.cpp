/*
 * Obstacle.cpp
 *
 *  Created on: Apr 6, 2019
 *      Author: beatrice
 */
//#if APM_BUILD_TYPE(APM_BUILD_APMrover2)
// # define AP_OBSTACLE_BEHAVE_DEFAULT Obstacle::BehaviourType::BEHAVIOR_STOP
//#else
// # define AP_OBSTACLE_BEHAVE_DEFAULT Obstacle::BehaviourType::BEHAVIOR_SLIDE
//#endif

#include "Obstacle.h"

AC_Avoid::BehaviourType Obstacle::behaviour = AC_Avoid::BehaviourType::BEHAVIOR_SLIDE;

//Vector2f Obstacle::adjust_velocity(float kP,Vector2f &currentP,float accel_cmss,
//        Vector2f &desired_vel_cms, float dt){return desired_vel_cms;};
float Obstacle::get_margin()
{
    /*INS_STILL_THRESH: Stillness threshold for detecting if we are moving
      Note: This parameter is for advanced users
      Threshold to tolerate vibration to determine if vehicle is motionless.
      This depends on the frame type and if there is a constant vibration due
      to motors before launch or after landing. Total motionless is about 0.05.
      Suggested values: Planes/rover use 0.1, multirotors use 1, tradHeli uses 5.

      In ardupilot/ArduCopter/mav.param default this value is 2.5,
      so I consider that a value greater than that means that the copter is in motion.
      */


    // atentie la verificarea asta caci inmod normal ar fi ==0, dar asa e default in cod, asa ca am pus
    // !=0 ca sa pot verifica testul
    if (AC_AVOID_DISABLED != 0)
    {
        return AC_Avoid::get_singleton()->_fence.get_margin();
    }
    else
    {
        return 2.0f;
    }
}

AC_Avoid::BehaviourType Obstacle::get_behavior()
{
//    if(AC_AVOID_DISABLED != 0)
//    {
//        return (AC_Avoid::BehaviourType)AC_Avoid::get_singleton()->_behavior.get();
//    }
//    else
//    {
//      //return AC_Avoid::BehaviourType::BEHAVIOR_SLIDE;
//      //return AC_Avoid::BehaviourType::BEHAVIOR_STOP; //- la testul pentru stop cand nue pe drona codul
//    }
    return behaviour;
}

Vector2f Obstacle::getStoppingPoint(float kP,float accel_cmss,Vector2f &currentP,
                                  Vector2f &desired_vel_cms)
{
    float speed = desired_vel_cms.length();
    Vector2f stopping_point = currentP + desired_vel_cms*( AC_Avoid::get_singleton()->get_stopping_distance(kP, accel_cmss, speed)/speed);

    return stopping_point;
}



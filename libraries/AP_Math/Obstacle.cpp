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

Vector2f Obstacle::adjust_velocity(float kP,Vector2f &currentP,float accel_cmss,
        Vector2f &desired_vel_cms, float dt){return desired_vel_cms;};



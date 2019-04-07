/*
 * ObjectAvoid.cpp
 *
 *  Created on: Apr 6, 2019
 *      Author: beatrice
 */

#include "ObjectAvoid.h"

//void ObjectAvoid::adjust_velocity_object(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt)
//{
//    for(unsigned int i=0;i<listOfObstacle.size();i++)
//    {
//        listOfObstacle[i].adjust_velocity_object(kP, accel_cmss, desired_vel_cms,dt);
//    }
//}

void ObjectAvoid::adjust_velocity(float kP,Vector2f &currentP, float accel_cmss, Vector2f &desired_vel_cms, float dt)
{
    for(unsigned int i=0;i<listOfObstacle.size();i++)
    {
        listOfObstacle[i].adjust_velocity(kP,currentP, accel_cmss, desired_vel_cms,dt);
    }
}

void ObjectAvoid::addObstacle(const Obstacle& value)
{
    listOfObstacle.push_back(value);
}




/*
 * ObstacleAvoid.cpp
 *
 *  Created on: Mar 11, 2019
 *      Author: beatrice
 */
#include <AC_Avoidance/ObjectAvoid.h>

void ObjectAvoid::adjust_velocity_object(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt)
{
    for(unsigned int i=0;i<listOfObstacle.size();i++)
    {
        listOfObstacle[i].adjust_velocity_object(kP, accel_cmss, desired_vel_cms,dt);
    }
}

/*void ObjectAvoid::adjust_velocity_object(float kP, float accel_cmss, Vector3f &desired_vel_cms, float dt)
{
    for(unsigned int i=0;i<listOfObstacle.size();i++)
    {
        listOfObstacle[i].adjust_velocity_object(kP, accel_cmss, desired_vel_cms,dt);
    }
}*/


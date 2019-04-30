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

void ObjectAvoid::createMap()
{
    Vector2f start(44.434665,26.046532);
    Vector2f stop (44.4349,26.046532);
    Line line(start,stop);

    Vector2f centre(44.434600,26.046532);
    Circle circle(5.0f,centre);

    Vector2f A(44.434675,26.047000);
    Vector2f B(44.434680,26.047000);
    Vector2f C(44.434690,26.047300);

    std::vector<Vector2f> points;
    points.push_back(A); points.push_back(B); points.push_back(C);
    PolygonConvex polConvex(points);

    addObstacle(line);
    addObstacle(circle);
    addObstacle(polConvex);


}




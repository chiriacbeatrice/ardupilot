/*
 * PolygonConvex.cpp
 *
 *  Created on: Apr 14, 2019
 *      Author: beatrice
 */
#include "PolygonConvexBC.h"

PolygonConvex::PolygonConvex(std::vector<Vector2f> points):
               _points(points){}

void PolygonConvex::adjust_velocity(float kP,Vector2f &currentP, float accel_cmss,
                                        Vector2f &desired_vel_cms, float dt)
{
    // nu cred ca e okfiindca astfel viteza mi s-arschimba la fiecare iteratie

    uint16_t i, j;
       for (i = 0, j = _points.size()-1; i < _points.size(); j = i++) {

           Vector2f start = _points[j];
           Vector2f stop = _points[i];
           Line line(start,stop);
           line.adjust_velocity(kP,currentP,accel_cmss,desired_vel_cms,dt);
       }
  //  return desired_vel_cms;
}

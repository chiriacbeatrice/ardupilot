/*
 * OpenCurve.cpp
 *
 *  Created on: Apr 19, 2019
 *      Author: beatrice
 */

#include "OpenCurve.h"

OpenCurve::OpenCurve(std::vector<Vector2f> points):
               _points(points){}

void OpenCurve::adjust_velocity(float kP,Vector2f &currentP, float accel_cmss,
                                        Vector2f &desired_vel_cms, float dt)
{
    // nu cred ca e okfiindca astfel viteza mi s-arschimba la fiecare iteratie

    uint16_t i;
       for (i = 0;i < _points.size()-1;i++) {

           Vector2f start = _points[i];
           Vector2f stop = _points[i+1];
           Line line(start,stop);
           line.adjust_velocity(kP,currentP,accel_cmss,desired_vel_cms,dt);
       }
   // return desired_vel_cms;
}

//Vector2f OpenCurve::getStoppingPoint(float kP,float accel_cmss,Vector2f &currentP,
//                                  Vector2f &desired_vel_cms)
//{
//    float speed = desired_vel_cms.length();
//    Vector2f stopping_point = currentP + desired_vel_cms*( AC_Avoid::get_singleton()->get_stopping_distance(kP, accel_cmss, speed)/speed);
//
//    return stopping_point;
//}


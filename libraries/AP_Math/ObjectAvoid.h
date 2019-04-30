/*
 * ObjectAvoid.h
 *
 *  Created on: Apr 6, 2019
 *      Author: beatrice
 */

#ifndef LIBRARIES_AP_MATH_OBJECTAVOID_H_
#define LIBRARIES_AP_MATH_OBJECTAVOID_H_


#include <vector>
#include "Line.h"
#include "Circle.h"
#include "PolygonConvexBC.h"
#include "OpenCurve.h"
#include "Obstacle.h"

class ObjectAvoid
{

public:

    bool mT(){return true;}
    virtual ~ObjectAvoid(){}
    //virtual void adjust_velocity_object(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt);
    virtual void adjust_velocity(float kP,Vector2f &currentP,float accel_cmss,
                                 Vector2f &desired_vel_cms, float dt);
    void addObstacle(const Obstacle& value);
    void createMap();

protected:
    std::vector<Obstacle> listOfObstacle;
};


#endif /* LIBRARIES_AP_MATH_OBJECTAVOID_H_ */

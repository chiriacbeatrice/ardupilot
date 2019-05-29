/*
 * ObjectAvoid.h
 *
 *  Created on: Apr 6, 2019
 *      Author: beatrice
 */

#include <vector>
#include <iostream>

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

//    ObjectAvoid(std::vector<Obstacle *> listOfObstacle);
//    ObjectAvoid(const ObjectAvoid &other) = default;
//    ObjectAvoid &operator=(const ObjectAvoid&) = default;
    virtual ~ObjectAvoid(){}

    //virtual void adjust_velocity_object(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt);
    virtual void adjust_velocity(float kP,Vector2f &currentP,float accel_cmss,
                                 Vector2f &desired_vel_cms, float dt);
    void addObstacle(Obstacle* const & value);
    void createMap();
    virtual Vector2f getStoppingPoint(float kP,float accel_cmss,Vector2f &currentP,
                                            Vector2f &desired_vel_cms);

    bool methodadetest(){ return true; }

//protected:
    std::vector<Obstacle *> _listOfObstacle;
};


#endif /* LIBRARIES_AP_MATH_OBJECTAVOID_H_ */

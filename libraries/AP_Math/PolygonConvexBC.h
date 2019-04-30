/*
 * PolygonConvexBC.h
 *
 *  Created on: Apr 14, 2019
 *      Author: beatrice
 */

#include <vector>

#ifndef LIBRARIES_AP_MATH_POLYGONCONVEXBC_H_
#define LIBRARIES_AP_MATH_POLYGONCONVEXBC_H_

#include "Obstacle.h"
#include "Line.h"

class PolygonConvex: public Obstacle{

public:
    PolygonConvex(std::vector<Vector2f> points);
    PolygonConvex(const PolygonConvex &other) = default;
    PolygonConvex &operator=(const PolygonConvex&) = default;
    virtual ~PolygonConvex(){}
    virtual void adjust_velocity(float kP,Vector2f &currentP, float accel_cmss,
                                         Vector2f &desired_vel_cms, float dt);

protected:
    std::vector<Vector2f> _points;

};





#endif /* LIBRARIES_AP_MATH_POLYGONCONVEXBC_H_ */

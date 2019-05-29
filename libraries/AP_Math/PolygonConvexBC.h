/*
 * PolygonConvexBC.h
 *
 *  Created on: Apr 14, 2019
 *      Author: beatrice
 */

#include <vector>
#include <iostream>

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
                                         Vector2f &desired_vel_cms, float dt) override;

//    Vector2f getStoppingPoint(float kP,float accel_cmss,Vector2f &currentP,
//                                  Vector2f &desired_vel_cms);
    //bool isPolygonComplete();
    Vector2f intersectionOfTwoLines(Vector2f &start1,Vector2f &stop1,Vector2f &start2,Vector2f &stop2);
    Vector2f vecRotate90CW(Vector2f &vec);
    Vector2f vecRotate90CCW(Vector2f &vec);
    bool polygonIsCw();
    std::vector<Vector2f> polygonWithMargin(float margin);
    Vector2f limit_velocity(float kP, float accel_cmss, Vector2f &desired_vel_cms,
                                const Vector2f& limit_direction, float limit_distance_cm, float dt) const;
    bool pointInsideThePolygon(Vector2f &point, PolygonConvex polygon);

protected:
    std::vector<Vector2f> _points;

};





#endif /* LIBRARIES_AP_MATH_POLYGONCONVEXBC_H_ */

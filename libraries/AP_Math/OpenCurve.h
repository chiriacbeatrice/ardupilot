/*
 * OpenCurve.h
 *
 *  Created on: Jun 11, 2019
 *      Author: beatricec
 */
#include <vector>
#ifndef LIBRARIES_AP_MATH_OPENCURVE_H_
#define LIBRARIES_AP_MATH_OPENCURVE_H_

#include "Obstacle.h"
#include "Line.h"
class OpenCurve: public Obstacle{

public:
    OpenCurve(std::vector<Vector2f> points);
    OpenCurve(const OpenCurve &other) = default;
    OpenCurve &operator=(const OpenCurve&) = default;
    virtual ~OpenCurve(){}
    virtual void adjust_velocity(float kP,Vector2f &currentP, float accel_cmss,
                                         Vector2f &desired_vel_cms, float dt) override;

//    Vector2f getStoppingPoint(float kP,float accel_cmss,Vector2f &currentP,
//                                  Vector2f &desired_vel_cms);


protected:
    std::vector<Vector2f> _points;

};




#endif /* LIBRARIES_AP_MATH_OPENCURVE_H_ */

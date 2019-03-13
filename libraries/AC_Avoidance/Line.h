/*
 * Line.h
 *
 *  Created on: Mar 12, 2019
 *      Author: beatrice
 */
#include "Obstacle.h"
#include <AP_Common/AP_Common.h>

#ifndef LIBRARIES_AC_AVOIDANCE_OBJECTAVOID_LINE_H_
#define LIBRARIES_AC_AVOIDANCE_OBJECTAVOID_LINE_H_

class Line : public Obstacle{
public:
    Line(const AC_Avoid& avoid, Vector2f& start, Vector2f& end);
    Line(const Line &other) = delete;
    Line &operator=(const Line&) = delete;
    ~Line();
    void adjust_velocity_object(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt);

protected:
    Vector2f& _start;
    Vector2f& _end;


};




#endif /* LIBRARIES_AC_AVOIDANCE_OBJECTAVOID_LINE_H_ */

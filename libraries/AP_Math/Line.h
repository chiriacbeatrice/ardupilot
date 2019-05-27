/*
 * Line.h
 *
 *  Created on: Apr 6, 2019
 *      Author: beatrice
 */

#ifndef LIBRARIES_AP_MATH_LINE_H_
#define LIBRARIES_AP_MATH_LINE_H_

#include "Obstacle.h"

class Line : public Obstacle{

    friend class LineTest;
    friend class PolygonConvex;
public:
    Line(Vector2f& start, Vector2f& end);

    Line(const Line &other) = default;
    Line &operator=(const Line&) = default;
    virtual ~Line(){}

    virtual void adjust_velocity(float kP,Vector2f &currentP, float accel_cmss,
                                     Vector2f &desired_vel_cms, float dt);

    Vector2f adjust_velocity_Stop(float kP,Vector2f &currentPos, float accel_cmss,
                                   Vector2f &desired_vel_cms,Vector2f &stopping_point,
                                   float dt,float margin_cm);

    Vector2f adjust_velocity_Slide(float kP,Vector2f &currentP, float accel_cmss,
                                     Vector2f &desired_vel_cms, float dt,float margin_cm);

    Vector2f limit_velocity(float kP, float accel_cmss, Vector2f &desired_vel_cms,
                            const Vector2f& limit_direction, float limit_distance_cm, float dt) const;

//    Vector2f getStoppingPoint(float kP,float accel_cmss,Vector2f &currentP,
//                              Vector2f &desired_vel_cms);

    Vector2f get_newPointOnMargin(Vector2f &currentP, float margin);



protected:
    Vector2f& _start;
    Vector2f& _stop;


};

#endif /* LIBRARIES_AP_MATH_LINE_H_ */

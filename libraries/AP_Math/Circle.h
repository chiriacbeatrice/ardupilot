/*
 * Circle.h
 *
 *  Created on: Apr 8, 2019
 *      Author: beatrice
 */

#ifndef LIBRARIES_AP_MATH_CIRCLE_H_
#define LIBRARIES_AP_MATH_CIRCLE_H_

#include "Obstacle.h"

class Circle : public Obstacle
{

public:
    Circle(float radius, Vector2f& centre);

  /* Do not allow copies */
    Circle(const Circle &other) = default;
    Circle &operator=(const Circle&) = default;
    virtual ~Circle(){};

    int methodaTest(int param){ return param+1;}

    virtual Vector2f adjust_velocity(float kP,Vector2f &currentP, float accel_cmss,
                                     Vector2f &desired_vel_cms, float dt);

    Vector2f adjust_velocity_Slide(float kP,Vector2f &currentPos, float accel_cmss,
                                   Vector2f &safe_vel,Vector2f &stopping_point,
                                   float dt,float margin_cm,float radius,float locationDiff, float speed);

    Vector2f adjust_velocity_Stop(float kP,Vector2f &position_xy, float accel_cmss,
                                  Vector2f &desired_vel_cms,Vector2f &stopping_point,
                                  float dt,float margin_cm,float radius,float speed);

protected:

    float _radius;  // in meters
    Vector2f& _centre;
};


#endif /* LIBRARIES_AP_MATH_CIRCLE_H_ */

/*
 * Circle.h
 *
 *  Created on: Jun 11, 2019
 *      Author: beatricec
 */

#ifndef LIBRARIES_AP_MATH_CIRCLE_H_
#define LIBRARIES_AP_MATH_CIRCLE_H_

#include "Obstacle.h"
#include <math.h>
class Circle : public Obstacle
{

public:
    Circle(float radius, Vector2f& centre);

  /* Do not allow copies */
    Circle(const Circle &other) = default;
    Circle &operator=(const Circle&) = default;
    virtual ~Circle(){};

    int methodaTest(int param){ return param+1;}

    virtual void adjust_velocity(float kP,Vector2f &currentP, float accel_cmss,
                                     Vector2f &desired_vel_cms, float dt) override;

    Vector2f adjust_velocity_Slide(float kP,Vector2f &position_xy, float accel_cmss,
                                   Vector2f &desired_vel_cms,Vector2f &stopping_point,
                                   float margin,float speed);


//    Vector2f adjust_velocity_Slide(float kP,Vector2f &position_xy, float accel_cmss,
//                                            Vector2f &desired_vel_cms,
//                                            float dt,float margin_cm, float radius,float locationDiff, float speed);
    Vector2f adjust_velocity_Stop(float kP,Vector2f &position_xy, float accel_cmss,
                                  Vector2f &desired_vel_cms,Vector2f &stopping_point,
                                  float dt,float margin,float speed);




    Vector2f rotate(Vector2f &v, float angle);
    int numberPointsIntersectionOfTwoCircles(Vector2f &centre1, float &radius1,
                                             Vector2f &centre2, float &radius2) const;

    Vector2f limit_velocityCircle(float max_speed,Vector2f &velocity_component);

    void intersectionOfTwoCircles(Vector2f &centre1, float &radius1,
                                  Vector2f &centre2, float &radius2,
                                  Vector2f &intersection1, Vector2f &intersection2);
    void tangentPointsOfCircle(Vector2f &position_xy, Vector2f &centre, float radius,
                               Vector2f &firstTangentPoint,Vector2f &secoundTangentPoint);

    Vector2f rotate_velocity(Vector2f &firstPoint, Vector2f &secoundPoint,
                             Vector2f &stopping_point, Vector2f &position_xy, Vector2f &desired_vel_cms);


protected:

    float _radius;  // in meters
    Vector2f& _centre;
};



#endif /* LIBRARIES_AP_MATH_CIRCLE_H_ */

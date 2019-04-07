///*
// * Circle.h
// *
// *  Created on: Mar 11, 2019
// *      Author: beatrice
// */
//#include "ObjectAvoid.h"
//#include "Obstacle.h"
//#include <AP_Common/AP_Common.h>
//
//
//#ifndef LIBRARIES_AC_AVOIDANCE_CIRCLE_H_
//#define LIBRARIES_AC_AVOIDANCE_CIRCLE_H_
//
//class Circle : public Obstacle
//{
//
//public:
//    Circle(const AC_Avoid& avoid,float& radius, struct Location& centre);
//
//  /* Do not allow copies */
//    Circle(const Circle &other) = default;
//    Circle &operator=(const Circle&) = default;
//    ~Circle();
//
//    void adjust_velocity_object(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt);
//    void setRadius(float value){ _radius = value;}
//
//
//protected:
//
//    float _radius;  // in meters
//    struct Location _centre;
//};
//
//
//
//#endif /* LIBRARIES_AC_AVOIDANCE_CIRCLE_H_ */

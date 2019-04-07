///*
// * Rectangle.h
// *
// *  Created on: Mar 11, 2019
// *      Author: beatrice
// */
//#include "ObjectAvoid.h"
//#include "Obstacle.h"
//#include <AP_Common/AP_Common.h>
//#include <vector>
//
//#ifndef LIBRARIES_AC_AVOIDANCE_RECTANGLE_H_
//#define LIBRARIES_AC_AVOIDANCE_RECTANGLE_H_
//
//class Rectangle: public Obstacle, public  ObjectAvoid{
//
//public:
//    Rectangle(const AC_Avoid& avoid,std::vector<Vector2f> point);
//  /* Do not allow copies */
//    Rectangle(const Rectangle &other) = default;
//    Rectangle &operator=(const Rectangle&) = default;
//    ~Rectangle();
//    bool check_is_rectangle();
//    void adjust_velocity_object(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt);
//
//protected:
//    std::vector<Vector2f> _points;
//};
//
//
//
//#endif /* LIBRARIES_AC_AVOIDANCE_RECTANGLE_H_ */

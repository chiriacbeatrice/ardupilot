/*
 * Line.h
 *
 *  Created on: Apr 6, 2019
 *      Author: beatrice
 */

#ifndef LIBRARIES_AP_MATH_LINE_H_
#define LIBRARIES_AP_MATH_LINE_H_

#include "Obstacle.h"
#include "AC_Fence/AC_Fence.h"

class Line : public Obstacle{

public:
    Line(Vector2f& start, Vector2f& end);

    Line(const Line &other) = default;
    Line &operator=(const Line&) = default;
    virtual ~Line(){}
    int methodTest(int param){if (param == 0) param +=1; return param;}
    int methodTestWithParams(int param){
        param = methodTest(param);
        return 2+param;
    }
    void setStart(Vector2f start){_start = start;}
    void setStop(Vector2f stop){_stop = stop;}

    virtual Vector2f adjust_velocity(float kP,Vector2f &currentP, float accel_cmss,
                                     Vector2f &desired_vel_cms, float dt);

    Vector2f adjust_velocity_Stop(float kP,Vector2f &currentP, float accel_cmss,
                                  Vector2f &safe_vel, float dt,float margin_cm);

    Vector2f adjust_velocity_Slide(float kP,Vector2f &currentPos, float accel_cmss,
                                   Vector2f &safe_vel,Vector2f &stopping_point,
                                   float dt,float margin_cm);

    Vector2f limit_velocity(float kP, float accel_cmss, Vector2f &desired_vel_cms,
                            const Vector2f& limit_direction, float limit_distance_cm, float dt) const;

    //
//    Vector2f limit_velocity(float kP, float accel_cmss, Vector2f &desired_vel_cms,
//                            const Vector2f& limit_direction, float limit_distance_cm,
//                            float dt) const;
//
//    float get_stopping_distance(float kP, float accel_cmss, float speed_cms) const;
//protected:
    Vector2f& _start;
    Vector2f& _stop;


};

#endif /* LIBRARIES_AP_MATH_LINE_H_ */

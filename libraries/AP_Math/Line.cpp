/*
 * Line.cpp
 *
 *  Created on: Apr 6, 2019
 *      Author: beatrice
 */


#include "Line.h"

Line::Line(Vector2f& start, Vector2f& end):
        _start(start),
        _stop(end){}


Vector2f Line::adjust_velocity(float kP,Vector2f &currentP, float accel_cmss,
                               Vector2f &desired_vel_cms, float dt)
{
    // get position as a 2D offset from ahrs home
       Vector2f position_xy;
       position_xy = currentP*100.0f; // m -> cm
       Vector2f safe_vel(desired_vel_cms);
       // calc margin in cm
       const float margin_cm = AC_Avoid::get_singleton()->_fence.get_margin() * 100.0f;

     // for stopping
      const float speed = safe_vel.length();

       //verifiace daca acest stoping_poin e ok???
       //calculul punctului in care ar reusi sa se opreasca
       Vector2f stopping_point = position_xy + safe_vel*(( margin_cm + AC_Avoid::get_singleton()->get_stopping_distance(kP, accel_cmss, speed))/speed);

       if ((AC_Avoid::BehaviourType)AC_Avoid::get_singleton()->_behavior.get()
               == AC_Avoid::BehaviourType::BEHAVIOR_SLIDE) {

         return  adjust_velocity_Slide(kP,position_xy,accel_cmss,safe_vel,stopping_point,dt,margin_cm);

       }else{

         return  adjust_velocity_Stop(kP,position_xy,accel_cmss,safe_vel,dt,margin_cm);

      }
}


Vector2f Line::adjust_velocity_Slide(float kP,Vector2f &currentPos, float accel_cmss,
                                     Vector2f &desired_vel_cms,Vector2f &stopping_point,
                                     float dt,float margin_cm)
{
    Vector2f safe_vel(desired_vel_cms);
    Vector2f intersection;
    if (Vector2f::segment_intersection(currentPos, stopping_point, _start, _stop, intersection)) {
       // vector from current position to point on current edge
       Vector2f limit_direction = intersection - currentPos;
       const float limit_distance_cm = limit_direction.length();
       if (!is_zero(limit_distance_cm)) {
           if (limit_distance_cm <= margin_cm) {
               // we are within the margin so stop vehicle
               safe_vel.zero();
           } else {
               // vehicle inside the given edge, adjust velocity to not violate this edge
               limit_direction /= limit_distance_cm;
               safe_vel = limit_velocity(kP, accel_cmss, safe_vel, limit_direction, MAX(limit_distance_cm - margin_cm, 0.0f), dt);

           }
       }else
           safe_vel.zero();
    }

    return safe_vel;
}


Vector2f Line::adjust_velocity_Stop(float kP,Vector2f &currentPos, float accel_cmss,
                                    Vector2f &desired_vel_cms, float dt,float margin_cm)
{
      Vector2f safe_vel(desired_vel_cms);
    //nu sunt convinsa ca e bine am urmat exemplul de la polygon
      Vector2f limit_direction = Vector2f::closest_point(currentPos, _start, _stop) - currentPos;
      // distance to closest point
      const float limit_distance_cm = limit_direction.length();

      if (!is_zero(limit_distance_cm)) {

        // We are strictly inside the given edge.
        // Adjust velocity to not violate this edge.
          //am facut-o dupa modelul din Avoid.cpp, nu ii inteleg insa rostul. nusut complet convinsa de ultimele doua linii de cod
        limit_direction /= limit_distance_cm;
        safe_vel=limit_velocity(kP, accel_cmss, safe_vel, limit_direction, MAX(limit_distance_cm - margin_cm, 0.0f), dt);

      }
      else
      {
          safe_vel.zero();
      }

      return safe_vel;
}

Vector2f Line::limit_velocity(float kP, float accel_cmss, Vector2f &desired_vel_cms,
                              const Vector2f& limit_direction, float limit_distance_cm, float dt) const
{
    const float max_speed =  AC_Avoid::get_singleton()->get_max_speed(kP, accel_cmss, limit_distance_cm, dt);
    // project onto limit direction
    const float speed = desired_vel_cms * limit_direction;
    if (speed > max_speed) {
        // subtract difference between desired speed and maximum acceptable speed
        desired_vel_cms += limit_direction*(max_speed - speed);
    }
    return desired_vel_cms;
}


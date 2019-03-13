/*
 * Line.cpp
 *
 *  Created on: Mar 12, 2019
 *      Author: beatrice
 */
#include "Line.h"

Line::Line(const AC_Avoid& avoid, Vector2f& start, Vector2f& end):
        Obstacle(avoid),
        _start(start),
        _end(end){}

void Line::adjust_velocity_object(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt)
{
    // get position as a 2D offset from ahrs home
       Vector2f position_xy;
          // return a position relative to home in meters, North/East
         // order. Return true if estimate is valid
       if (!_avoid._ahrs.get_relative_position_NE_home(position_xy)) {

          return;
       }

       position_xy *= 100.0f; // m -> cm
       Vector2f safe_vel(desired_vel_cms);
       // calc margin in cm
       const float margin_cm = _avoid._fence.get_margin() * 100.0f;

     // for stopping
       const float speed = safe_vel.length();

       //verifiace daca acest stoping_poin e ok???
       //calculul punctului in care ar reusi sa se opreasca
       const Vector2f stopping_point = position_xy + safe_vel*(( margin_cm + _avoid.get_stopping_distance(kP, accel_cmss, speed))/speed);

       if ((AC_Avoid::BehaviourType)_avoid._behavior.get() == AC_Avoid::BehaviourType::BEHAVIOR_SLIDE) {
           //nu sunt convinsa ca e bine am urmat exemplul de la polygon
           Vector2f limit_direction = Vector2f::closest_point(position_xy, _start, _end) - position_xy;
           // distance to closest point
           const float limit_distance_cm = limit_direction.length();

           if (!is_zero(limit_distance_cm)) {

             // We are strictly inside the given edge.
             // Adjust velocity to not violate this edge.
               //am facut-o dupa modelul din Avoid.cpp, nu ii inteleg insa rostul. nusut complet convinsa de ultimele doua linii de cod
             limit_direction /= limit_distance_cm;
             _avoid.limit_velocity(kP, accel_cmss, safe_vel, limit_direction, MAX(limit_distance_cm - margin_cm, 0.0f), dt);
           } else {
                 // We are exactly on the edge - treat this as a fence breach.
                 // i.e. do not adjust velocity.
                 return;
           }
       }else{
           // find intersection with line segment
           Vector2f intersection;
             if (Vector2f::segment_intersection(position_xy, stopping_point, _start, _end, intersection)) {
                // vector from current position to point on current edge
                Vector2f limit_direction = intersection - position_xy;
                const float limit_distance_cm = limit_direction.length();
                if (!is_zero(limit_distance_cm)) {
                    if (limit_distance_cm <= margin_cm) {
                        // we are within the margin so stop vehicle
                        safe_vel.zero();
                    } else {
                        // vehicle inside the given edge, adjust velocity to not violate this edge
                        limit_direction /= limit_distance_cm;
                        _avoid.limit_velocity(kP, accel_cmss, safe_vel, limit_direction, MAX(limit_distance_cm - margin_cm, 0.0f), dt);
                    }
                } else {
                    // We are exactly on the edge - treat this as a fence breach.
                    // i.e. do not adjust velocity.
                    return;
                }
            }
        }
}





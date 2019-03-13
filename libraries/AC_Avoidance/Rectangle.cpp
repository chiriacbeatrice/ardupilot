/*
 * Rectangle.cpp
 *
 *  Created on: Mar 11, 2019
 *      Author: beatrice
 */

#include "Rectangle.h"

Rectangle::Rectangle(const AC_Avoid& avoid, std::vector<Vector2f> points):
    Obstacle(avoid),
    _points(points){}

bool Rectangle::check_is_rectangle()
{
    Vector2f centre;
    centre.zero();
    bool ok = true;
    std::vector<float> dd;

    if(_points.size() != 4)
    {
        return false;
    }

    else
    {
       for(unsigned int i=0;i<_points.size();i++)
       {
         centre += _points[i];
       }
       centre /=4;

    for(unsigned int j=0;j<_points.size();j++)
    {
        dd[j] = sqrtf(centre.x-_points[j].y)+sqrtf(centre.x-_points[j].y);
    }

    for(unsigned int j=0;j<dd.size()-1;j++)
    {
        if(dd[j]!= dd[j+1])
        {
            ok =false;
        }
    }
     return ok;
    }
}
void Rectangle::adjust_velocity_object(float kP, float accel_cmss,  Vector2f &desired_vel_cms, float dt)
{
    check_is_rectangle();
    Vector2f position_xy;
    if (!_avoid._ahrs.get_relative_position_NE_home(position_xy)){
             // we have no idea where we are....
             return;
      }

           position_xy *=100; //m -> cm

           Vector2f safe_vel(desired_vel_cms);
           // calc margin in cm
           const float margin_cm = _avoid._fence.get_margin() * 100.0f;

         // for stopping
           const float speed = safe_vel.length();


           //verifiace daca acest stoping_poin e ok???
           //calculul punctului in care ar reusi sa se opreasca
           const Vector2f stopping_point = position_xy + safe_vel*(( margin_cm + _avoid.get_stopping_distance(kP, accel_cmss, speed))/speed);

           uint16_t i, j;
              for (i = 0, j = _points.size()-1; i < _points.size(); j = i++) {

                  Vector2f start = _points[j];
                  Vector2f end = _points[i];

                  if ((AC_Avoid::BehaviourType)_avoid._behavior.get() == AC_Avoid::BehaviourType::BEHAVIOR_SLIDE) {
                       //nu sunt convinsa ca e bine am urmat exemplul de la polygon
                       Vector2f limit_direction = Vector2f::closest_point(position_xy, start, end) - position_xy;
                       // distance to closest point
                       const float limit_distance_cm = limit_direction.length();

                       if (!is_zero(limit_distance_cm)) {
                         // We are strictly inside the given edge.
                         // Adjust velocity to not violate this edge.
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
                         if (Vector2f::segment_intersection(position_xy, stopping_point, start, end, intersection)) {
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


}



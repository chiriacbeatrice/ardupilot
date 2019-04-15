/*
 * Line.cpp
 *
 *  Created on: Apr 6, 2019
 *      Author: beatrice
 */


#include "Line.h"
#include <DataFlash/DataFlash.h>

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

       const float margin_cm = get_margin() * 100.0f;

      //  const float margin = get_margin();
       // for stopping
        const float speed = safe_vel.length();

       //verifiace daca acest stoping_poin e ok???
      //calculul punctului in care ar reusi sa se opreasca
      Vector2f stopping_point_plus_margin = position_xy +
                safe_vel*((margin_cm + AC_Avoid::get_singleton()->get_stopping_distance(kP, accel_cmss, speed))/speed);
      if (get_behavior() == AC_Avoid::BehaviourType::BEHAVIOR_SLIDE) {

          desired_vel_cms = adjust_velocity_Slide (kP,currentP,accel_cmss,desired_vel_cms,dt,margin_cm);

       }
         else{
          desired_vel_cms =adjust_velocity_Stop(kP,currentP,accel_cmss,desired_vel_cms,stopping_point_plus_margin,dt,margin_cm);

       }

    return desired_vel_cms;
}


Vector2f Line::adjust_velocity_Stop(float kP,Vector2f &currentPos, float accel_cmss,
                                     Vector2f &desired_vel_cms,Vector2f &stopping_point,
                                     float dt,float margin_cm)
{
    Vector2f intersection;
    if (Vector2f::segment_intersection(currentPos, (stopping_point/100.0f), _start, _stop, intersection)) {
       // vector from current position to point on current edge
       Vector2f limit_direction = intersection - currentPos;
       const float limit_distance = limit_direction.length();
       if (!is_zero(limit_distance)) {
           if (limit_distance <= (margin_cm/100.0f)) {

               // we are within the margin so stop vehicle
               desired_vel_cms.zero();
           } else {

               // vehicle inside outside the edge, adjust velocity to not violate this edge
               limit_direction /= limit_distance;

               Vector2f limit_direction_cm = limit_direction*100.0f; // m->cm
               const float limit_distance_cm = limit_distance*100.0f; // m->cm

               desired_vel_cms = limit_velocity(kP, accel_cmss, desired_vel_cms, limit_direction_cm, MAX(limit_distance_cm - margin_cm, 0.0f), dt);

           }
       }else
           // the edge(obiectu) has been violated
           desired_vel_cms.zero();
//           DataFlash_Class::instance()->Log_Write("ERROR", "the edge has been violetate-stop",
//                                                "cm/s", // units: seconds, meters
//                                                (Vector2f)safe_vel);
    }

    return desired_vel_cms;
}


Vector2f Line::adjust_velocity_Slide(float kP,Vector2f &currentPos, float accel_cmss,
                                    Vector2f &desired_vel_cms, float dt,float margin_cm)
{

    //nu sunt convinsa ca e bine am urmat exemplul de la polygon
      Vector2f limit_direction = (Vector2f::closest_point(currentPos, _start, _stop) - currentPos);
     // Vector2f limit_direction_cm = limit_direction * 100.0f;
      // distance to closest point
      const float limit_distance = limit_direction.length();

      if (!is_zero(limit_distance)) {

          if(limit_distance <= (margin_cm / 100.0f)){

              desired_vel_cms.zero();
          }else
          {
            // We are strictly outside the given edge.
            // Adjust velocity to not violate this edge.
              //am facut-o dupa modelul din Avoid.cpp, nu ii inteleg insa rostul. nusut complet convinsa de ultimele doua linii de cod

            limit_direction /= limit_distance;

            Vector2f limit_direction_cm = limit_direction*100.0f; // m->cm
            const float limit_distance_cm = limit_distance*100.0f; // m->cm

            desired_vel_cms=limit_velocity(kP, accel_cmss, desired_vel_cms, limit_direction_cm, MAX(limit_distance_cm - margin_cm, 0.0f), dt);
          }
      }
      else
      {
          // We are exactly on the edge - treat this as a fence breach.
          // i.e. stop the vechile
          desired_vel_cms.zero();
//          DataFlash_Class::instance()->Log_Write("ERROR", "the edge has been violetate-slide",
//                                                          "cm/s", // units: seconds, meters
//                                                          (Vector2f)safe_vel);
      }

      return desired_vel_cms;
}


/*
 * Limits the component of desired_vel_cms in the direction of the unit vector
 * limit_direction to be at most the maximum speed permitted by the limit_distance_cm.
 *
 * Uses velocity adjustment idea from Randy's second email on this thread:
 * https://groups.google.com/forum/#!searchin/drones-discuss/obstacle/drones-discuss/QwUXz__WuqY/qo3G8iTLSJAJ
 */
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




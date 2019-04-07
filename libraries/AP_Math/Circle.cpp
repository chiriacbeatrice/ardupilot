///*
// * Circle.cpp
// *
// *  Created on: Mar 11, 2019
// *      Author: beatrice
// */
//#include "Circle.h"
//
//
//Circle::Circle(const AC_Avoid& avoid,float& radius,struct Location& centre):
//    Obstacle(avoid),
//    _radius(radius),
//    _centre(centre){}
//
//void Circle::adjust_velocity_object(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt)
//{
//    // get position as a 2D offset from ahrs home
//      Vector2f position_xy;
//          // return a position relative to home in meters, North/East
//          // order. Return true if estimate is valid
//      if (!_avoid._ahrs.get_relative_position_NE_home(position_xy)){
//          // we have no idea where we are....
//          return;
//      }
//      position_xy *= 100.0f; // m -> cm
//      const float speed = desired_vel_cms.length();
//      const float radius = _radius * 100.0f;  // m -> cm
//        // get the margin to the fence in cm
//      const float margin_cm = _avoid._fence.get_margin() * 100.0f;
//
//
//      struct Location currentPosition;
//      _avoid._ahrs.get_position(currentPosition);
//      float locationDiff = get_distance_cm(currentPosition,_centre);
//
//      if (!is_zero(speed) && locationDiff >= radius) {
//
//          //nu sunt sigura ca e calculat ok PUNCTUL DE STOP
//         Vector2f stopping_point = position_xy + desired_vel_cms*(_avoid.get_stopping_distance(kP, accel_cmss, speed)/speed);
//         float stopping_point_length = stopping_point.length();
//
//         if(locationDiff > radius + margin_cm){
//
//             if ((AC_Avoid::BehaviourType)_avoid._behavior.get() == AC_Avoid::BehaviourType::BEHAVIOR_SLIDE) {
//                const Vector2f target = stopping_point * ((locationDiff-(radius + margin_cm)) / stopping_point_length);
//
//                // nu sunt sigura ca ceea ce urmeaza e ok
//                const Vector2f target_direction = target - position_xy;
//                const float distance_to_target = target_direction.length(); // calculeaza cat trebuie sa inainteze pe baza liniei anterioare de cod
//                const float max_speed =_avoid.get_max_speed(kP, accel_cmss, distance_to_target, dt);  //calculeaza viteza maxima cu care poate inainta
//
//                //M-am gandit la min pentru a incetinii
//                desired_vel_cms = target_direction * (MIN(speed,max_speed) / distance_to_target); // item
//
//             }else{
//                 // shorten vector without adjusting its direction
//                 Vector2f intersection;
//
//                 //!!!!!!!!! nu stiu cat este de corect, dar am presupus ca e bine fiindca Vector2f este un typedef?????
//                 Vector2f centre;
//                 centre.x = _centre.lat; //!!!!!!!
//                 centre.y =_centre.lng;
//                 if (Vector2f::circle_segment_intersection(position_xy, stopping_point,centre,radius, intersection)) {
//                     const float distance_to_target = MAX((intersection - position_xy).length() - margin_cm, 0.0f);
//                     const float max_speed = _avoid.get_max_speed(kP, accel_cmss, distance_to_target, dt);
//                     if (max_speed < speed) {
//                         desired_vel_cms *= MAX(max_speed, 0.0f) / speed;
//                     }
//                 }
//             }
//
//         }
//      }
//
//
//}
//
//

/*
 * Circle.cpp
 *
 *  Created on: Apr 8, 2019
 *      Author: beatrice
 */


#include "Circle.h"


Circle::Circle(float radius,Vector2f& centre):
    _radius(radius),
    _centre(centre){}

Vector2f Circle::adjust_velocity(float kP,Vector2f &currentP, float accel_cmss,
                                 Vector2f &desired_vel_cms, float dt)
{
    // get position as a 2D offset from ahrs home
       Vector2f position_xy;
       Vector2f centre;
       //Vector2f safe_vel(desired_vel_cms);

       position_xy = currentP*100.0f; // m -> cm
       centre = _centre*100.0f; //m->cm

       const float radius = _radius * 100.0f;  // m -> cm
        // get the margin to the fence in cm
       const float margin_cm = get_margin() * 100.0f;
       const float speed = desired_vel_cms.length();

       //locationDiff is the distance between the current position and the centre of circle
       float locationDiff = (position_xy - centre).length();

       if(locationDiff >= radius){
           if(!is_zero(speed)){

             //nu sunt sigura ca e calculat ok PUNCTUL DE STOP
              Vector2f stopping_point = position_xy + desired_vel_cms*( AC_Avoid::get_singleton()->get_stopping_distance(kP, accel_cmss, speed)/speed);
              float stopping_point_length = stopping_point.length();
//
//         if(stopping_point_length >= locationDiff - (radius + margin_cm)){
//
               if (get_behavior()== AC_Avoid::BehaviourType::BEHAVIOR_SLIDE) {

                   desired_vel_cms = adjust_velocity_Slide( kP,position_xy,accel_cmss,desired_vel_cms,stopping_point,
                                               dt,margin_cm, radius,locationDiff,speed);

               }else{

                   desired_vel_cms = adjust_velocity_Stop(kP,position_xy,accel_cmss,desired_vel_cms,stopping_point,
                                             dt,margin_cm,radius,speed);

               }
//
//
//         }
          }
        }else
         {
            desired_vel_cms.zero();
         }

return desired_vel_cms;
}



Vector2f Circle :: adjust_velocity_Slide(float kP,Vector2f &position_xy, float accel_cmss,
                                        Vector2f &desired_vel_cms,Vector2f &stopping_point,
                                        float dt,float margin_cm, float radius,float locationDiff, float speed)
{
    //targetul este proiectia lui SP pe circumferinta cercului,conform  cu
   //  https://www.quora.com/I-am-given-centre-of-a-circle-its-radius-and-a-point-outside-circle-How-can-I-find-a-point-on-circumference-of-circle-corresponding-to-the-point-outside
//    Construct a vector from the circle to the point (P-C)
//
//    Normalise it.
//
//    Multiply it by the circle radius.
//
//    Add it to the circle centre.
//
//    P' = C + (P-C)/|P-C| * r
    // in schimb analizand mai atent am sesizat ca de fapt cu .lenght mi se afla norma vectorului
    // in sistemul de coordonate ce are originea punctul de decolare, deci as merge pe varianta clasica
    // si se merge pe formula matematica v1=lambda*v2, v1 siv2 vectori si se imparte la stopping_point length
    //tocmai pentru a vedea de cate ori intra

   // const Vector2f target = _centre +
                           // ((stopping_point - _centre)/(stopping_point - _centre).length())*(_radius+margin_cm);

    const Vector2f target =  stopping_point * ((locationDiff-(radius + margin_cm)) / stopping_point.length());



    const Vector2f target_direction = target - position_xy;
    const float distance_to_target = target_direction.length(); // calculeaza cat trebuie sa inainteze pe baza liniei anterioare de cod

    const float max_speed =AC_Avoid::get_singleton()->get_max_speed(kP, accel_cmss, distance_to_target, dt);  //calculeaza viteza maxima cu care poate inainta

   //M-am gandit la min pentru a incetinii
    desired_vel_cms = target_direction * (MIN(speed,max_speed) / distance_to_target); // item

    return desired_vel_cms;
}



Vector2f Circle :: adjust_velocity_Stop(float kP,Vector2f &position_xy, float accel_cmss,
                                        Vector2f &desired_vel_cms,Vector2f &stopping_point,
                                        float dt,float margin_cm,float radius,float speed)
{
     //shorten vector without adjusting its direction
     Vector2f intersection;

     if (Vector2f::circle_segment_intersection(position_xy, stopping_point,(_centre*100.0f),radius+margin_cm, intersection)) {
         const float distance_to_target = MAX((intersection - position_xy).length(), 0.0f);
         const float max_speed = AC_Avoid::get_singleton()->get_max_speed(kP, accel_cmss, distance_to_target, dt);
        // if (max_speed < speed) {
             desired_vel_cms *= MAX(max_speed, 0.0f) / speed;
        // }
       }
     return desired_vel_cms;
}


//daca stai sa privesti atent vitezele astea sunt defapt acelasi lucru, doar ca la una fiind vorba de drona intra in discutie target,
//la alta ramane norma vitezei in xOy

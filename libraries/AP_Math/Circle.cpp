/*
 * Circle.cpp
 *
 *  Created on: Jun 11, 2019
 *      Author: beatricec
 */

#include "Circle.h"
//#include <iostream>

Circle::Circle(float radius,Vector2f& centre):
    _radius(radius),
    _centre(centre){}

void Circle::adjust_velocity(float kP,Vector2f &currentP, float accel_cmss,
                                 Vector2f &desired_vel_cms, float dt)
{
    // get position as a 2D offset from ahrs home
       const float margin = get_margin();
       const float speed = desired_vel_cms.length();

       //locationDiff is the distance between the current position and the centre of circle
       float locationDiff = (currentP - _centre).length();


       //std::cout<<"\n\n  Valoarea vitezei inainte de procesare pe X este  "<<desired_vel_cms.x;
       //std::cout<<"\n\n  Valoarea vitezei inainte de procesare pe Y este  "<<desired_vel_cms.y;

       if(locationDiff > (_radius + margin))
       {
           if(!is_zero(speed))
           {
               //std::cout<<"\n\n SPEED E DIFERIT DE 0 \n\n";
             //nu sunt sigura ca e calculat ok PUNCTUL DE STOP
              Vector2f stopping_point = currentP + desired_vel_cms*( AC_Avoid::get_singleton()->get_stopping_distance(kP, accel_cmss, speed)/speed);

              //std::cout<<"\n\nValoarea StoppingPoint X "<<stopping_point.x<<"\n";
              //std::cout<<"Valoarea StoppingPoint Y "<<stopping_point.y<<"\n";


              Vector2f intersectionSegCircle;
              //verificare daca orientarea e spre tinta-> se verifica intersectia linie cerc
              if(Vector2f::circle_segment_intersection(currentP, stopping_point,_centre, (_radius+margin), intersectionSegCircle))
              {
                //  std::cout<<"punct de intersectie X"<<intersectionSegCircle.x;
                  //std::cout<<"punct de intersectie Y"<<intersectionSegCircle.y;
                 // std::cout<<"Distanta de oprire intersecteaza cercul \n\n";

                  if (get_behavior() == AC_Avoid::BehaviourType::BEHAVIOR_SLIDE)
                  {
                   //    std::cout<<"A intrat pe slide";


                       desired_vel_cms = adjust_velocity_Slide(kP,currentP, accel_cmss,
                                                              desired_vel_cms,stopping_point,
                                                              margin,speed);

//                       desired_vel_cms = adjust_velocity_Stop(kP,currentP,accel_cmss,desired_vel_cms,stopping_point,
//                                                             dt,margin,speed);

                  }else{

                     // std::cout<<"A intrat pe stop";
                      desired_vel_cms = adjust_velocity_Stop(kP,currentP,accel_cmss,desired_vel_cms,stopping_point,
                                                         dt,margin,speed);

                  }
               }else{
                   //std::cout<<"Distanta de oprire nu intersecteaza cercul.Nu e necesara ajustarea! \n\n";
               }

           }
        }else
         {
            desired_vel_cms.zero();
         }

}


Vector2f Circle::adjust_velocity_Slide(float kP,Vector2f &position_xy, float accel_cmss,
                                       Vector2f &desired_vel_cms,Vector2f &stopping_point,
                                       float margin,float speed)

{

  //  std::cout<<"\n A intrat in Slide!\n";

    Vector2f firstIntersection,
             secoundIntersection,
             result(desired_vel_cms),
             firstTangentPoint,
             secoundTangentPoint,
             intersectionSegCircle,
             intersectionSegCircleNew;

    float radiusOfObstacle = _radius + margin;


    float stoppingDistance = AC_Avoid::get_singleton()->get_stopping_distance(kP, accel_cmss, speed); // nu sunt sigura ca returneaza in cm si nu in m


    int numberPointOfIntersection = numberPointsIntersectionOfTwoCircles(_centre,radiusOfObstacle,position_xy,stoppingDistance);

    switch(numberPointOfIntersection)
    {
        case 2  :
        {

            intersectionOfTwoCircles(_centre,radiusOfObstacle,position_xy, stoppingDistance,firstIntersection,secoundIntersection);

            result = rotate_velocity(firstIntersection,secoundIntersection,
                                     stopping_point, position_xy,desired_vel_cms);

            Vector2f stopping_point_new = position_xy + result*( AC_Avoid::get_singleton()->get_stopping_distance(kP, accel_cmss, result.length())/result.length());

            if(Vector2f::circle_segment_intersection(position_xy, stopping_point_new,_centre, (_radius+margin), intersectionSegCircleNew))
            {

                tangentPointsOfCircle(position_xy, _centre, radiusOfObstacle,
                                       firstTangentPoint,secoundTangentPoint);

                result = rotate_velocity(firstTangentPoint,secoundTangentPoint,
                                         stopping_point_new, position_xy,result);

            }
         break;
        }

        case -1:
        {

            tangentPointsOfCircle(position_xy, _centre, radiusOfObstacle,
                                   firstTangentPoint,secoundTangentPoint);


            result = rotate_velocity(firstTangentPoint,secoundTangentPoint,
                                    stopping_point, position_xy,desired_vel_cms);

            break;

        }
        case -2:
        {

            result.zero();
            break;
        }
        default:
        {
            break;
        }
    }


    return result;
}


//daca stai sa privesti atent vitezele astea sunt defapt acelasi lucru, doar ca la una fiind vorba de drona intra in discutie target,
//la alta ramane norma vitezei in xOy

Vector2f Circle::rotate(Vector2f& v, float angle)
{
    Vector2f result;

    result.x = v.x*cosf(angle) - v.y*sinf(angle);
    result.y = v.y*cosf(angle) + v.x*sinf(angle);

    return result;
}


int Circle::numberPointsIntersectionOfTwoCircles(Vector2f &centre1, float &radius1,
                                                 Vector2f &centre2, float &radius2)const
{
    float distanceBetweenCentres = (centre1 - centre2).length();
    float sumOfRadius = radius1 + radius2;
    int result;

    if(is_equal(distanceBetweenCentres,sumOfRadius)) // se ating cercurile
    {
        result = 1;
    }
    else if(distanceBetweenCentres>sumOfRadius)  // nu se ating deloc
    {
       result = 0;
    }
    else if(is_equal(distanceBetweenCentres,0.0f) && is_equal(radius1,radius2)) // cercurile coincid
    {
       result = -2;
    }
    else if(distanceBetweenCentres+MIN(radius1,radius2) < MAX(radius1,radius2)) //cercurile se cuprind unul pe altul, caz special
    {
       result = -1;
    }
    else
    {
       result = 2;
    }

   return result;
}

void Circle::intersectionOfTwoCircles(Vector2f &centre1, float &radius1,
                                      Vector2f &centre2, float &radius2,
                                      Vector2f &intersection1, Vector2f &intersection2)
{
    float distanceBetweenCenteres = (centre2-centre1).length();
    float a = (sq(radius1) - sq(radius2) + sq(distanceBetweenCenteres))/(2*distanceBetweenCenteres);
    float h = safe_sqrt(sq(radius1) - sq(a));

    Vector2f intersection;
    intersection.x = centre1.x + a*(centre2.x - centre1.x)/distanceBetweenCenteres;
    intersection.y = centre1.y + a*(centre2.y - centre1.y)/distanceBetweenCenteres;

    intersection1.x = intersection.x +(h*(centre2.y - centre1.y))/distanceBetweenCenteres;
    intersection1.y = intersection.y -(h*(centre2.x - centre1.x))/distanceBetweenCenteres;

    intersection2.x = intersection.x - (h*(centre2.y - centre1.y))/distanceBetweenCenteres;
    intersection2.y = intersection.y + (h*(centre2.x - centre1.x))/distanceBetweenCenteres;

}

void Circle:: tangentPointsOfCircle(Vector2f &position_xy, Vector2f &centre, float radius,
                                   Vector2f &firstTangentPoint,Vector2f &secoundTangentPoint)
{
    float diffCurrentPosCentre = (_centre - position_xy).length();
    float ad = sq(radius/diffCurrentPosCentre);
    float bd = radius/diffCurrentPosCentre * sqrtf(1-ad);
    float dx = position_xy.x - _centre.x;
    float dy = position_xy.y - _centre.y;


    firstTangentPoint.x = _centre.x + ad*dx + bd *dy;
    firstTangentPoint.y = _centre.y + ad*dy - bd *dx;

    secoundTangentPoint.x = _centre.x + ad*dx - bd *dy;
    secoundTangentPoint.y = _centre.y + ad*dy + bd *dx;

}

Vector2f Circle::rotate_velocity(Vector2f &firstPoint, Vector2f &secoundPoint,
                                 Vector2f &stopping_point, Vector2f &position_xy, Vector2f &desired_vel_cms)
{
    Vector2f diffPositionFirstPoint, diffPositionSecoundPoint, diffPosStoppingPoint, result;
    float angle1, angle2; //values are converted in rad


    float firstDistance = (stopping_point - firstPoint).length();
    float secoundDistance = (stopping_point - secoundPoint).length();

    //vector between first intersection poin and current position
    diffPositionFirstPoint = firstPoint - position_xy;

    //vector between secound intersection poin and current position
    diffPositionSecoundPoint = secoundPoint - position_xy;

    //vector between first stopping_poin and current position
    diffPosStoppingPoint = stopping_point - position_xy;

  if(is_equal(MIN(firstDistance,secoundDistance),firstDistance))
     {

        // functia angle imi construieste practic axa de rotatie a = u*v/||u*v|| care e normalizata si apoi scoate unghiul cu arccos(u*v)
        angle1 = diffPosStoppingPoint.angle(diffPositionFirstPoint); // aicitrebuie unghiul dintre distante sau intre viteza si punctele de intersectie???
        result = rotate(desired_vel_cms,angle1);
       // std::cout<<"A intrat pe primul punct de intersectie/tangenta si unghiul este:"<< angle1<<"\n";

     }
   else
     {
         angle2 = diffPosStoppingPoint.angle(diffPositionSecoundPoint);
         result = rotate(desired_vel_cms,-angle2);
        // std::cout<<"A intrat pe al doilea punct de intersectie/tangenta si unghiul este:"<< angle2<<"\n";
     }

  return result;
}


Vector2f Circle :: adjust_velocity_Stop(float kP,Vector2f &position_xy, float accel_cmss,
                                        Vector2f &desired_vel_cms,Vector2f &stopping_point,
                                        float dt,float margin,float speed)
{
    //std::cout<<"A intrat in Stop!";
     //shorten vector without adjusting its direction
     Vector2f intersection;

     if (Vector2f::circle_segment_intersection(position_xy, stopping_point,_centre,_radius+margin, intersection)) {
         const float distance_to_target = MAX((intersection - position_xy).length(), 0.0f);
         const float max_speed = AC_Avoid::get_singleton()->get_max_speed(kP, accel_cmss, distance_to_target, dt);
        // if (max_speed < speed) {
             desired_vel_cms *= MAX(max_speed, 0.0f) / speed;

        // std::cout<<"\n\n  Valoarea vitezei dupa procesare pe X este  "<<desired_vel_cms.x;
        // std::cout<<"\n\n  Valoarea vitezei dupa procesare pe Y este  "<<desired_vel_cms.y;
        // }

       }
     return desired_vel_cms;
}

//Se cunos  cdupa mine max_speed care este o norma  si componenta vitezei ce este suprapusa pe max speed
//de  aici va iesi scalarul lambda care este legat de formula a doi vectori suprapusi
//v1 = lambda*vmax(1); daca aplic norma peste tot si tinand cont de proprietatile normei
//||v1||=|lamba|*||vmax||(2), cei doi vectori au acelasi sens, deci lambda va fi mereu pozitiv, prin urmare il
//il pot scoate pe lambda si inlocui in ecutia 1, astfel aflu vectorul vmax, caruia ii stiam doar norma
//dupa  asta scad din v1 pe vmax si aflu componenta  suplimentara a vitezei pe care cu regula triunghiului o
//adun la v  initial, astfel iese viteza ajustata;


Vector2f Circle::limit_velocityCircle(float max_speed,Vector2f &velocity_component)
{
    Vector2f max_speed_Vector = velocity_component*(max_speed/velocity_component.length());

    Vector2f velocity_additional = velocity_component - max_speed_Vector;

   return velocity_additional;

}

 /*
 * Circle.cpp
 *
 *  Created on: Apr 8, 2019
 *      Author: beatrice
 */


#include "Circle.h"
#include <iostream>

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


       std::cout<<"\n\n  Valoarea vitezei inainte de procesare pe X este  "<<desired_vel_cms.x;
       std::cout<<"\n\n  Valoarea vitezei inainte de procesare pe Y este  "<<desired_vel_cms.y;

       if(locationDiff > (_radius + margin))
       {
           if(!is_zero(speed))
           {
               std::cout<<"\n\n SPEED E DIFERIT DE 0 \n\n";
             //nu sunt sigura ca e calculat ok PUNCTUL DE STOP
              Vector2f stopping_point = currentP + desired_vel_cms*( AC_Avoid::get_singleton()->get_stopping_distance(kP, accel_cmss, speed)/speed);

              std::cout<<"\n\nValoarea StoppingPoint X "<<stopping_point.x<<"\n";
              std::cout<<"Valoarea StoppingPoint Y "<<stopping_point.y<<"\n";


              Vector2f intersectionSegCircle;
              //verificare daca orientarea e spre tinta-> se verifica intersectia linie cerc
              if(Vector2f::circle_segment_intersection(currentP, stopping_point,_centre, (_radius+margin), intersectionSegCircle))
              {
                  std::cout<<"punct de intersectie X"<<intersectionSegCircle.x;
                  std::cout<<"punct de intersectie Y"<<intersectionSegCircle.y;
                  std::cout<<"Distanta de oprire intersecteaza cercul \n\n";

                  if (get_behavior() == AC_Avoid::BehaviourType::BEHAVIOR_SLIDE)
                  {
                       std::cout<<"A intrat pe slide";


                       desired_vel_cms = adjust_velocity_Slide(kP,currentP, accel_cmss,
                                                              desired_vel_cms,stopping_point,
                                                              margin,speed);

//                       desired_vel_cms = adjust_velocity_Slide(kP,currentP, accel_cmss,
//                                             desired_vel_cms,
//                                             dt,margin,_radius,locationDiff,speed);

                  }else{

                      std::cout<<"A intrat pe stop";
                      desired_vel_cms = adjust_velocity_Stop(kP,currentP,accel_cmss,desired_vel_cms,stopping_point,
                                                         dt,margin,speed);

                  }
               }else{
                   std::cout<<"Distanta de oprire nu intersecteaza cercul.Nu e necesara ajustarea! \n\n";
               }

           }
        }else
         {
            desired_vel_cms.zero();
         }

//return desired_vel_cms;
       std::cout<<"\n\n  Valoarea vitezei dupa procesare pe X este  "<<desired_vel_cms.x;
       std::cout<<"\n\n  Valoarea vitezei dupa procesare pe Y este  "<<desired_vel_cms.y;
}



//Vector2f Circle :: adjust_velocity_Slide(float kP,Vector2f &position_xy, float accel_cmss,
//                                        Vector2f &desired_vel_cms,
//                                        float dt,float margin_cm, float radius,float locationDiff, float speed)
//{
//    //targetul este proiectia lui SP pe circumferinta cercului,conform  cu
//   //  https://www.quora.com/I-am-given-centre-of-a-circle-its-radius-and-a-point-outside-circle-How-can-I-find-a-point-on-circumference-of-circle-corresponding-to-the-point-outside
////    Construct a vector from the circle to the point (P-C)
////
////    Normalise it.
////
////    Multiply it by the circle radius.
////
////    Add it to the circle centre.
////
////    P' = C + (P-C)/|P-C| * r
//    // in schimb analizand mai atent am sesizat ca de fapt cu .lenght mi se afla norma vectorului
//    // in sistemul de coordonate ce are originea punctul de decolare, deci as merge pe varianta clasica
//    // si se merge pe formula matematica v1=lambda*v2, v1 siv2 vectori si se imparte la stopping_point length
//    //tocmai pentru a vedea de cate ori intra
//
//   // const Vector2f target = _centre +
//                           // ((stopping_point - _centre)/(stopping_point - _centre).length())*(_radius+margin_cm);
////
////    const Vector2f target =  stopping_point * ((locationDiff-(radius + margin_cm)) / stopping_point.length());
////
////
////
////    const Vector2f target_direction = target - position_xy;
////    const float distance_to_target = target_direction.length(); // calculeaza cat trebuie sa inainteze pe baza liniei anterioare de cod
////
////    const float max_speed =AC_Avoid::get_singleton()->get_max_speed(kP, accel_cmss, distance_to_target, dt);  //calculeaza viteza maxima cu care poate inainta
////
////   //M-am gandit la min pentru a incetinii
////    desired_vel_cms = target_direction * (MIN(speed,max_speed) / distance_to_target); // item
////
//
//
//    //rotatia cu unghiul alpha pe tangenta
//    Vector2f t1,t2;
//    Vector2f centre = _centre;
//
//    //Unghiul dintre  tangenta si distanta pozitie curenta->centru
//    float alpha = safe_asin((radius+margin_cm)/locationDiff);
//
//    Vector2f locationVector = position_xy - centre;
//    Vector2f locationVectorNorm = locationVector/locationDiff;
//
//    //Rotirea vectorului normat cu unghiul alpha in sens trigonometric
//    t1 = rotate(locationVectorNorm,alpha);
//    //Rotirea vectorului normat cu unghiul alpha in sens antitrigonometric
//    t2 = rotate(locationVectorNorm,-alpha);
//
//    Vector2f velocityProjection1 = desired_vel_cms.projected(t1);
//    Vector2f velocityProjection2 = desired_vel_cms.projected(t2);
//
//    // cum distantele de la pozitia curenta pana la punctele de tangenta sunt egale le-am scos din cosinus
//
//    float distanceToTangentialPoint = locationDiff*cos(alpha);
//    const float max_speed =
//                    AC_Avoid::get_singleton()->get_max_speed(kP, accel_cmss, distanceToTangentialPoint, dt);
//
//    if(velocityProjection1.length() >=  velocityProjection2.length())
//     {
//
//        //desired_vel_cms *= (MIN(velocityProjection1.length(),max_speed) / distanceToTangentialPoint);
//          desired_vel_cms = desired_vel_cms + limit_velocityCircle(max_speed,velocityProjection1);
//     }else
//     {
//        //desired_vel_cms *= (MIN(velocityProjection2.length(),max_speed) / distanceToTangentialPoint);
//          desired_vel_cms = desired_vel_cms + limit_velocityCircle(max_speed,velocityProjection2);
//     }
//
//    return desired_vel_cms;
//}
Vector2f Circle::adjust_velocity_Slide(float kP,Vector2f &position_xy, float accel_cmss,
                                       Vector2f &desired_vel_cms,Vector2f &stopping_point,
                                       float margin,float speed)

{

    std::cout<<"\n A intrat in Slide!\n";

    Vector2f firstIntersection,
             secoundIntersection,
             result(desired_vel_cms),
             firstTangentPoint,
             secoundTangentPoint,
             intersectionSegCircle,
             intersectionSegCircleNew;

    float radiusOfObstacle = _radius + margin;


    float stoppingDistance = AC_Avoid::get_singleton()->get_stopping_distance(kP, accel_cmss, speed); // nu sunt sigura ca returneaza in cm si nu in m

    std::cout<<"Raza si marginea "<< radiusOfObstacle<<"\n";
    std::cout<<"Distanta de oprire "<< stoppingDistance<<"\n";
    std::cout<<"Centru x"<< _centre.x<<"\n";
    std::cout<<"Centru y"<< _centre.y<<"\n";

    int numberPointOfIntersection = numberPointsIntersectionOfTwoCircles(_centre,radiusOfObstacle,position_xy,stoppingDistance);

    switch(numberPointOfIntersection)
    {
        case 2  :
        {

            std::cout<<"Numarul de puncte este 2"<<"\n";
            intersectionOfTwoCircles(_centre,radiusOfObstacle,position_xy, stoppingDistance,firstIntersection,secoundIntersection);

            std::cout<<"first intersection X"<<firstIntersection.x<<"\n";
            std::cout<<"first intersection Y"<<firstIntersection.y<<"\n";

            std::cout<<"secound intersection X"<<secoundIntersection.x<<"\n";
            std::cout<<"secound intersection Y"<<secoundIntersection.y<<"\n";

            result = rotate_velocity(firstIntersection,secoundIntersection,
                                     stopping_point, position_xy,desired_vel_cms);


            std::cout<<"\n\n  Valoarea vitezei dupa prima rotatie pe X este  "<<result.x;
            std::cout<<"\n\n  Valoarea vitezei dupa prima rotatie pe Y este  "<<result.y;
            Vector2f stopping_point_new = position_xy + result*( AC_Avoid::get_singleton()->get_stopping_distance(kP, accel_cmss, result.length())/result.length());

            std::cout<<"\n\n  Valoarea noului punct de stop X "<<stopping_point_new.x;
            std::cout<<"\n\n  Valoarea noului punct de stop Y "<<stopping_point_new.y;

            if(Vector2f::circle_segment_intersection(position_xy, stopping_point_new,_centre, (_radius+margin), intersectionSegCircleNew))
            {
                std::cout<<"A intrat in intersectie";
                tangentPointsOfCircle(position_xy, _centre, radiusOfObstacle,
                                       firstTangentPoint,secoundTangentPoint);

                result = rotate_velocity(firstTangentPoint,secoundTangentPoint,
                                         stopping_point_new, position_xy,result);

            }
         break;
        }

        case -1:
        {
            std::cout<<"Un cerc contine un altul\n";
            //distanta dintre pozitia curenta si centru
            tangentPointsOfCircle(position_xy, _centre, radiusOfObstacle,
                                   firstTangentPoint,secoundTangentPoint);

            std::cout<<"first tangentPoint X"<<firstTangentPoint.x<<"\n";
            std::cout<<"first tangentPoint Y"<<firstTangentPoint.y<<"\n";

            std::cout<<"secound tangentPoint X"<<secoundTangentPoint.x<<"\n";
            std::cout<<"secound tangentPoint Y"<<secoundTangentPoint.y<<"\n";


            result = rotate_velocity(firstTangentPoint,secoundTangentPoint,
                                    stopping_point, position_xy,desired_vel_cms);

            break;

        }
        case -2:
        {
            std::cout<<"Cercurile coincid";
            result.zero();
            break;
        }
        default:
        {
            std::cout<<"Nu e necesara modificarea vitezei";
        }
    }


    return result;
}


//daca stai sa privesti atent vitezele astea sunt defapt acelasi lucru, doar ca la una fiind vorba de drona intra in discutie target,
//la alta ramane norma vitezei in xOy

Vector2f Circle::rotate(Vector2f& v, float angle)
{
    Vector2f result;

    result.x = v.x*cos(angle) - v.y*sin(angle);
    result.y = v.y*cos(angle) + v.x*sin(angle);

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

   return result; // cazul in care se cuprind unul pe altul nu a fost tratat
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

    std::cout<<"intersection X: "<<intersection.x<<"\n";
    std::cout<<"intersection Y: "<<intersection.y<<"\n";

    intersection1.x = intersection.x +(h*(centre2.y - centre1.y))/distanceBetweenCenteres;
    intersection1.y = intersection.y -(h*(centre2.x - centre1.x))/distanceBetweenCenteres;

    intersection2.x = intersection.x - (h*(centre2.y - centre1.y))/distanceBetweenCenteres;
    intersection2.y = intersection.y + (h*(centre2.x - centre1.x))/distanceBetweenCenteres;


    std::cout<<"first intersection X in function: "<<intersection1.x<<"\n";
    std::cout<<"first intersection Y in function: "<<intersection1.y<<"\n";

    std::cout<<"secound intersection X in function: "<<intersection2.x<<"\n";
    std::cout<<"secound intersection Y in function: "<<intersection2.y<<"\n";

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

    std::cout<<"first tangentPoint in function X"<<firstTangentPoint.x<<"\n";
    std::cout<<"first tangentPoint in function Y"<<firstTangentPoint.y<<"\n";

    std::cout<<"secound tangentPoint in function X"<<secoundTangentPoint.x<<"\n";
    std::cout<<"secound tangentPoint in function Y"<<secoundTangentPoint.y<<"\n";
}

Vector2f Circle::rotate_velocity(Vector2f &firstPoint, Vector2f &secoundPoint,
                                 Vector2f &stopping_point, Vector2f &position_xy, Vector2f &desired_vel_cms)
{
    Vector2f diffPositionFirstPoint, diffPositionSecoundPoint, diffPosStoppingPoint, result;
    float angle1, angle2; //sunt  intoarse valori in radiani


    float firstDistance = (stopping_point - firstPoint).length();
    float secoundDistance = (stopping_point - secoundPoint).length();


    //vector between first intersection poin and current position
    diffPositionFirstPoint = firstPoint - position_xy;

    //vector between secound intersection poin and current position
    diffPositionSecoundPoint = secoundPoint - position_xy;

    //vector between first stopping_poin and current position
    diffPosStoppingPoint = stopping_point - position_xy;

  std::cout<<"first distance "<<firstDistance<<"\n";
  std::cout<<"secound distance "<<secoundDistance<<"\n";


  if(is_equal(MIN(firstDistance,secoundDistance),firstDistance))
     {

        // functia angle imi construieste practic axa de rotatie a = u*v/||u*v|| care e normalizata si apoi scoate unghiul cu arccos(u*v)
        angle1 = diffPosStoppingPoint.angle(diffPositionFirstPoint); // aicitrebuie unghiul dintre distante sau intre viteza si punctele de intersectie???
        result = rotate(desired_vel_cms,angle1);
        std::cout<<"A intrat pe primul punct de intersectie/tangenta si unghiul este:"<< angle1<<"\n";

     }
   else
     {
         angle2 = diffPosStoppingPoint.angle(diffPositionSecoundPoint);
         result = rotate(desired_vel_cms,-angle2);
         std::cout<<"A intrat pe al doilea punct de intersectie/tangenta si unghiul este:"<< angle2<<"\n";
     }

  return result;
}


Vector2f Circle :: adjust_velocity_Stop(float kP,Vector2f &position_xy, float accel_cmss,
                                        Vector2f &desired_vel_cms,Vector2f &stopping_point,
                                        float dt,float margin,float speed)
{
    std::cout<<"A intrat in Stop!";
     //shorten vector without adjusting its direction
     Vector2f intersection;

     if (Vector2f::circle_segment_intersection(position_xy, stopping_point,_centre,_radius+margin, intersection)) {
         const float distance_to_target = MAX((intersection - position_xy).length(), 0.0f);
         const float max_speed = AC_Avoid::get_singleton()->get_max_speed(kP, accel_cmss, distance_to_target, dt);
        // if (max_speed < speed) {
             desired_vel_cms *= MAX(max_speed, 0.0f) / speed;

         std::cout<<"\n\n  Valoarea vitezei dupa procesare pe X este  "<<desired_vel_cms.x;
         std::cout<<"\n\n  Valoarea vitezei dupa procesare pe Y este  "<<desired_vel_cms.y;
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



/*
 * PolygonConvex.cpp
 *
 *  Created on: Apr 14, 2019
 *      Author: beatrice
 */
#include "PolygonConvexBC.h"

PolygonConvex::PolygonConvex(std::vector<Vector2f> points):
               _points(points){}

void PolygonConvex::adjust_velocity(float kP,Vector2f &currentP, float accel_cmss,
                                        Vector2f &desired_vel_cms, float dt)
{
    // nu cred ca e okfiindca astfel viteza mi s-arschimba la fiecare iteratie

    std::cout<<"A intrat in functie \n";
    uint16_t i, j;
    std::vector<Line> segments;
    std::vector<Vector2f>polygonWithMargins;
    Vector2f intersection;

    Vector2f safe_vel(desired_vel_cms);

    const float margin_cm = get_margin();

    // for stopping
    const float speed = safe_vel.length();


    std::cout<<"\n\n  Valoarea vitezei inainte de procesare pe X este  "<<desired_vel_cms.x;
    std::cout<<"\n\n  Valoarea vitezei inainte de procesare pe Y este  "<<desired_vel_cms.y;

    Vector2f stopping_point = currentP +
              safe_vel*((AC_Avoid::get_singleton()->get_stopping_distance(kP, accel_cmss, speed))/speed);

    std::cout<<"\n\nValoarea StoppingPoint inainte X "<<stopping_point.x<<"\n";
    std::cout<<"Valoarea StoppingPoint inainte Y "<<stopping_point.y<<"\n";

    std::cout<<"Poligon vechi \n";
    for(uint16_t i1 =0; i1<_points.size();i1++)
     {
        std::cout<<"Se afiseaza poligonul initial.\n";
        std::cout<<"poligon_initial = "<<_points[i1].x<<" "<<_points[i1].y<<"\n";
     }

    polygonWithMargins = polygonWithMargin(margin_cm);
    std::cout<<"A creat poligonul \n";

    for(uint16_t i1 =0; i1<polygonWithMargins.size();i1++)
   {
       std::cout<<"Se afiseaza poligonul mare.\n";
       std::cout<<"poligon_nou = "<<polygonWithMargins[i1].x<<" "<<polygonWithMargins[i1].y<<"\n";
   }




    if(!pointInsideThePolygon(currentP,polygonWithMargins))
    {
       std::cout<<"A ajuns inainte de for.\n";
       for (i = 0, j = polygonWithMargins.size()-1; i < polygonWithMargins.size(); j = i++) {

           Vector2f start = polygonWithMargins[j];
           Vector2f stop = polygonWithMargins[i];

           if (get_behavior() == AC_Avoid::BehaviourType::BEHAVIOR_SLIDE)
           {
               Vector2f limit_direction = (Vector2f::closest_point(currentP, start, stop) - currentP);

               const float limit_distance = limit_direction.length();

                if (!is_zero(limit_distance))
                {
                    if(limit_distance <= (margin_cm))
                    {
                        desired_vel_cms.zero();
                    }else
                     {
                        limit_direction /= limit_distance;

                         Vector2f limit_direction_cm = limit_direction; // m->cm
                         const float limit_distance_cm = limit_distance; // m->cm

                           desired_vel_cms=limit_velocity(kP, accel_cmss, desired_vel_cms, limit_direction_cm, MAX(limit_distance_cm - margin_cm, 0.0f), dt);
                       }
                  }
                  else
                   {

                      desired_vel_cms.zero();

                   }
           }
           else
           {
               if (Vector2f::segment_intersection(currentP, stopping_point, start, stop, intersection))
               {

                      Vector2f limit_direction = intersection - currentP;
                      const float limit_distance = limit_direction.length();
                      if (!is_zero(limit_distance))
                      {

                        limit_direction /= limit_distance;
                        desired_vel_cms = limit_velocity(kP, accel_cmss, desired_vel_cms, limit_direction, MAX(limit_distance, 0.0f), dt);

                       }else
                       {
                          desired_vel_cms.zero();
                       }
              }


       }

    }
  }else
  {
     desired_vel_cms.zero();
   }
}


Vector2f PolygonConvex::getStoppingPoint(float kP,float accel_cmss,Vector2f &currentP,
                                  Vector2f &desired_vel_cms)
{
    float speed = desired_vel_cms.length();
    Vector2f stopping_point = currentP + desired_vel_cms*( AC_Avoid::get_singleton()->get_stopping_distance(kP, accel_cmss, speed)/speed);

    return stopping_point;
}


// this function determinate the intersection of two lines
Vector2f PolygonConvex::intersectionOfTwoLines(Vector2f &start1,Vector2f &stop1,Vector2f &start2,Vector2f &stop2)
{
    Vector2f result;

    float a1 = stop1.y-start1.y;
    float b1 = start1.x-stop1.x;
    float c1 = a1*start1.x + b1*start1.y;

    float a2 = stop2.y-start2.y;
    float b2 = start2.x-stop2.x;
    float c2 = a2*start2.x + b2*start2.y;

    float determinant = a1*b2 - a2*b1;

    if(is_equal(determinant,0.0f))
    {
        // In this case lines are parallel
        result.x = FLT_MAX;
        result.y = FLT_MAX;
    }
    else
    {
        result.x = (b2*c1-b1*c2)/determinant;
        result.y = (a1*c2-a2*c1)/determinant;
    }

    return result;
}

// this function rotate a vector with 90 degree in CW sense
Vector2f PolygonConvex::vecRotate90CW(Vector2f &vec)
{
    Vector2f result;
    result.x = vec.y;
    result.y = -vec.x;
    return result;

}

// this function rotate a vector with 90 degree in CCW sense
Vector2f PolygonConvex::vecRotate90CCW(Vector2f &vec)
{
    Vector2f result;
    result.x = -vec.y;
    result.y = vec.x;
    return result;

}

// this function return true if the points whitch form the polygon are inside the vector in a CW sense
bool PolygonConvex::polygonIsCw()
{
    Vector2f first = _points[1] - _points[0];
    Vector2f first1 = vecRotate90CW(first);
    Vector2f secound = _points[2] - _points[1];
    if (first1*secound >= 0)
        return true;
    else
        return false;

}

//this function has the capability to expand with a distance a polygon
std::vector<Vector2f> PolygonConvex::polygonWithMargin(float margin)
{

    std::vector<Vector2f>polygonWithMargins;
    Vector2f d01, d12, rotation01,rotation12;

    for (uint16_t i = 0; i< _points.size(); ++i)
    {
        Vector2f pt0 = _points[(i > 0) ? i-1 : _points.size()-1];
        Vector2f pt1 = _points[i];
        Vector2f pt2 = _points[(i<_points.size()-1)? i+1:0];
        Vector2f var01 = pt0 - pt1;
        Vector2f var12 = pt1 - pt2;

        if(polygonIsCw())
        {
            std::cout<<"Rotatie CW \n";
            rotation01 = vecRotate90CW(var01);
            rotation12 = vecRotate90CW(var12);

        }else
        {   std::cout<<"Rotatie CCW \n";
            rotation01 = vecRotate90CCW(var01);
            rotation12 = vecRotate90CCW(var12);

        }

        d01 = (rotation01/rotation01.length())*margin;
        d12 = (rotation12/rotation12.length())*margin;
        Vector2f ptx0 = pt0 + d01;
        Vector2f ptx10 = pt1 + d01;
        Vector2f ptx12 = pt1 + d12;
        Vector2f ptx2 = pt2 + d12;

      polygonWithMargins.push_back(intersectionOfTwoLines(ptx0,ptx10,ptx2,ptx12));
    }

    return polygonWithMargins;
}

Vector2f PolygonConvex::limit_velocity(float kP, float accel_cmss, Vector2f &desired_vel_cms,
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


//this function returns true if a given point is inside of a convex polygon
bool PolygonConvex::pointInsideThePolygon(Vector2f &point, PolygonConvex polygon)
{

    bool inside = false;
    uint16_t i,j;

    float  minX = polygon._points[0].x;
    float  maxX = polygon._points[0].x;
    float  minY = polygon._points[0].y;
    float  maxY = polygon._points[0].y;

    for (uint16_t k =1 ; k< polygon._points.size(); k++)
    {
        Vector2f aux = polygon._points[k];
        minX = MIN(aux.x,minX);
        maxX = MAX(aux.x,maxX);
        minY = MIN(aux.y,minY);
        maxY = MAX(aux.y,maxY);
    }

    if(point.x < minX || point.x > maxX || point.y < minY || point.y > maxY)
    {
        return false;
    }

    for (i = 0, j = polygon._points.size()-1; i < polygon._points.size(); j = i++)
    {
       if ( (polygon._points[i].y > point.y) != (polygon._points[j].y > point.y)
           && (point.x < ((polygon._points[j].x - polygon._points[i].x)*(point.y - polygon._points[i].y) /
                        (polygon._points[j].y - polygon._points[i].y) + polygon._points[i].x)))
       {
           inside = !inside;
       }
    }

    std::cout<<"%d este in poligon este: %d" << inside <<"\n";
    return inside;
}

/*
 * ObjectAvoid.cpp
 *
 *  Created on: Jun 11, 2019
 *      Author: beatricec
 */

#include "ObjectAvoid.h"

//ObjectAvoid::ObjectAvoid(std::vector<Obstacle*> listOfObstacle):
//                         _listOfObstacle(listOfObstacle){}


//void ObjectAvoid::adjust_velocity_object(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt)
//{
//    for(unsigned int i=0;i<listOfObstacle.size();i++)
//    {
//        listOfObstacle[i].adjust_velocity_object(kP, accel_cmss, desired_vel_cms,dt);
//    }
//}


void ObjectAvoid::adjust_velocity(float kP,Vector2f &currentP, float accel_cmss, Vector2f &desired_vel_cms, float dt)
{
    for(uint16_t i=0;i<_listOfObstacle.size();i++)
    {
        _listOfObstacle[i]->adjust_velocity(kP,currentP, accel_cmss, desired_vel_cms,dt);
    }
}

void ObjectAvoid::addObstacle(Obstacle* const & value)
{
    _listOfObstacle.push_back(value);
}

Vector2f ObjectAvoid::location_to_xy(Location loc2)
{
    static Location loc1;
    loc1.lat = 44.434973*1e7;
    loc1.lng = 26.047265*1e7;

        return Vector2f((loc2.lat - loc1.lat) * LOCATION_SCALING_FACTOR,
                        (loc2.lng - loc1.lng) * LOCATION_SCALING_FACTOR * longitude_scale(loc1));


}


Vector2f ObjectAvoid::location_to_xy(double lat, double lng)
{
    static Location loc;
    loc.lat = lat*(double)1e7;
    loc.lng = lng*(double)1e7;

        return location_to_xy(loc);
}

float ObjectAvoid::longitude_scale(const struct Location &loc)
{
    float scale = cosf(loc.lat * (1.0e-7f * DEG_TO_RAD));
    return constrain_float(scale, 0.01f, 1.0f);
}



void ObjectAvoid::createMap()
{
    Vector2f a = location_to_xy(44.434665,26.046532);
    Vector2f b = location_to_xy(44.4349,26.046532);

    Line line(a,b);


    //creare cerc

    Vector2f centre = location_to_xy(44.435143,26.046603);
    Circle circle(3.0f,centre);

    //creare patrat

    std::vector<Vector2f> points;
    Vector2f c =location_to_xy(44.434772,26.047177);
    Vector2f d =location_to_xy(44.435009,26.047172);
    Vector2f e =location_to_xy(44.435036,26.046962);
    Vector2f f =location_to_xy(44.046962,26.046952);

    points.push_back(c);
    points.push_back(d);
    points.push_back(e);
    points.push_back(f);
    PolygonConvex polygon(points);


//  //creare poligon convex
//    std::vector<Vector2f> pointsC;
//
//   Vector2f H(5.0f,18.0f);
//   Vector2f I(11.0f,19.0f);
//   Vector2f J(9.0f,22.0f);
//   Vector2f K(12.0f,24.0f);
//   Vector2f L(6.0f,23.0f);
//
//   pointsC.push_back(H);
//   pointsC.push_back(I);
//   pointsC.push_back(J);
//   pointsC.push_back(K);
//   pointsC.push_back(L);
//   PolygonConvex polygonC(pointsC);


//   std::vector<Vector2f> pointsOC;
//
//   Vector2f M(20.0f,23.0f);
//   Vector2f N(23.0f,20.0f);
//   Vector2f O(23.0f,17.0f);
//   Vector2f P(22.0f,17.0f);
//
//   pointsOC.push_back(M);
//   pointsOC.push_back(N);
//   pointsOC.push_back(O);
//   pointsOC.push_back(P);
//
//   OpenCurve polygonOC(pointsOC);

   addObstacle(&line);
   addObstacle(&circle);
   addObstacle(&polygon);

}

Vector2f ObjectAvoid::getStoppingPoint(float kP,float accel_cmss,Vector2f &currentP,
                                            Vector2f &desired_vel_cms)
{
    Vector2f stoppingPoint;
    for(uint16_t i=0;i<_listOfObstacle.size();i++)
      {
        stoppingPoint = _listOfObstacle[i]->getStoppingPoint(kP,accel_cmss,currentP, desired_vel_cms);
      }

   return stoppingPoint;

}



/*
 * ObjectAvoid.cpp
 *
 *  Created on: Apr 6, 2019
 *      Author: beatrice
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

void ObjectAvoid::createMap()
{
    Vector2f A(44.434665,26.046532);
    Vector2f B(44.4349,26.046532);
    Line line(A,B);


    //creare cerc
    Vector2f centre(44.435143,26.046603);
    Circle circle(3.0f,centre);

    //creare patrat
    std::vector<Vector2f> points;

    Vector2f C(44.434772,26.047177);
    Vector2f D(44.435009,26.047172);
    Vector2f E(44.435036,26.046962);
    Vector2f F(44.434898,26.046952);

    points.push_back(C);
    points.push_back(D);
    points.push_back(E);
    points.push_back(F);
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


/*
 * test_complexBC.cpp
 *
 *  Created on: Jun 11, 2019
 *      Author: beatricec
 */
#include <AP_gtest.h>
#include <vector>

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>

#include "AP_Math/ObjectAvoid.h"
#include "AP_Math/Line.h"
#include "AP_Math/Circle.h"
#include "AP_Math/PolygonConvexBC.h"
#include "AP_Math/OpenCurve.h"

// this line is necessary for linking, according to
// http://ardupilot.org/dev/docs/learning-ardupilot-the-example-sketches.html
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

//static ObjectAvoid map1;
//ObjectAvoid CreateObjects(void)
//{
//
//     //creare linie
//      Vector2f start(4.0f,5.0f);
//      Vector2f stop(8.0f, 9.0f);
//      Line line(start,stop);
//      line.setBehaviourStop();
//
//      //creare cerc
//      Vector2f centre(27.0f,5.0f);
//      Circle circle(3.0f,centre);
//      circle.setBehaviourStop();
//
//      //creare patrat
//      std::vector<Vector2f> points;
//
//      Vector2f C(14.0f,11.0f);
//      Vector2f D(18.0f,11.0f);
//      Vector2f E(18.0f,15.0f);
//      Vector2f F(14.0f,15.0f);
//
//      points.push_back(C);
//      points.push_back(D);
//      points.push_back(E);
//      points.push_back(F);
//      PolygonConvex polygon(points);
//      polygon.setBehaviourStop();
//
//      //creare poligon convex
//      std::vector<Vector2f> pointsC;
//
//      Vector2f H(5.0f,18.0f);
//      Vector2f I(11.0f,10.0f);
//      Vector2f J(9.0f,12.0f);
//      Vector2f K(12.0f,15.0f);
//      Vector2f L(6.0f,14.0f);
//
//      pointsC.push_back(H);
//      pointsC.push_back(I);
//      pointsC.push_back(J);
//      pointsC.push_back(K);
//      pointsC.push_back(L);
//      PolygonConvex polygonC(pointsC);
//      polygonC.setBehaviourStop();
//
//      ObjectAvoid map1;
//      map1.addObstacle(&line);
//      map1.addObstacle(&circle);
//      map1.addObstacle(&polygon);
//      map1.addObstacle(&polygonC);
//
//      return map1;
//}


TEST(ComplexTest, Test0)
{
   std::cout<<"=====================================================\n\n";
   std::cout<<"Tests Multiples Objects\n";

   std::cout<<"=====================================================\n\n";
   std::cout<<"Tests 0 Multiples Objects\n";

     //creare linie
     Vector2f start(4.0f,5.0f);
     Vector2f stop(8.0f, 9.0f);
     Line line(start,stop);
     line.setBehaviourStop();

     //creare cerc
     Vector2f centre(27.0f,5.0f);
     Circle circle(3.0f,centre);
     circle.setBehaviourStop();

     //creare patrat
     std::vector<Vector2f> points;

     Vector2f C(14.0f,11.0f);
     Vector2f D(18.0f,11.0f);
     Vector2f E(18.0f,15.0f);
     Vector2f F(14.0f,15.0f);

     points.push_back(C);
     points.push_back(D);
     points.push_back(E);
     points.push_back(F);
     PolygonConvex polygon(points);
     polygon.setBehaviourStop();

     //creare poligon convex
     std::vector<Vector2f> pointsC;

     Vector2f H(5.0f,18.0f);
     Vector2f I(11.0f,19.0f);
     Vector2f J(9.0f,22.0f);
     Vector2f K(12.0f,24.0f);
     Vector2f L(6.0f,23.0f);

     pointsC.push_back(H);
     pointsC.push_back(I);
     pointsC.push_back(J);
     pointsC.push_back(K);
     pointsC.push_back(L);
     PolygonConvex polygonC(pointsC);
     polygonC.setBehaviourStop();

     ObjectAvoid map1;
     map1.addObstacle(&line);
     map1.addObstacle(&circle);
     map1.addObstacle(&polygon);
     map1.addObstacle(&polygonC);

//Caz 0 se intreapta spre linie

  std::cout<<"=====================================================\n\n";
  std::cout<<"Caz 0\n";

   Vector2f currentP(12.0f,5.0f);
   Vector2f velocity(-2.0f,5.0f);
   Vector2f safevel(velocity);
  // std::cout<<"\nBehaiviourl este "<<(int)line.get_behavior()<<"\n";
   float Kp=1.0f;
   float acc=5.0f;
   float dt = 1.0f;
   map1.adjust_velocity(Kp,currentP,acc,velocity,dt);

   EXPECT_TRUE(velocity == safevel);

   //Caz in care se intreapta spre linie

   std::cout<<"=====================================================\n\n";
   std::cout<<"Caz 1\n";
     Vector2f velocity1(-7.0f,0.0f);
     Vector2f safevel1(velocity);


     map1.adjust_velocity(Kp,currentP,acc,velocity1,dt);
     std::cout<<"\n\nViteza in testul pe bune .x = "<<velocity1.x<<"\n";
     std::cout<<"Viteza in testul pe bune .y = "<<velocity1.y<<"\n";

     Vector2f newStoppingPoint = map1.getStoppingPoint(Kp,acc,currentP,velocity1);

     std::cout<<"newStoppingPoint.x = "<<newStoppingPoint.x<<"\n";
     std::cout<<"newStoppingPoint.y = "<<newStoppingPoint.y<<"\n";


     EXPECT_TRUE(newStoppingPoint.x>=6.5f && newStoppingPoint.x<=7.0f);
     EXPECT_TRUE(is_equal(newStoppingPoint.y,5.0f));

   //Caz in care se indrepata spre patrat
   std::cout<<"=====================================================\n\n";
    std::cout<<"Caz 2\n";

   Vector2f velocity2(5.0f,5.0f);
   Vector2f safevel2(velocity);

    map1.adjust_velocity(Kp,currentP,acc,velocity2,dt);
    std::cout<<"\n\nViteza in testul pe bune .x = "<<velocity2.x<<"\n";
    std::cout<<"Viteza in testul pe bune .y = "<<velocity2.y<<"\n";

    newStoppingPoint = map1.getStoppingPoint(Kp,acc,currentP,velocity2);

    std::cout<<"newStoppingPoint.x = "<<newStoppingPoint.x<<"\n";
    std::cout<<"newStoppingPoint.y = "<<newStoppingPoint.y<<"\n";


    EXPECT_TRUE(is_equal(newStoppingPoint.x,16.0f));
    EXPECT_TRUE(is_equal(newStoppingPoint.y,9.0f));


    //  ATENTIE ACEST TEST REPREZINTA UN BUG, ATUNCI CAND VITEZA ESTE PREA MARE NU MAI E CAPABIL IN CAZUL
    //  CERCULUI SA SE OPREASCA

    //   Vector2f velocity3(14.0f,0.0f);
    //   Vector2f safevel3(velocity);

    //   map1.adjust_velocity(Kp,currentP,acc,velocity3,dt);
    //   std::cout<<"\n\nViteza in testul pe bune .x = "<<velocity.x<<"\n";
    //   std::cout<<"Viteza in testul pe bune .y = "<<velocity.y<<"\n";
    //
    //    newStoppingPoint = map1.getStoppingPoint(Kp,acc,currentP,velocity3);
    //
    //   std::cout<<"newStoppingPoint.x = "<<newStoppingPoint.x<<"\n";
    //   std::cout<<"newStoppingPoint.y = "<<newStoppingPoint.y<<"\n";
    //
    //
    //   EXPECT_TRUE(is_equal(newStoppingPoint.x,22.0f));
    //   EXPECT_TRUE(is_equal(newStoppingPoint.y,5.0f));

   //EXPECT_TRUE(is_equal(1.0f,0.0f));

}

TEST(ComplexTest, TestLinie)
{    std::cout<<"=====================================================\n\n";
     std::cout<<"TEST SEPARAT";
    //creare linie
     Vector2f start(4.0f,5.0f);
     Vector2f stop(8.0f, 9.0f);
     Line line(start,stop);
     line.setBehaviourStop();
     Vector2f currentP(12.0f,5.0f);
     Vector2f velocity(-7.0f,0.0f);
     Vector2f safevel(velocity);
     float Kp=1.0f;
     float acc=5.0f;
     float dt = 1.0f;


     line.adjust_velocity(Kp,currentP,acc,velocity,dt);
     std::cout<<"Viteza IN TEST SEPARAT.x = "<<velocity.x<<"\n";
     std::cout<<"Viteza IN TEST SEPARAT.y = "<<velocity.y<<"\n";

     Vector2f newStoppingPoint =line.getStoppingPoint(Kp,acc,currentP,velocity);

     std::cout<<"newStoppingPoint.x = "<<newStoppingPoint.x<<"\n";
     std::cout<<"newStoppingPoint.y = "<<newStoppingPoint.y<<"\n";
     //EXPECT_TRUE(is_equal(1.0f,0.0f));

}
TEST(ComplexTest, Test1)
{

   std::cout<<"=====================================================\n\n";
   std::cout<<"Tests 1 Multiples Objects\n";

      //creare linie
      Vector2f start(4.0f,5.0f);
      Vector2f stop(8.0f, 9.0f);
      Line line(start,stop);
      line.setBehaviourStop();

      //creare cerc
      Vector2f centre(27.0f,5.0f);
      Circle circle(3.0f,centre);
      circle.setBehaviourStop();

      //creare patrat
      std::vector<Vector2f> points;

      Vector2f C(14.0f,11.0f);
      Vector2f D(18.0f,11.0f);
      Vector2f E(18.0f,15.0f);
      Vector2f F(14.0f,15.0f);

      points.push_back(C);
      points.push_back(D);
      points.push_back(E);
      points.push_back(F);
      PolygonConvex polygon(points);
      polygon.setBehaviourStop();

      //creare poligon convex
      std::vector<Vector2f> pointsC;

      Vector2f H(5.0f,18.0f);
      Vector2f I(11.0f,19.0f);
      Vector2f J(9.0f,22.0f);
      Vector2f K(12.0f,24.0f);
      Vector2f L(6.0f,23.0f);

      pointsC.push_back(H);
      pointsC.push_back(I);
      pointsC.push_back(J);
      pointsC.push_back(K);
      pointsC.push_back(L);
      PolygonConvex polygonC(pointsC);
      polygonC.setBehaviourStop();

      ObjectAvoid map1;
      map1.addObstacle(&line);
      map1.addObstacle(&circle);
      map1.addObstacle(&polygon);
      map1.addObstacle(&polygonC);

     float Kp=1.0f;
     float acc=5.0f;
     float dt = 1.0f;

   Vector2f currentP(6.0f,14.0f);

   //Caz in care se indreapta spre poligonul convex
   std::cout<<"=====================================================\n\n";
   std::cout<<"Caz 4\n";
   Vector2f velocity(0.0f,5.0f);
   Vector2f safevel(velocity);

   map1.adjust_velocity(Kp,currentP,acc,velocity,dt);
   Vector2f newStoppingPoint = map1.getStoppingPoint(Kp,acc,currentP,velocity);

   std::cout<<"newStoppingPoint.x = "<<newStoppingPoint.x<<"\n";
   std::cout<<"newStoppingPoint.y = "<<newStoppingPoint.y<<"\n";


   EXPECT_TRUE(newStoppingPoint.y>=16.0f && newStoppingPoint.y<=16.5f);
   EXPECT_TRUE(is_equal(newStoppingPoint.x,6.0f));

 //Caz in care se intreapta spre patrat
   std::cout<<"=====================================================\n\n";
   std::cout<<"Caz 5\n";
   Vector2f velocity1(10.0f,0.0f);
   Vector2f safevel1(velocity);

     map1.adjust_velocity(Kp,currentP,acc,velocity1,dt);
     newStoppingPoint = map1.getStoppingPoint(Kp,acc,currentP,velocity1);

     std::cout<<"newStoppingPoint.x = "<<newStoppingPoint.x<<"\n";
     std::cout<<"newStoppingPoint.y = "<<newStoppingPoint.y<<"\n";
     EXPECT_TRUE(newStoppingPoint.x>=12.0f && newStoppingPoint.x<=12.001f);
     EXPECT_TRUE(is_equal(newStoppingPoint.y,14.0f));

     std::cout<<"=====================================================\n\n";
     std::cout<<"Caz 6\n";
     Vector2f velocity2(-2.0f,-2.0f);
     Vector2f safevel2(velocity2);

     map1.adjust_velocity(Kp,currentP,acc,velocity2,dt);
     newStoppingPoint = map1.getStoppingPoint(Kp,acc,currentP,velocity2);

     std::cout<<"newStoppingPoint.x = "<<newStoppingPoint.x<<"\n";
     std::cout<<"newStoppingPoint.y = "<<newStoppingPoint.y<<"\n";
     EXPECT_TRUE(velocity2 == safevel2);


  // EXPECT_TRUE(is_equal(2.0f,1.0f));
}

TEST(ComplexTest, Test2)
{

   std::cout<<"=====================================================\n\n";
   std::cout<<"Tests 2 Multiples Objects\n";

      //creare linie
      Vector2f start(4.0f,5.0f);
      Vector2f stop(8.0f, 9.0f);
      Line line(start,stop);
      line.setBehaviourStop();

      //creare cerc
      Vector2f centre(27.0f,5.0f);
      Circle circle(3.0f,centre);
      circle.setBehaviourStop();

      //creare patrat
      std::vector<Vector2f> points;

      Vector2f C(14.0f,11.0f);
      Vector2f D(18.0f,11.0f);
      Vector2f E(18.0f,15.0f);
      Vector2f F(14.0f,15.0f);

      points.push_back(C);
      points.push_back(D);
      points.push_back(E);
      points.push_back(F);
      PolygonConvex polygon(points);
      polygon.setBehaviourStop();

      //creare poligon convex
      std::vector<Vector2f> pointsC;

      Vector2f H(5.0f,18.0f);
      Vector2f I(11.0f,19.0f);
      Vector2f J(9.0f,22.0f);
      Vector2f K(12.0f,24.0f);
      Vector2f L(6.0f,23.0f);

      pointsC.push_back(H);
      pointsC.push_back(I);
      pointsC.push_back(J);
      pointsC.push_back(K);
      pointsC.push_back(L);
      PolygonConvex polygonC(pointsC);
      polygonC.setBehaviourStop();

      ObjectAvoid map1;
      map1.addObstacle(&line);
      map1.addObstacle(&circle);
      map1.addObstacle(&polygon);
      map1.addObstacle(&polygonC);
      float Kp=1.0f;
      float acc=5.0f;
      float dt = 1.0f;

      Vector2f currentP(21.0f,8.0f);

      //Caz in care se intreapta spre patrat
        std::cout<<"=====================================================\n\n";
        std::cout<<"Caz 7\n";
        Vector2f velocity(-4.0f,4.0f);
        Vector2f safevel(velocity);

        map1.adjust_velocity(Kp,currentP,acc,velocity,dt);
        Vector2f newStoppingPoint = map1.getStoppingPoint(Kp,acc,currentP,velocity);

        std::cout<<"newStoppingPoint.x = "<<newStoppingPoint.x<<"\n";
        std::cout<<"newStoppingPoint.y = "<<newStoppingPoint.y<<"\n";

        EXPECT_TRUE(is_equal(newStoppingPoint.x,20.0f));
        EXPECT_TRUE(is_equal(newStoppingPoint.y,9.0f));


        std::cout<<"=====================================================\n\n";
        std::cout<<"Caz 8\n";
        Vector2f velocity1(6.0f,0.0f);
        Vector2f safevel1(velocity);

          map1.adjust_velocity(Kp,currentP,acc,velocity1,dt);
          newStoppingPoint = map1.getStoppingPoint(Kp,acc,currentP,velocity1);

          std::cout<<"newStoppingPoint.x = "<<newStoppingPoint.x<<"\n";
          std::cout<<"newStoppingPoint.y = "<<newStoppingPoint.y<<"\n";
          EXPECT_TRUE(is_equal(newStoppingPoint.x,23.0f));
          EXPECT_TRUE(is_equal(newStoppingPoint.y,8.0f));

          std::cout<<"=====================================================\n\n";
          std::cout<<"Caz 9\n";
          Vector2f velocity2(3.0f,3.0f);
          Vector2f safevel2(velocity2);

          map1.adjust_velocity(Kp,currentP,acc,velocity2,dt);
          newStoppingPoint = map1.getStoppingPoint(Kp,acc,currentP,velocity2);

          std::cout<<"newStoppingPoint.x = "<<newStoppingPoint.x<<"\n";
          std::cout<<"newStoppingPoint.y = "<<newStoppingPoint.y<<"\n";
          EXPECT_TRUE(velocity2 == safevel2);

}

TEST(ComplexTest, Test3)
{

   std::cout<<"=====================================================\n\n";
   std::cout<<"Tests 3 Multiples Objects\n";

      //creare linie
      Vector2f start(4.0f,5.0f);
      Vector2f stop(8.0f, 9.0f);
      Line line(start,stop);
      line.setBehaviourStop();

      //creare cerc
      Vector2f centre(27.0f,5.0f);
      Circle circle(3.0f,centre);
      circle.setBehaviourStop();

      //creare patrat
      std::vector<Vector2f> points;

      Vector2f C(14.0f,11.0f);
      Vector2f D(18.0f,11.0f);
      Vector2f E(18.0f,15.0f);
      Vector2f F(14.0f,15.0f);

      points.push_back(C);
      points.push_back(D);
      points.push_back(E);
      points.push_back(F);
      PolygonConvex polygon(points);
      polygon.setBehaviourStop();

      //creare poligon convex
      std::vector<Vector2f> pointsC;

      Vector2f H(5.0f,18.0f);
      Vector2f I(11.0f,19.0f);
      Vector2f J(9.0f,22.0f);
      Vector2f K(12.0f,24.0f);
      Vector2f L(6.0f,23.0f);

      pointsC.push_back(H);
      pointsC.push_back(I);
      pointsC.push_back(J);
      pointsC.push_back(K);
      pointsC.push_back(L);
      PolygonConvex polygonC(pointsC);
      polygonC.setBehaviourStop();

      std::vector<Vector2f> pointsOC;

       Vector2f M(20.0f,23.0f);
       Vector2f N(23.0f,20.0f);
       Vector2f O(23.0f,17.0f);
       Vector2f P(22.0f,17.0f);

       pointsOC.push_back(M);
       pointsOC.push_back(N);
       pointsOC.push_back(O);
       pointsOC.push_back(P);

       OpenCurve polygonOC(pointsOC);
       polygonOC.setBehaviourStop();

      ObjectAvoid map1;
      map1.addObstacle(&line);
      map1.addObstacle(&circle);
      map1.addObstacle(&polygon);
      map1.addObstacle(&polygonC);
      map1.addObstacle(&polygonOC);

      float Kp=1.0f;
      float acc=5.0f;
      float dt = 1.0f;

    Vector2f currentP(19.0f,19.0f);

    //Caz in care se intreapta spre patrat
      std::cout<<"=====================================================\n\n";
      std::cout<<"Caz 10\n";
      Vector2f velocity(2.0f,2.0f);
      Vector2f safevel(velocity);

      map1.adjust_velocity(Kp,currentP,acc,velocity,dt);
      Vector2f newStoppingPoint = map1.getStoppingPoint(Kp,acc,currentP,velocity);

      std::cout<<"newStoppingPoint.x = "<<newStoppingPoint.x<<"\n";
      std::cout<<"newStoppingPoint.y = "<<newStoppingPoint.y<<"\n";

      EXPECT_TRUE(newStoppingPoint.x>20.0f && newStoppingPoint.x<21.0f);
      EXPECT_TRUE(newStoppingPoint.y>20.0f && newStoppingPoint.y<21.0f);

      std::cout<<"=====================================================\n\n";
      std::cout<<"Caz 11\n";
      Vector2f velocity1(5.0f,0.0f);
      Vector2f safevel1(velocity);

        map1.adjust_velocity(Kp,currentP,acc,velocity1,dt);
        newStoppingPoint = map1.getStoppingPoint(Kp,acc,currentP,velocity1);

        std::cout<<"newStoppingPoint.x = "<<newStoppingPoint.x<<"\n";
        std::cout<<"newStoppingPoint.y = "<<newStoppingPoint.y<<"\n";
        EXPECT_TRUE(is_equal(newStoppingPoint.x,21.0f));
        EXPECT_TRUE(is_equal(newStoppingPoint.y,19.0f));

        std::cout<<"=====================================================\n\n";
        std::cout<<"Caz 12\n";
        Vector2f velocity2(0.0f,-3.0f);
        Vector2f safevel2(velocity2);

        map1.adjust_velocity(Kp,currentP,acc,velocity2,dt);
        newStoppingPoint = map1.getStoppingPoint(Kp,acc,currentP,velocity2);

        std::cout<<"newStoppingPoint.x = "<<newStoppingPoint.x<<"\n";
        std::cout<<"newStoppingPoint.y = "<<newStoppingPoint.y<<"\n";
        EXPECT_TRUE(is_equal(newStoppingPoint.x,19.0f));
        EXPECT_TRUE(is_equal(newStoppingPoint.y,17.0f));

        std::cout<<"=====================================================\n\n";
        std::cout<<"Caz 13\n";
        Vector2f velocity3(-10.0f,0.0f);
        Vector2f safevel3(velocity3);

        map1.adjust_velocity(Kp,currentP,acc,velocity3,dt);
        newStoppingPoint = map1.getStoppingPoint(Kp,acc,currentP,velocity3);

       std::cout<<"newStoppingPoint.x = "<<newStoppingPoint.x<<"\n";
       std::cout<<"newStoppingPoint.y = "<<newStoppingPoint.y<<"\n";
       EXPECT_TRUE(newStoppingPoint.x>=13.0f && newStoppingPoint.x<=14.0f);
       EXPECT_TRUE(is_equal(newStoppingPoint.y,19.0f));

}
TEST(ComplexTest, Test4)
{

   std::cout<<"=====================================================\n\n";
   std::cout<<"Tests 4 Multiples Objects\n";

      //creare linie
      Vector2f start(4.0f,5.0f);
      Vector2f stop(8.0f, 9.0f);
      Line line(start,stop);
      line.setBehaviourStop();

      //creare cerc
      Vector2f centre(27.0f,5.0f);
      Circle circle(3.0f,centre);
      circle.setBehaviourStop();

      //creare patrat
      std::vector<Vector2f> points;

      Vector2f C(14.0f,11.0f);
      Vector2f D(18.0f,11.0f);
      Vector2f E(18.0f,15.0f);
      Vector2f F(14.0f,15.0f);

      points.push_back(C);
      points.push_back(D);
      points.push_back(E);
      points.push_back(F);
      PolygonConvex polygon(points);
      polygon.setBehaviourStop();

      //creare poligon convex
      std::vector<Vector2f> pointsC;

      Vector2f H(5.0f,18.0f);
      Vector2f I(11.0f,19.0f);
      Vector2f J(9.0f,22.0f);
      Vector2f K(12.0f,24.0f);
      Vector2f L(6.0f,23.0f);

      pointsC.push_back(H);
      pointsC.push_back(I);
      pointsC.push_back(J);
      pointsC.push_back(K);
      pointsC.push_back(L);
      PolygonConvex polygonC(pointsC);
      polygonC.setBehaviourStop();

      std::vector<Vector2f> pointsOC;

       Vector2f M(20.0f,23.0f);
       Vector2f N(23.0f,20.0f);
       Vector2f O(23.0f,17.0f);
       Vector2f P(22.0f,17.0f);

       pointsOC.push_back(M);
       pointsOC.push_back(N);
       pointsOC.push_back(O);
       pointsOC.push_back(P);

       OpenCurve polygonOC(pointsOC);
       polygonOC.setBehaviourStop();

      ObjectAvoid map1;
      map1.addObstacle(&line);
      map1.addObstacle(&circle);
      map1.addObstacle(&polygon);
      map1.addObstacle(&polygonC);
      map1.addObstacle(&polygonOC);

      float Kp=1.0f;
      float acc=5.0f;
      float dt = 1.0f;

      //Caz in care se intreapta spre curba deschisa
       std::cout<<"=====================================================\n\n";
       std::cout<<"Caz 14\n";
      Vector2f currentP(28.0f,21.0f);
      Vector2f velocity(-6.0f,-5.0f);
      Vector2f safevel(velocity);

      map1.adjust_velocity(Kp,currentP,acc,velocity,dt);
      Vector2f newStoppingPoint = map1.getStoppingPoint(Kp,acc,currentP,velocity);

      std::cout<<"newStoppingPoint.x = "<<newStoppingPoint.x<<"\n";
      std::cout<<"newStoppingPoint.y = "<<newStoppingPoint.y<<"\n";
      EXPECT_TRUE(newStoppingPoint.x>=24.5f && newStoppingPoint.x<=25.0f);
      EXPECT_TRUE(newStoppingPoint.y>=18.0f && newStoppingPoint.y<=18.5f);

      std::cout<<"=====================================================\n\n";
      std::cout<<"Caz 15\n";
      Vector2f currentP1(32.0f,12.0f);
      Vector2f velocity1(-3.0f,-4.0f);
      Vector2f safevel1(velocity1);

    map1.adjust_velocity(Kp,currentP1,acc,velocity1,dt);
    newStoppingPoint = map1.getStoppingPoint(Kp,acc,currentP1,velocity1);

    std::cout<<"newStoppingPoint.x = "<<newStoppingPoint.x<<"\n";
    std::cout<<"newStoppingPoint.y = "<<newStoppingPoint.y<<"\n";
    EXPECT_TRUE(newStoppingPoint.x>=29.0f && newStoppingPoint.x<=30.0f);
    EXPECT_TRUE(newStoppingPoint.y>=9.0f && newStoppingPoint.y<=10.0f);

}



AP_GTEST_MAIN()





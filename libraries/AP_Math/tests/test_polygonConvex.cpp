/*
 * test_polygonConvex.cpp
 *
 *  Created on: May 25, 2019
 *      Author: beatrice
 */


#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_Math/PolygonConvexBC.h"


// this line is necessary for linking, according to
// http://ardupilot.org/dev/docs/learning-ardupilot-the-example-sketches.html
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

TEST(PolygonConvex, Test0)
{
    std::cout<<"=====================================================\n\n";
    std::cout<<"Test PolygonConvexBC\n\n";
    std::cout<<"TestStop0\n";
//    Vector2f A(5.0f,5.0f);
//    Vector2f B(5.0f,9.0f);
//    Vector2f C(12.0f,9.0f);
//    Vector2f D(12.0f,5.0f);
    Vector2f A(2.0f,2.0f);
    Vector2f B(4.0f,6.0f);
    Vector2f C(8.0f,5.0f);
    Vector2f D(11.0f,8.0f);
    Vector2f E(13.0f,3.0f);
    Vector2f F(8.0f,3.0f);

    Vector2f currentP(8.0f,9.0f);
    Vector2f velocity(-1.0f,-4.0f);

    Vector2f safeVelocity(velocity);
    std::vector<Vector2f> points;
    points.push_back(A);
    points.push_back(B);
    points.push_back(C);
    points.push_back(D);
    points.push_back(E);
    points.push_back(F);

    PolygonConvex polygon(points);

    polygon.setBehaviourStop();
    std::cout<<"\nBehaiviourl este "<<(int)polygon.get_behavior()<<"\n";
    float Kp=1.0f;
    float acc=5.0f;
    float dt = 1.0f;


    polygon.adjust_velocity(Kp,currentP,acc,velocity,dt);

//    Vector2f stopping_point_new = polygon.getStoppingPoint(Kp,acc,currentP,velocity);
//    std::cout<<"\n\nValoarea StoppingPointNew X "<<stopping_point_new.x<<"\n";
//    std::cout<<"Valoarea StoppingPointNew Y "<<stopping_point_new.y<<"\n";



//
//    EXPECT_TRUE(is_equal(stopping_point_new.x,7.0f));
//    EXPECT_TRUE(stopping_point_new.y <= 8.0f);

    EXPECT_TRUE(is_equal(1.0f,0.0f));

}


//Se testeaza cazuri simple in care pozitia este in afara poligonului si viteza vine perpendicular indreptata spre laturile lui.
//Tinand cont ca este un drepatunghi s-a testat pentru toate cele 4laturi
TEST(PolygonConvex, Test1)
{
    std::cout<<"=====================================================\n\n";
    std::cout<<"TestStop1\n";

    std::vector<Vector2f> points;
    Vector2f A(5.0f,5.0f);
    Vector2f B(5.0f,9.0f);
    Vector2f C(12.0f,9.0f);
    Vector2f D(12.0f,5.0f);
    points.push_back(A);
    points.push_back(B);
    points.push_back(C);
    points.push_back(D);
    PolygonConvex polygon(points);

    Vector2f currentP(8.0f,14.0f);
    Vector2f velocity(0.0f,-4.0f);

    Vector2f safeVelocity(velocity);
    polygon.setBehaviourStop();
    std::cout<<"\nBehaiviourl este "<<(int)polygon.get_behavior()<<"\n";
    float Kp=1.0f;
    float acc=5.0f;
    float dt = 1.0f;

// Caz 1
    std::cout<<"=====================================================\n\n";
    std::cout<<"Caz 1.1\n";
    polygon.adjust_velocity(Kp,currentP,acc,velocity,dt);

    Vector2f stopping_point_new = polygon.getStoppingPoint(Kp,acc,currentP,velocity);
    std::cout<<"\n\nValoarea StoppingPointNew X "<<stopping_point_new.x<<"\n";
    std::cout<<"Valoarea StoppingPointNew Y "<<stopping_point_new.y<<"\n";


    EXPECT_TRUE(is_equal(stopping_point_new.x,8.0f));
    EXPECT_TRUE(stopping_point_new.y >= 11.0f);

 //Caz 2
    std::cout<<"=====================================================\n\n";
    std::cout<<"Caz 1.2\n";
    Vector2f currentP1(17.0f,7.0f);
    Vector2f velocity1(-4.0f,0.0f);
    Vector2f safeVelocity1(velocity1);
    polygon.adjust_velocity(Kp,currentP1,acc,velocity1,dt);

    stopping_point_new = polygon.getStoppingPoint(Kp,acc,currentP1,velocity1);
    std::cout<<"\n\nValoarea StoppingPointNew X "<<stopping_point_new.x<<"\n";
    std::cout<<"Valoarea StoppingPointNew Y "<<stopping_point_new.y<<"\n";

    EXPECT_TRUE(is_equal(stopping_point_new.x,14.0f));
    EXPECT_TRUE(is_equal(stopping_point_new.y,7.0f));

//Caz 3
   std::cout<<"=====================================================\n\n";
   std::cout<<"Caz 1.3\n";
   Vector2f currentP2(5.0f,2.0f);
   Vector2f velocity2(0.0f,2.0f);
   Vector2f safeVelocity2(velocity2);
   polygon.adjust_velocity(Kp,currentP2,acc,velocity2,dt);

   stopping_point_new = polygon.getStoppingPoint(Kp,acc,currentP2,velocity2);
   std::cout<<"\n\nValoarea StoppingPointNew X "<<stopping_point_new.x<<"\n";
   std::cout<<"Valoarea StoppingPointNew Y "<<stopping_point_new.y<<"\n";

   EXPECT_TRUE(is_equal(stopping_point_new.x,5.0f));
   EXPECT_TRUE(is_equal(stopping_point_new.y,3.0f));

//Caz 4
  std::cout<<"=====================================================\n\n";
  std::cout<<"Caz 1.4\n";
  Vector2f currentP3(2.0f,8.0f);
  Vector2f velocity3(4.0f,0.0f);
  Vector2f safeVelocity3(velocity3);
  polygon.adjust_velocity(Kp,currentP3,acc,velocity3,dt);

  stopping_point_new = polygon.getStoppingPoint(Kp,acc,currentP3,velocity3);
  std::cout<<"\n\nValoarea StoppingPointNew X "<<stopping_point_new.x<<"\n";
  std::cout<<"Valoarea StoppingPointNew Y "<<stopping_point_new.y<<"\n";

  EXPECT_TRUE(is_equal(stopping_point_new.x,3.0f));
  EXPECT_TRUE(is_equal(stopping_point_new.y,8.0f));

}


//Cazuri in care avem acelasi dreptunghi simplu da in acest caz viteza are o directie oblica
TEST(PolygonConvex, Test2)
{
    std::cout<<"=====================================================\n\n";
    std::cout<<"TestStop2\n";

    std::vector<Vector2f> points;
    Vector2f A(5.0f,5.0f);
    Vector2f B(5.0f,9.0f);
    Vector2f C(12.0f,9.0f);
    Vector2f D(12.0f,5.0f);
    points.push_back(A);
    points.push_back(B);
    points.push_back(C);
    points.push_back(D);
    PolygonConvex polygon(points);
    polygon.setBehaviourStop();

    std::cout<<"\nBehaiviourl este "<<(int)polygon.get_behavior()<<"\n";
    float Kp=1.0f;
    float acc=5.0f;
    float dt = 1.0f;

 // Caz 1
    std::cout<<"=====================================================\n\n";
    std::cout<<"Caz 2.1\n";

    Vector2f currentP(2.0f,2.0f);
    Vector2f velocity(5.0f,5.0f);
    Vector2f safeVelocity(velocity);
    polygon.adjust_velocity(Kp,currentP,acc,velocity,dt);

    Vector2f stopping_point_new = polygon.getStoppingPoint(Kp,acc,currentP,velocity);
    std::cout<<"\n\nValoarea StoppingPointNew X "<<stopping_point_new.x<<"\n";
    std::cout<<"Valoarea StoppingPointNew Y "<<stopping_point_new.y<<"\n";


    EXPECT_TRUE(is_equal(stopping_point_new.x,3.0f));
    EXPECT_TRUE(is_equal(stopping_point_new.y,3.0f));

 //Caz 2
    std::cout<<"=====================================================\n\n";
    std::cout<<"Caz 2.2\n";
    Vector2f currentP1(8.0f,14.0f);
    Vector2f velocity1(2.0f,-6.0f);
    Vector2f safeVelocity1(velocity1);
    polygon.adjust_velocity(Kp,currentP1,acc,velocity1,dt);

    stopping_point_new = polygon.getStoppingPoint(Kp,acc,currentP1,velocity1);
    std::cout<<"\n\nValoarea StoppingPointNew X "<<stopping_point_new.x<<"\n";
    std::cout<<"Valoarea StoppingPointNew Y "<<stopping_point_new.y<<"\n";

    EXPECT_TRUE(is_equal(stopping_point_new.x,9.0f));
    EXPECT_TRUE(is_equal(stopping_point_new.y,11.0f));

//Caz 3
   std::cout<<"=====================================================\n\n";
   std::cout<<"Caz 2.3\n";
   Vector2f currentP2(8.0f,14.0f);
   Vector2f velocity2(5.0f,-4.0f);
   Vector2f safeVelocity2(velocity2);
   polygon.adjust_velocity(Kp,currentP2,acc,velocity2,dt);

   stopping_point_new = polygon.getStoppingPoint(Kp,acc,currentP2,velocity2);
   std::cout<<"\n\nValoarea StoppingPointNew X "<<stopping_point_new.x<<"\n";
   std::cout<<"Valoarea StoppingPointNew Y "<<stopping_point_new.y<<"\n";

   EXPECT_TRUE(stopping_point_new.x > 11.5f);
   EXPECT_TRUE(stopping_point_new.x < 12.0f);
   EXPECT_TRUE(is_equal(stopping_point_new.y,11.0f));

//Caz 4
  std::cout<<"=====================================================\n\n";
  std::cout<<"Caz 2.4\n";
  Vector2f currentP3(8.0f,14.0f);
  Vector2f velocity3(-2.0f,-6.0f);
  Vector2f safeVelocity3(velocity3);
  polygon.adjust_velocity(Kp,currentP3,acc,velocity3,dt);

  stopping_point_new = polygon.getStoppingPoint(Kp,acc,currentP3,velocity3);
  std::cout<<"\n\nValoarea StoppingPointNew X "<<stopping_point_new.x<<"\n";
  std::cout<<"Valoarea StoppingPointNew Y "<<stopping_point_new.y<<"\n";

  EXPECT_TRUE(is_equal(stopping_point_new.x,7.0f));
  EXPECT_TRUE(is_equal(stopping_point_new.y,11.0f));

}

//Cazuri in care avem acelasi dreptunghi simplu dar in care vehicolul se deplaseaza in asa fel incat nu este necesara ajustarea vitezei
//Deci se va testa daca ramane atat viteza cat si punctul de stop la fel
TEST(PolygonConvex, Test3)
{
    std::cout<<"=====================================================\n\n";
    std::cout<<"TestStop3\n";

    std::vector<Vector2f> points;
    Vector2f A(5.0f,5.0f);
    Vector2f B(5.0f,9.0f);
    Vector2f C(12.0f,9.0f);
    Vector2f D(12.0f,5.0f);
    points.push_back(A);
    points.push_back(B);
    points.push_back(C);
    points.push_back(D);
    PolygonConvex polygon(points);
    polygon.setBehaviourStop();

    std::cout<<"\nBehaiviourl este "<<(int)polygon.get_behavior()<<"\n";
    float Kp=1.0f;
    float acc=5.0f;
    float dt = 1.0f;

 // Caz 1
    std::cout<<"=====================================================\n\n";
    std::cout<<"Caz 3.1\n";

    Vector2f currentP(17.0f,7.0f);
    Vector2f velocity(-3.0f,6.0f);
    Vector2f safeVelocity(velocity);

    Vector2f stopping_point = polygon.getStoppingPoint(Kp,acc,currentP,velocity);

    polygon.adjust_velocity(Kp,currentP,acc,velocity,dt);


    Vector2f stopping_point_new = polygon.getStoppingPoint(Kp,acc,currentP,velocity);
    std::cout<<"\n\nValoarea StoppingPointNew X "<<stopping_point_new.x<<"\n";
    std::cout<<"Valoarea StoppingPointNew Y "<<stopping_point_new.y<<"\n";

    EXPECT_TRUE(stopping_point_new ==stopping_point);
    EXPECT_TRUE(velocity == safeVelocity);


 // Caz 2
   std::cout<<"=====================================================\n\n";
   std::cout<<"Caz 3.2\n";

   Vector2f currentP1(17.0f,7.0f);
   Vector2f velocity1(2.0f,0.0f);
   Vector2f safeVelocity1(velocity1);

   stopping_point = polygon.getStoppingPoint(Kp,acc,currentP1,velocity1);

   polygon.adjust_velocity(Kp,currentP1,acc,velocity1,dt);


   stopping_point_new = polygon.getStoppingPoint(Kp,acc,currentP1,velocity1);
   std::cout<<"\n\nValoarea StoppingPointNew X "<<stopping_point_new.x<<"\n";
   std::cout<<"Valoarea StoppingPointNew Y "<<stopping_point_new.y<<"\n";

   EXPECT_TRUE(stopping_point_new ==stopping_point);
   EXPECT_TRUE(velocity1 == safeVelocity1);


// Caz 3
  std::cout<<"=====================================================\n\n";
  std::cout<<"Caz 3.3\n";

  Vector2f currentP2(17.0f,7.0f);
  Vector2f velocity2(0.0f,-3.0f);
  Vector2f safeVelocity2(velocity2);

  stopping_point = polygon.getStoppingPoint(Kp,acc,currentP2,velocity2);

  polygon.adjust_velocity(Kp,currentP2,acc,velocity2,dt);


  stopping_point_new = polygon.getStoppingPoint(Kp,acc,currentP2,velocity2);
  std::cout<<"\n\nValoarea StoppingPointNew X "<<stopping_point_new.x<<"\n";
  std::cout<<"Valoarea StoppingPointNew Y "<<stopping_point_new.y<<"\n";

  EXPECT_TRUE(stopping_point_new ==stopping_point);
  EXPECT_TRUE(velocity2 == safeVelocity2);


// Caz 4
 std::cout<<"=====================================================\n\n";
 std::cout<<"Caz 3.4\n";

 Vector2f currentP3(17.0f,7.0f);
 Vector2f velocity3(-1.0f,-1.0f);
 Vector2f safeVelocity3(velocity3);

 stopping_point = polygon.getStoppingPoint(Kp,acc,currentP3,velocity3);

 polygon.adjust_velocity(Kp,currentP3,acc,velocity3,dt);


 stopping_point_new = polygon.getStoppingPoint(Kp,acc,currentP3,velocity3);
 std::cout<<"\n\nValoarea StoppingPointNew X "<<stopping_point_new.x<<"\n";
 std::cout<<"Valoarea StoppingPointNew Y "<<stopping_point_new.y<<"\n";

 EXPECT_TRUE(stopping_point_new ==stopping_point);
 EXPECT_TRUE(velocity3 == safeVelocity3);

}


//In acest test se testeaza diferite cazuri pentru un polygon convev neregulat
TEST(PolygonConvex, Test4)
{
    std::cout<<"=====================================================\n\n";
    std::cout<<"TestStop4\n";

    std::vector<Vector2f> points;
    Vector2f A(5.0f,4.0f);
    Vector2f B(5.0f,8.0f);
    Vector2f C(9.0f,12.0f);
    Vector2f D(12.0f,9.0f);
    Vector2f E(15.0f,9.0f);
    Vector2f F(12.0f,6.0f);
    Vector2f G(9.0f,6.0f);
    Vector2f H(10.0f,4.0f);

    points.push_back(A);
    points.push_back(B);
    points.push_back(C);
    points.push_back(D);
    points.push_back(E);
    points.push_back(F);
    points.push_back(G);
    points.push_back(H);

    PolygonConvex polygon(points);
    polygon.setBehaviourStop();

    std::cout<<"\nBehaiviourl este "<<(int)polygon.get_behavior()<<"\n";
    float Kp=1.0f;
    float acc=5.0f;
    float dt = 1.0f;

 // Caz 1
    std::cout<<"=====================================================\n\n";
    std::cout<<"Caz 4.1\n";

    Vector2f currentP(20.0f,5.0f);
    Vector2f velocity(-6.5f,3.5f);
    Vector2f safeVelocity(velocity);
    polygon.adjust_velocity(Kp,currentP,acc,velocity,dt);

    Vector2f stopping_point_new = polygon.getStoppingPoint(Kp,acc,currentP,velocity);
    std::cout<<"\n\nValoarea StoppingPointNew X "<<stopping_point_new.x<<"\n";
    std::cout<<"Valoarea StoppingPointNew Y "<<stopping_point_new.y<<"\n";


    EXPECT_TRUE((stopping_point_new.x >= 15.5f) && (stopping_point_new.x <= 16.0f));
    EXPECT_TRUE((stopping_point_new.y >= 6.5f) && (stopping_point_new.y <= 7.5f));


 //Caz 2
   std::cout<<"=====================================================\n\n";
   std::cout<<"Caz 4.2\n";

   Vector2f currentP1(2.0f,6.0f);
   Vector2f velocity1(5.0f,0.0f);
   Vector2f safeVelocity1(velocity1);
   polygon.adjust_velocity(Kp,currentP1,acc,velocity1,dt);

   stopping_point_new = polygon.getStoppingPoint(Kp,acc,currentP1,velocity1);
   std::cout<<"\n\nValoarea StoppingPointNew X "<<stopping_point_new.x<<"\n";
   std::cout<<"Valoarea StoppingPointNew Y "<<stopping_point_new.y<<"\n";

   EXPECT_TRUE(is_equal(stopping_point_new.x,3.0f));
   EXPECT_TRUE(is_equal(stopping_point_new.y,6.0f));


   //Caz 3
    std::cout<<"=====================================================\n\n";
    std::cout<<"Caz 4.3\n";

    Vector2f currentP2(2.0f,1.0f);
    Vector2f velocity2(4.0f,4.0f);
    Vector2f safeVelocity2(velocity2);
    polygon.adjust_velocity(Kp,currentP2,acc,velocity2,dt);

    stopping_point_new = polygon.getStoppingPoint(Kp,acc,currentP2,velocity2);
    std::cout<<"\n\nValoarea StoppingPointNew X "<<stopping_point_new.x<<"\n";
    std::cout<<"Valoarea StoppingPointNew Y "<<stopping_point_new.y<<"\n";

    EXPECT_TRUE(is_equal(stopping_point_new.x,3.0f));
    EXPECT_TRUE(is_equal(stopping_point_new.y,2.0f));


    //Caz 4
     std::cout<<"=====================================================\n\n";
     std::cout<<"Caz 4.4\n";

     Vector2f currentP3(14.0f,14.0f);
     Vector2f velocity3(0.0f,-11.0f);
     Vector2f safeVelocity3(velocity3);
     polygon.adjust_velocity(Kp,currentP3,acc,velocity3,dt);

     stopping_point_new = polygon.getStoppingPoint(Kp,acc,currentP3,velocity3);
     std::cout<<"\n\nValoarea StoppingPointNew X "<<stopping_point_new.x<<"\n";
     std::cout<<"Valoarea StoppingPointNew Y "<<stopping_point_new.y<<"\n";

     EXPECT_TRUE(is_equal(stopping_point_new.x,14.0f));
     EXPECT_TRUE(is_equal(stopping_point_new.y,11.0f));


}



//In acest test se testeaza cazuri in care viteza nu trebuie schimbata pentru un polygon convev neregulat
TEST(PolygonConvex, Test5)
{
    std::cout<<"=====================================================\n\n";
    std::cout<<"TestStop4\n";

    std::vector<Vector2f> points;
    Vector2f A(5.0f,4.0f);
    Vector2f B(5.0f,8.0f);
    Vector2f C(9.0f,12.0f);
    Vector2f D(12.0f,9.0f);
    Vector2f E(15.0f,9.0f);
    Vector2f F(12.0f,6.0f);
    Vector2f G(9.0f,6.0f);
    Vector2f H(10.0f,4.0f);

    points.push_back(A);
    points.push_back(B);
    points.push_back(C);
    points.push_back(D);
    points.push_back(E);
    points.push_back(F);
    points.push_back(G);
    points.push_back(H);

    PolygonConvex polygon(points);
    polygon.setBehaviourStop();

    std::cout<<"\nBehaiviourl este "<<(int)polygon.get_behavior()<<"\n";
    float Kp=1.0f;
    float acc=5.0f;
    float dt = 1.0f;

 /// Caz 1
    std::cout<<"=====================================================\n\n";
    std::cout<<"Caz 5.1\n";

    Vector2f currentP(20.0f,5.0f);
    Vector2f velocity(-4.0f,-2.0f);
    Vector2f safeVelocity(velocity);

    Vector2f stopping_point = polygon.getStoppingPoint(Kp,acc,currentP,velocity);

    polygon.adjust_velocity(Kp,currentP,acc,velocity,dt);


    Vector2f stopping_point_new = polygon.getStoppingPoint(Kp,acc,currentP,velocity);
    std::cout<<"\n\nValoarea StoppingPointNew X "<<stopping_point_new.x<<"\n";
    std::cout<<"Valoarea StoppingPointNew Y "<<stopping_point_new.y<<"\n";

    EXPECT_TRUE(stopping_point_new ==stopping_point);
    EXPECT_TRUE(velocity == safeVelocity);


 // Caz 2
   std::cout<<"=====================================================\n\n";
   std::cout<<"Caz 5.2\n";

   Vector2f currentP1(20.0f,5.0f);
   Vector2f velocity1(5.0f,0.0f);
   Vector2f safeVelocity1(velocity1);

   stopping_point = polygon.getStoppingPoint(Kp,acc,currentP1,velocity1);

   polygon.adjust_velocity(Kp,currentP1,acc,velocity1,dt);


   stopping_point_new = polygon.getStoppingPoint(Kp,acc,currentP1,velocity1);
   std::cout<<"\n\nValoarea StoppingPointNew X "<<stopping_point_new.x<<"\n";
   std::cout<<"Valoarea StoppingPointNew Y "<<stopping_point_new.y<<"\n";

   EXPECT_TRUE(stopping_point_new ==stopping_point);
   EXPECT_TRUE(velocity1 == safeVelocity1);


// Caz 3
  std::cout<<"=====================================================\n\n";
  std::cout<<"Caz 5.3\n";

  Vector2f currentP2(2.0f,1.0f);
  Vector2f velocity2(5.0f,0.0f);
  Vector2f safeVelocity2(velocity2);

  stopping_point = polygon.getStoppingPoint(Kp,acc,currentP2,velocity2);

  polygon.adjust_velocity(Kp,currentP2,acc,velocity2,dt);


  stopping_point_new = polygon.getStoppingPoint(Kp,acc,currentP2,velocity2);
  std::cout<<"\n\nValoarea StoppingPointNew X "<<stopping_point_new.x<<"\n";
  std::cout<<"Valoarea StoppingPointNew Y "<<stopping_point_new.y<<"\n";

  EXPECT_TRUE(stopping_point_new ==stopping_point);
  EXPECT_TRUE(velocity2 == safeVelocity2);


// Caz 4
  std::cout<<"=====================================================\n\n";
  std::cout<<"Caz 5.4\n";

  Vector2f currentP3(2.0f,6.0f);
  Vector2f velocity3(-1.0f,2.0f);
  Vector2f safeVelocity3(velocity3);

  stopping_point = polygon.getStoppingPoint(Kp,acc,currentP3,velocity3);

  polygon.adjust_velocity(Kp,currentP3,acc,velocity3,dt);


  stopping_point_new = polygon.getStoppingPoint(Kp,acc,currentP3,velocity3);
  std::cout<<"\n\nValoarea StoppingPointNew X "<<stopping_point_new.x<<"\n";
  std::cout<<"Valoarea StoppingPointNew Y "<<stopping_point_new.y<<"\n";

  EXPECT_TRUE(stopping_point_new ==stopping_point);
  EXPECT_TRUE(velocity3 == safeVelocity3);

}

TEST(PolygonConvex, Test6)
{
    std::cout<<"=====================================================\n\n";
    std::cout<<"TestStop4\n";

    std::vector<Vector2f> points;
    Vector2f A(5.0f,4.0f);
    Vector2f B(5.0f,8.0f);
    Vector2f C(9.0f,12.0f);
    Vector2f D(12.0f,9.0f);
    Vector2f E(15.0f,9.0f);
    Vector2f F(12.0f,6.0f);
    Vector2f G(9.0f,6.0f);
    Vector2f H(10.0f,4.0f);

    points.push_back(A);
    points.push_back(B);
    points.push_back(C);
    points.push_back(D);
    points.push_back(E);
    points.push_back(F);
    points.push_back(G);
    points.push_back(H);

    PolygonConvex polygon(points);
    polygon.setBehaviourStop();

    std::cout<<"\nBehaiviourl este "<<(int)polygon.get_behavior()<<"\n";
    float Kp=1.0f;
    float acc=5.0f;
    float dt = 1.0f;

 /// Caz 1
    std::cout<<"=====================================================\n\n";
    std::cout<<"Caz 6.1\n";

    Vector2f currentP(10.0f,3.0f);
    Vector2f velocity(1.0f,1.0f);
    Vector2f safeVelocity(velocity);

    polygon.adjust_velocity(Kp,currentP,acc,velocity,dt);

    EXPECT_TRUE(velocity.is_zero());


 // Caz 2
    std::cout<<"=====================================================\n\n";
    std::cout<<"Caz 6.2\n";
    Vector2f currentP1(9.0f,8.0f);

    Vector2f velocity1(0.0f,-7.0f);
    Vector2f safeVelocity1(velocity1);

    polygon.adjust_velocity(Kp,currentP1,acc,velocity1,dt);

    EXPECT_TRUE(velocity1.is_zero());
}

AP_GTEST_MAIN()

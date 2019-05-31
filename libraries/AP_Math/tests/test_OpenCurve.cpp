/*
 * test_OpenCurve.cpp
 *
 *  Created on: May 27, 2019
 *      Author: beatrice
 */

#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_Math/OpenCurve.h"

// this line is necessary for linking, according to
// http://ardupilot.org/dev/docs/learning-ardupilot-the-example-sketches.html
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

TEST(ComplexTest, Test0)
{
   std::cout<<"=====================================================\n\n";
   std::cout<<"OpenCurve\n";

   std::cout<<"=====================================================\n\n";
   std::cout<<"Test 0 \n";
   std::vector<Vector2f> points;
   Vector2f A(6.0f,8.0f);
   Vector2f B(6.0f,4.0f);
   Vector2f C(12.0f,4.0f);
   Vector2f D(12.0f,10.0f);

   points.push_back(A);
   points.push_back(B);
   points.push_back(C);
   points.push_back(D);

   Vector2f currentP(9.0f,9.0f);
   Vector2f velocity(2.0f,-7.0f);
   Vector2f safeVelocity(velocity);

   OpenCurve openCurve(points);
   openCurve.setBehaviourStop();

       std::cout<<"\nBehaiviourl este "<<(int)openCurve.get_behavior()<<"\n";
       float Kp=1.0f;
       float acc=5.0f;
       float dt = 1.0f;
       openCurve.adjust_velocity(Kp,currentP,acc,velocity,dt);

       Vector2f stopping_point_new = openCurve.getStoppingPoint(Kp,acc,currentP,velocity);
       std::cout<<"\n\nValoarea StoppingPointNew X "<<stopping_point_new.x<<"\n";
       std::cout<<"Valoarea StoppingPointNew Y "<<stopping_point_new.y<<"\n";
}

TEST(ComplexTest, Test1)
{
   std::cout<<"=====================================================\n\n";
   std::cout<<"Test 1 \n";
   std::vector<Vector2f> points;
   Vector2f A(6.0f,8.0f);
   Vector2f B(6.0f,4.0f);
   Vector2f C(12.0f,4.0f);
   Vector2f D(12.0f,10.0f);

   points.push_back(A);
   points.push_back(B);
   points.push_back(C);
   points.push_back(D);

   Vector2f currentP(9.0f,9.0f);
   Vector2f velocity(2.0f,-2.0f);
   Vector2f safeVelocity(velocity);

   OpenCurve openCurve(points);
   openCurve.setBehaviourStop();

   std::cout<<"\nBehaiviourl este "<<(int)openCurve.get_behavior()<<"\n";
   float Kp=1.0f;
   float acc=5.0f;
   float dt = 1.0f;
   openCurve.adjust_velocity(Kp,currentP,acc,velocity,dt);

   Vector2f stopping_point_new = openCurve.getStoppingPoint(Kp,acc,currentP,velocity);
   std::cout<<"\n\nValoarea StoppingPointNew X "<<stopping_point_new.x<<"\n";
   std::cout<<"Valoarea StoppingPointNew Y "<<stopping_point_new.y<<"\n";

   EXPECT_TRUE(is_equal(stopping_point_new.x,10.0f));
   EXPECT_TRUE(is_equal(stopping_point_new.y,8.0f));
   //EXPECT_TRUE(is_equal(1.0f,0.0f));
}

TEST(ComplexTest, Test2)
{
   std::cout<<"=====================================================\n\n";
   std::cout<<"Test 2 \n";
   std::vector<Vector2f> points;
   Vector2f A(6.0f,8.0f);
   Vector2f B(6.0f,4.0f);
   Vector2f C(12.0f,4.0f);
   Vector2f D(12.0f,10.0f);

   points.push_back(A);
   points.push_back(B);
   points.push_back(C);
   points.push_back(D);

   Vector2f currentP(6.0f,1.0f);
   Vector2f velocity(0.0f,4.0f);
   Vector2f safeVelocity(velocity);

   OpenCurve openCurve(points);
   openCurve.setBehaviourStop();

   std::cout<<"\nBehaiviourl este "<<(int)openCurve.get_behavior()<<"\n";
   float Kp=1.0f;
   float acc=5.0f;
   float dt = 1.0f;
   openCurve.adjust_velocity(Kp,currentP,acc,velocity,dt);

   Vector2f stopping_point_new = openCurve.getStoppingPoint(Kp,acc,currentP,velocity);
   std::cout<<"\n\nValoarea StoppingPointNew X "<<stopping_point_new.x<<"\n";
   std::cout<<"Valoarea StoppingPointNew Y "<<stopping_point_new.y<<"\n";

   EXPECT_TRUE(is_equal(stopping_point_new.x,6.0f));
   EXPECT_TRUE(is_equal(stopping_point_new.y,2.0f));

}

TEST(ComplexTest, Test3)
{
   std::cout<<"=====================================================\n\n";
   std::cout<<"Test 3 \n";
   std::vector<Vector2f> points;
   Vector2f A(6.0f,8.0f);
   Vector2f B(6.0f,4.0f);
   Vector2f C(12.0f,4.0f);
   Vector2f D(12.0f,10.0f);

   points.push_back(A);
   points.push_back(B);
   points.push_back(C);
   points.push_back(D);

   Vector2f currentP(3.0f,5.0f);
   Vector2f velocity(7.0f,8.0f);
   Vector2f safeVelocity(velocity);

   OpenCurve openCurve(points);
   openCurve.setBehaviourStop();

   std::cout<<"\nBehaiviourl este "<<(int)openCurve.get_behavior()<<"\n";
   float Kp=1.0f;
   float acc=5.0f;
   float dt = 1.0f;
   openCurve.adjust_velocity(Kp,currentP,acc,velocity,dt);

   Vector2f stopping_point_new = openCurve.getStoppingPoint(Kp,acc,currentP,velocity);
   std::cout<<"\n\nValoarea StoppingPointNew X "<<stopping_point_new.x<<"\n";
   std::cout<<"Valoarea StoppingPointNew Y "<<stopping_point_new.y<<"\n";

   EXPECT_TRUE(stopping_point_new.x >= 3.9f && stopping_point_new.x <= 4.05f);
   EXPECT_TRUE(stopping_point_new.y > 5.9f && stopping_point_new.y < 6.5f);

}

TEST(ComplexTest, Test4)
{
   std::cout<<"=====================================================\n\n";
   std::cout<<"Test 4 \n";
   std::vector<Vector2f> points;
   Vector2f A(5.0f,8.0f);
   Vector2f B(8.0f,5.0f);
   Vector2f C(6.0f,3.0f);
   Vector2f D(5.0f,4.0f);
   Vector2f E(5.0f,1.0f);

   points.push_back(A);
   points.push_back(B);
   points.push_back(C);
   points.push_back(D);
   points.push_back(E);

   Vector2f currentP(10.0f,2.0f);
   Vector2f velocity(-3.0f,3.0f);
   Vector2f safeVelocity(velocity);

   OpenCurve openCurve(points);
   openCurve.setBehaviourStop();

   std::cout<<"\nBehaiviourl este "<<(int)openCurve.get_behavior()<<"\n";
   float Kp=1.0f;
   float acc=5.0f;
   float dt = 1.0f;
   openCurve.adjust_velocity(Kp,currentP,acc,velocity,dt);

   Vector2f stopping_point_new = openCurve.getStoppingPoint(Kp,acc,currentP,velocity);
   std::cout<<"\n\nValoarea StoppingPointNew X "<<stopping_point_new.x<<"\n";
   std::cout<<"Valoarea StoppingPointNew Y "<<stopping_point_new.y<<"\n";

   EXPECT_TRUE(stopping_point_new.x > 8.7f && stopping_point_new.x <= 9.0f  );
   EXPECT_TRUE(stopping_point_new.y >= 3.0f && stopping_point_new.y <= 3.1f);

}

TEST(ComplexTest, Test5)
{
   std::cout<<"=====================================================\n\n";
   std::cout<<"Test 5 \n";
   std::vector<Vector2f> points;
   Vector2f A(5.0f,8.0f);
   Vector2f B(8.0f,5.0f);
   Vector2f C(6.0f,3.0f);
   Vector2f D(5.0f,4.0f);
   Vector2f E(5.0f,1.0f);

   points.push_back(A);
   points.push_back(B);
   points.push_back(C);
   points.push_back(D);
   points.push_back(E);

   Vector2f currentP(8.0f,10.0f);
   Vector2f velocity(0.0f,-4.0f);
   Vector2f safeVelocity(velocity);

   OpenCurve openCurve(points);
   openCurve.setBehaviourStop();

   std::cout<<"\nBehaiviourl este "<<(int)openCurve.get_behavior()<<"\n";
   float Kp=1.0f;
   float acc=5.0f;
   float dt = 1.0f;
   openCurve.adjust_velocity(Kp,currentP,acc,velocity,dt);

   Vector2f stopping_point_new = openCurve.getStoppingPoint(Kp,acc,currentP,velocity);
   std::cout<<"\n\nValoarea StoppingPointNew X "<<stopping_point_new.x<<"\n";
   std::cout<<"Valoarea StoppingPointNew Y "<<stopping_point_new.y<<"\n";

   EXPECT_TRUE(is_equal(stopping_point_new.x,8.0f));
   EXPECT_TRUE(stopping_point_new.y <= 8.0f && stopping_point_new.y >= 7.8f);

}

AP_GTEST_MAIN()



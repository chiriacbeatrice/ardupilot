/*
 * test_line.cpp
 *
 *  Created on: Apr 6, 2019
 *      Author: beatrice
 */
#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_Math/Line.h"
//#include "AC_Avoidance/AC_Avoid.h"

// this line is necessary for linking, according to
// http://ardupilot.org/dev/docs/learning-ardupilot-the-example-sketches.html
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

//Scenariu: pozitia curenta a dronei este ori in extremetitatile segmentului, ori pe acesta
//Comportament asteptat: drona se opreste


TEST(LineTest, AdjustVelocitySlideTest0)
{

    //caz pozitia curenta este start
   Vector2f start(2.0f,1.0f);
   Vector2f stop(7.0f, 6.0f);
   Vector2f velocity(1.0f,2.0f);
   Vector2f zero;

   Line line(start,stop);

  Vector2f v = line.adjust_velocity_Slide(1.0f,start,5.0f,velocity,1.0f,200.0f);
  EXPECT_TRUE(velocity == zero);
  EXPECT_TRUE(v == zero);
}


TEST(LineTest, AdjustVelocitySlideTest1)
{

    //caz pozitia curenta este stop
   Vector2f start(2.0f,1.0f);
   Vector2f stop(7.0f, 6.0f);
   Vector2f velocity(1.0f,2.0f);
   Vector2f zero;

   Line line(start,stop);

  Vector2f v = line.adjust_velocity_Slide(1.0f,stop,5.0f,velocity,1.0f,200.0f);
  EXPECT_TRUE(velocity == zero);
  EXPECT_TRUE(v == zero);
}


TEST(LineTest, AdjustVelocitySlideTest2)
{

  //caz pozitia curenta este pe segment
   Vector2f start(2.0f,1.0f);
   Vector2f stop(7.0f, 6.0f);
   Vector2f velocity(1.0f,2.0f);
   Vector2f zero;

   Line line(start,stop);

   Vector2f segment(5.0f,4.0f);
   Vector2f v4 = line.adjust_velocity_Slide(1.0f,segment,5.0f,velocity,1.0f,200.0f);
   EXPECT_TRUE(v4 == zero);
}


TEST(LineTest, AdjustVelocityTest0)
{   //caz punct egal cu start

    Vector2f start(2.0f,1.0f);
    Vector2f stop(7.0f, 6.0f);
    Vector2f velocity(1.0f,2.0f);
    Vector2f zero;
    Line line(start,stop);


   Vector2f v = line.adjust_velocity(1.0f,start,5.0f,velocity,1.0f);
   EXPECT_TRUE(velocity == zero);
   EXPECT_TRUE(v == zero);
}


TEST(LineTest, AdjustVelocityTest1)
{

    //caz pozitia curenta este stop
   Vector2f start(2.0f,1.0f);
   Vector2f stop(7.0f, 6.0f);
   Vector2f velocity(1.0f,2.0f);
   Vector2f zero;

   Line line(start,stop);

  Vector2f v = line.adjust_velocity(1.0f,stop,5.0f,velocity,1.0f);
  EXPECT_TRUE(velocity == zero);
  EXPECT_TRUE(v == zero);
}


TEST(LineTest, AdjustVelocityTest2)
{

  //caz pozitia curenta este pe segment
   Vector2f start(2.0f,1.0f);
   Vector2f stop(7.0f, 6.0f);
   Vector2f velocity(1.0f,2.0f);
   Vector2f zero;

   Line line(start,stop);

   Vector2f segment(5.0f,4.0f);
   Vector2f v4 = line.adjust_velocity(1.0f,segment,5.0f,velocity,1.0f);
   EXPECT_TRUE(v4 == zero);
}


//Scenariu: pozitia dronei se afla pe aceeasi liniecu segmentul dar nu pe segment
//Asteptare: viteza nuse modifica
TEST(LineTest, AdjustVelocitySlideTest3)
{
    //caz dupa stop

    Vector2f start(2.0f,1.0f);
    Vector2f stop(7.0f, 6.0f);
    Vector2f velocity(1.0f,2.0f);
    Vector2f safevel(velocity);
    Vector2f zero;
    Line line(start,stop);

    Vector2f position(10.0f,9.0f);

    Vector2f v = line.adjust_velocity_Slide(1.0f,position,5.0f,velocity,1.0f,200.0f);
    EXPECT_TRUE(v != zero);
    EXPECT_TRUE(v == velocity);
    EXPECT_TRUE(velocity == safevel);
}

TEST(LineTest, AdjustVelocityTest3)
{
    //caz dupa stop

    Vector2f start(2.0f,1.0f);
    Vector2f stop(7.0f, 6.0f);
    Vector2f velocity(1.0f,2.0f);
    Vector2f safevel(velocity);
    Vector2f zero;
    Line line(start,stop);

    Vector2f position(10.0f,9.0f);

    Vector2f v = line.adjust_velocity(1.0f,position,5.0f,velocity,1.0f);
    EXPECT_TRUE(v != zero);
    EXPECT_TRUE(v == velocity);
    EXPECT_TRUE(velocity == safevel);
}

TEST(LineTest, AdjustVelocityTest6)
{
    //caz inainte de start

    Vector2f start(4.0f,3.0f);
    Vector2f stop(7.0f, 6.0f);
    Vector2f velocity(1.0f,2.0f);
    Vector2f safevel(velocity);
    Vector2f zero;
    Line line(start,stop);

    Vector2f position(2.0f,1.0f);

    Vector2f v = line.adjust_velocity(1.0f,position,5.0f,velocity,1.0f);
    EXPECT_TRUE(v != zero);
    EXPECT_TRUE(v == velocity);
}

TEST(LineTest, AdjustVelocitySlideTest6)
{
    //caz inainte de start

    Vector2f start(4.0f,3.0f);
    Vector2f stop(7.0f, 6.0f);
    Vector2f velocity(1.0f,2.0f);
    Vector2f safevel(velocity);
    Vector2f zero;
    Line line(start,stop);

    Vector2f position(2.0f,1.0f);

    Vector2f v = line.adjust_velocity_Slide(1.0f,position,5.0f,velocity,1.0f,200.0f);
    EXPECT_TRUE(v != zero);
    EXPECT_TRUE(v == velocity);
}


// Scenariu s-a depasit marginea de siguranta, deci trebuie sa fie zero

TEST(LineTest, AdjustVelocityTest4)
{
    //caz dupa margin

    Vector2f start(2.0f,1.0f);
    Vector2f stop(7.0f, 6.0f);
    Vector2f velocity(1.0f,2.0f);
    Vector2f safevel(velocity);
    Vector2f zero;
    Line line(start,stop);

    Vector2f position(5.0f,3.9f);

    Vector2f v = line.adjust_velocity(1.0f,position,5.0f,velocity,1.0f);
    EXPECT_TRUE(v == zero);
    EXPECT_TRUE(v == velocity);
}

////Scenariu s-a atins marginea de siguranta, deci trebuie sa fie zero
TEST(LineTest, AdjustVelocityTest5)
{
    //caz pe margin

    Vector2f start(1.0f,5.0f);
    Vector2f stop(6.0f, 5.0f);
    Vector2f velocity(1.0f,2.0f);
    Vector2f safevel(velocity);
    Vector2f zero;
    Line line(start,stop);

    Vector2f position(4.0f,3.0f);

    Vector2f v = line.adjust_velocity(1.0f,position,5.0f,velocity,1.0f);
    EXPECT_TRUE(v == zero);
    EXPECT_TRUE(v == velocity);
}

////Scenariu caz foarte aproape de marginea de siguranta
TEST(LineTest, AdjustVelocityTestn_apropeMargineTestIntuitiv)
{
    /**
    //    * Test for adjust_velocity_Stop function
    //    *case when the current position is under the margin, but very close
    //    *expected scenario the velocity will became lower then the initial value
    //    **/

   Vector2f start(1.0f,5.0f);
   Vector2f stop(5.0f, 5.0f);
   Vector2f currentPosition(2.0f,3.0f);
   Vector2f velocity(10.0f,20.0f);
   Vector2f safevelocity(velocity);
   Vector2f zeroVector(0.0f,0.0f);
   Vector2f velocity0(velocity);
   Line line(start,stop);

    line.adjust_velocity_Slide(1.0f,currentPosition,5.0f,velocity,1.0f,100.0f);

       EXPECT_TRUE(safevelocity != velocity);
       EXPECT_TRUE(is_equal(velocity.x,safevelocity.x));
       EXPECT_TRUE(velocity.y < safevelocity.y);

       EXPECT_TRUE(velocity != zeroVector);
}

TEST(LineTest, AdjustVelocityTestn)
{
  //caz concret

   Vector2f start(4.0f,7.0f);
   Vector2f stop(12.0f, 7.0f);
   Vector2f currentPosition(8.0f,4.0f);
   Vector2f velocity(100.0f,200.0f);
   Vector2f safevelocity(velocity);
   Vector2f zeroVector(0.0f,0.0f);

   Line line(start,stop);
   Vector2f velocityVerification(100.0f, -1989800.0f); // in calculul pe hartie atat a dat

    line.adjust_velocity(0.0f,currentPosition,5.0f,velocity,1.0f);

       EXPECT_TRUE(safevelocity != velocity);
       EXPECT_TRUE(is_equal(velocity.x,safevelocity.x));
      // EXPECT_TRUE(velocity.x < safevelocity.x); //-pentru  behaivior stop
       EXPECT_TRUE(velocity.y < safevelocity.y);
       EXPECT_TRUE(velocity.y < -1980000.0f);  // am pus asa ca mai sunt mici erori de ordin mic
       EXPECT_TRUE(velocity != zeroVector);
}



AP_GTEST_MAIN()

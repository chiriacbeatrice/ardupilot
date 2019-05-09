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


//TEST(LineTest, AdjustVelocityTest0)
//{
//  //testing if some lines which are inside the functions work fine
//  // Vector2f intersection and closet point
//
//   Vector2f start(44.434753f,26.046537f);
//   Vector2f stop(44.434753f, 26.056557f);
//   Vector2f currentPosition(44.434753f,26.046537f);
//   Vector2f velocity(1.0f,2.0f);
//   Vector2f intersection;
//   Vector2f limit_direction;
//   Line line(start,stop);
//
//   Vector2f stopping_point = currentPosition + velocity*( 200.0f + 1.0f);
//
//      EXPECT_TRUE(Vector2f::segment_intersection(currentPosition, stopping_point, start, stop, intersection));
//      EXPECT_TRUE(intersection == currentPosition);
//      EXPECT_TRUE(limit_direction == (intersection-currentPosition));
//
//
//          //ADAUGAT ULTIMUL MAI UITTE-TE PE SCENARII:
//      Vector2f v = line.adjust_velocity(1.0f,currentPosition,5.0f,velocity,1.0f);
//      EXPECT_TRUE(v==velocity);
//}
//
//TEST(LineTest, adjust_velocity_Stop)
//{  /**
//     * Test for adjust_velocity_Slide function
//     *case when the current position is on the line
//     *expected scenario the velocity will become zero
//    **/
//
//   Vector2f start(2.0f,1.0f);
//   Vector2f stop(7.0f, 6.0f);
//   Vector2f currentPosition0(5.0f,4.0f);
//   Vector2f velocity(1.0f,2.0f);
//   Vector2f zeroVector(0.0f,0.0f);
//   Vector2f velocity0(velocity);
//   Line line(start,stop);
//
//   EXPECT_TRUE(velocity0 == velocity);
//
//   Vector2f stopping_point0 = currentPosition0 + velocity*( 200.0f + 1.0f);
//   velocity0 = line.adjust_velocity_Stop(1.0f,currentPosition0,5.0f,velocity,stopping_point0,1.0f,200.0f);
//
//       EXPECT_TRUE(velocity0 != velocity);
//       EXPECT_TRUE(velocity0 == zeroVector);
//
//
//
//   /**
//     * Test for adjust_velocity_Slide function
//     *case when the current position is outside the segment(but on the line _star _stop)
//     *expected scenario the velocity will remain with the same value
//    **/
//
//   // the current point is on the line, but not on the segment
//
//   Vector2f currentPosition1(8.0f,7.0f);
//   Vector2f stopping_point1 = currentPosition1 + velocity*( 200.0f + 1.0f);
//   Vector2f velocity1 = line.adjust_velocity_Stop(1.0f,currentPosition1,5.0f,velocity,stopping_point1,1.0f,2.0f);
//
//        EXPECT_TRUE(velocity1 == velocity);
//        EXPECT_FALSE(velocity1 == zeroVector);
//
//}
//
//
//
//TEST(LineTest, adjust_velocity_Stop1)
//{
//    /**
//      * Test for adjust_velocity_Slide function
//      *case when the current position is under the margin, but very close
//      *expected scenario the velocity will became lower then the initial value
//     **/
//    Vector2f start(1.0f,5.0f);
//    Vector2f stop(5.0f, 5.0f);
//    Vector2f currentPosition(2.0f,3.0f);
//    Vector2f velocity(1.0f,2.0f);
//    Vector2f zeroVector(0.0f,0.0f);
//    Vector2f velocity0(velocity);
//    Line line(start,stop);
//    Vector2f intersection;
//
//   Vector2f stopping_point = currentPosition + velocity*( 1.0f + 1.0f);
//       EXPECT_TRUE(Vector2f::segment_intersection(currentPosition, stopping_point, start, stop, intersection));
//       EXPECT_FALSE(intersection == zeroVector);
//
//   velocity0 = line.adjust_velocity_Stop(1.0f,currentPosition,5.0f,velocity,stopping_point,1.0f,1.0f);
//
//       EXPECT_TRUE(velocity0 != velocity);
//       EXPECT_TRUE(velocity0.x < velocity.x);
//       EXPECT_TRUE(velocity0.y < velocity.y);
//       EXPECT_TRUE(velocity0 != zeroVector);
//
//
//   /**
//     * Test for adjust_velocity_Slide function
//     *case when the current position is on the margin
//     *expected scenario: the velocity will became zero
//    **/
//
//    Vector2f currentPosition1(4.0f,5.0f);
//    Vector2f stopping_point1 = currentPosition1 + velocity*( 1.0f + 1.0f);
//    velocity0 = line.adjust_velocity_Stop(1.0f,currentPosition1,5.0f,velocity,stopping_point1,1.0f,1.0f);
//
//     EXPECT_TRUE(velocity0 != velocity);
//     EXPECT_TRUE(velocity0 == zeroVector);
//}
//
//TEST(LineTest, adjust_velocity_Slide)
//{  /**
//    * Test for adjust_velocity_Stop function
//    *case when the current position is on the line
//    *expected scenario the velocity will become zero
//    **/
//
//   Vector2f start(2.0f,1.0f);
//   Vector2f stop(7.0f, 6.0f);
//   Vector2f currentPosition(5.0f,4.0f);
//   Vector2f velocity(1.0f,2.0f);
//   Vector2f zeroVector(0.0f,0.0f);
//   Vector2f velocity0(velocity);
//   Line line(start,stop);
//
//   velocity0 = line.adjust_velocity_Slide(1.0f,currentPosition,5.0f,velocity,1.0f,200.0f);
//
//       EXPECT_TRUE(velocity0 != velocity);
//       EXPECT_TRUE(velocity0 == zeroVector);
//
//
//
//   /**
//     * Test for adjust_velocity_Stop function
//     *case when the current position is outside the line(segment _star _stop)
//     *expected scenario the velocity will remain with the same value
//    **/
//
//      // the current point is on the line, but not on the segment
//
//  Vector2f currentPosition1(8.0f,7.0f);
//
//  Vector2f velocity1 = line.adjust_velocity_Slide(1.0f,currentPosition1,5.0f,velocity,1.0f,200.0f);
//
//       EXPECT_TRUE(velocity1 == velocity);
//       EXPECT_FALSE(velocity1 == zeroVector);
//}
//
//TEST(LineTest, adjust_velocity_Slide1)
//{  /**
//    * Test for adjust_velocity_Stop function
//    *case when the current position is under the margin, but very close
//    *expected scenario the velocity will became lower then the initial value
//    **/
//
//   Vector2f start(1.0f,5.0f);
//   Vector2f stop(5.0f, 5.0f);
//   Vector2f currentPosition(2.0f,3.0f);
//   Vector2f velocity(1.0f,2.0f);
//   Vector2f zeroVector(0.0f,0.0f);
//   Vector2f velocity0(velocity);
//   Line line(start,stop);
//
//   velocity0 = line.adjust_velocity_Slide(1.0f,currentPosition,5.0f,velocity,1.0f,100.0f);
//
//       EXPECT_TRUE(velocity0 != velocity);
//       EXPECT_TRUE(is_equal(velocity0.x,velocity.x));
//       EXPECT_TRUE(velocity0.y < velocity.y);
//       EXPECT_TRUE(velocity0 != zeroVector);
//
//
///**
//   * Test for adjust_velocity_Stop function
//   *case when the current position is on the margin
//   *expected scenario: the velocity will became zero
//  **/
//
//  Vector2f currentPosition1(4.0f,5.0f);
//  velocity0 = line.adjust_velocity_Slide(1.0f,currentPosition1,5.0f,velocity,1.0f,1.0f);
//
//   EXPECT_TRUE(velocity0 != velocity);
//   EXPECT_TRUE(velocity0 == zeroVector);
//
//}




AP_GTEST_MAIN()

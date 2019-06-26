/*
 * test_ObjectAvoid.cpp
 *
 *  Created on: Jun 11, 2019
 *      Author: beatricec
 */
#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_Math/ObjectAvoid.h"

// this line is necessary for linking, according to
// http://ardupilot.org/dev/docs/learning-ardupilot-the-example-sketches.html
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

TEST(ObjectAvoid, Test0)
{
 ObjectAvoid obj;
 Vector2f result=obj.location_to_xy(44.435406,26.053265);
 Vector2f result1 = obj.location_to_xy(44.434973,26.047265);
 std::cout<<"Resultatul este: "<<result.x<<" "<<result.y<<"iar norma este: "<<result.length();
 std::cout<<"\n Originea este: "<<result1.x<<" "<<result1.y<<"\n";
 //EXPECT_TRUE(1.0f < 0.0f);
 EXPECT_TRUE(result.x >43 && result.x <49);
 EXPECT_TRUE(result.y <478.08 && result.y >476);
}

TEST(ObjectAvoid, Test1)
{
 ObjectAvoid obj;
 Vector2f result=obj.location_to_xy(44.437211,26.046736);
 Vector2f result1 = obj.location_to_xy(44.434973,26.047265);
 std::cout<<"Resultatul este: "<<result.x<<" "<<result.y<<"iar morma este: "<<result.length();
 std::cout<<"\n Originea este: "<<result1.x<<" "<<result1.y;
 EXPECT_TRUE(result.x <250 && result.x >249);
 EXPECT_TRUE(result.y <-39 && result.y >-44);

}

TEST(ObjectAvoid, Test2)
{
 ObjectAvoid obj;
 Vector2f result=obj.location_to_xy(44.437908,26.049455);
 Vector2f result1 = obj.location_to_xy(44.434973,26.047265);
 std::cout<<"Resultatul este: "<<result.x<<" "<<result.y<<"iar morma este: "<<result.length();
 std::cout<<"\n Originea este: "<<result1.x<<" "<<result1.y;

 EXPECT_TRUE(result.x <327 && result.x >323);
 EXPECT_TRUE(result.y <178 && result.y >174);

}

TEST(ObjectAvoid, Test3)
{
 ObjectAvoid obj;
 Vector2f result=obj.location_to_xy(44.433681,26.045966);
 Vector2f result1 = obj.location_to_xy(44.434973,26.047265);
 std::cout<<"Resultatul este: "<<result.x<<" "<<result.y<<"iar morma este: "<<result.length();
 std::cout<<"\n Originea este: "<<result1.x<<" "<<result1.y;
 EXPECT_TRUE(result.x <-142 && result.x >-144);
 EXPECT_TRUE(result.y <-103 && result.y >-106);

}

TEST(ObjectAvoid, Test4)
{
 ObjectAvoid obj;
 Vector2f result=obj.location_to_xy(44.434772,26.047177);
 Vector2f result1 = obj.location_to_xy(44.434973,26.047265);
 std::cout<<"Resultatul este: "<<result.x<<" "<<result.y<<"iar norma este: "<<result.length();
 std::cout<<"\n Originea este: "<<result1.x<<" "<<result1.y;
// EXPECT_TRUE(1.0f < 0.0f);
}

TEST(ObjectAvoid, Test5)
{
 ObjectAvoid obj;
 Vector2f result=obj.location_to_xy(44.435009,26.047172);
 Vector2f result1 = obj.location_to_xy(44.434973,26.047265);
 std::cout<<"Resultatul este: "<<result.x<<" "<<result.y<<"iar norma este: "<<result.length();
 std::cout<<"\n Originea este: "<<result1.x<<" "<<result1.y;
}


AP_GTEST_MAIN()





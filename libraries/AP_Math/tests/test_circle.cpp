#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_Math/Circle.h"


// this line is necessary for linking, according to
// http://ardupilot.org/dev/docs/learning-ardupilot-the-example-sketches.html
const AP_HAL::HAL& hal = AP_HAL::get_HAL();


TEST(CircleTest, ajustVelocityTest0)
{
    //in afara
    Vector2f centre(10.0f,8.0f);
    Circle circle(4.0f,centre);
    EXPECT_TRUE(is_equal(circle.methodaTest(0.0f),1));
    Vector2f currentP(15.0f,14.0f);
    Vector2f velocity(10.0f,20.0f);
    Vector2f safevel(velocity);
    Vector2f zero;

    Vector2f v = circle.adjust_velocity(1.0f,currentP,5.0f,velocity,1.0f);
    EXPECT_TRUE(v==velocity);
    EXPECT_TRUE(velocity!=safevel);
    EXPECT_TRUE(velocity.x <= zero.x);
    EXPECT_TRUE(velocity.y <= zero.y);
}

TEST(CircleTest, ajustVelocityTest1)
{
    //in afara
    Vector2f centre(10.0f,8.0f);
    Circle circle(4.0f,centre);
    EXPECT_TRUE(is_equal(circle.methodaTest(0.0f),1));
    Vector2f currentP(15.0f,2.0f);
    Vector2f velocity(10.0f,20.0f);
    Vector2f safevel(velocity);
    Vector2f zero;

    Vector2f v = circle.adjust_velocity(1.0f,currentP,5.0f,velocity,1.0f);
    EXPECT_TRUE(v==velocity);
    EXPECT_TRUE(velocity!=safevel);
    EXPECT_TRUE(velocity.x <= zero.x);
    EXPECT_TRUE(velocity.y <= zero.y);
}

TEST(CircleTest, ajustVelocityTest2)
{
    //in afara
    Vector2f centre(10.0f,8.0f);
    Circle circle(4.0f,centre);
    EXPECT_TRUE(is_equal(circle.methodaTest(0.0f),1));
    Vector2f currentP(4.0f,2.0f);
    Vector2f velocity(10.0f,20.0f);
    Vector2f safevel(velocity);
    Vector2f zero;

    Vector2f v = circle.adjust_velocity(1.0f,currentP,5.0f,velocity,1.0f);
    EXPECT_TRUE(v==velocity);
    EXPECT_TRUE(velocity!=safevel);
    EXPECT_TRUE(velocity.x <= zero.x);
    EXPECT_TRUE(velocity.y <= zero.y);
}

TEST(CircleTest, ajustVelocityTestn)
{
    //in afara
    Vector2f centre(10.0f,8.0f);
    Circle circle(4.0f,centre);
    EXPECT_TRUE(is_equal(circle.methodaTest(0.0f),1));
    Vector2f currentP(13.0f,2000.0f);
    Vector2f velocity(10.0f,20.0f);
    Vector2f safevel(velocity);
    Vector2f zero;

    Vector2f v = circle.adjust_velocity(1.0f,currentP,5.0f,velocity,1.0f);
    EXPECT_TRUE(v==velocity);
    EXPECT_TRUE(velocity!=safevel);
    EXPECT_FALSE(velocity.x <= zero.x);  //????
    EXPECT_TRUE(velocity.y <= zero.y);
}


TEST(CircleTest, ajustVelocityTest3)
{
    //pe cerc
    Vector2f centre(10.0f,8.0f);
    Circle circle(4.0f,centre);
    EXPECT_TRUE(is_equal(circle.methodaTest(0.0f),1));
    Vector2f currentP(14.0f,8.0f);
    Vector2f velocity(10.0f,20.0f);
    Vector2f safevel(velocity);
    Vector2f zero;

    Vector2f v = circle.adjust_velocity(1.0f,currentP,5.0f,velocity,1.0f);
    EXPECT_TRUE(v==velocity);
    EXPECT_TRUE(velocity!=safevel);
    EXPECT_TRUE(velocity.x <= zero.x);
    EXPECT_TRUE(velocity.y <= zero.y);
}

//caz in care este pe marginea
TEST(CircleTest, ajustVelocityTest4)
{
    Vector2f centre(10.0f,8.0f);
    Circle circle(4.0f,centre);
    EXPECT_TRUE(is_equal(circle.methodaTest(0.0f),1));
    Vector2f currentP(16.0f,9.0f);
    Vector2f velocity(10.0f,20.0f);
    Vector2f safevel(velocity);
    Vector2f zero;

    Vector2f v = circle.adjust_velocity(1.0f,currentP,5.0f,velocity,1.0f);
    EXPECT_TRUE(v==velocity);
    EXPECT_TRUE(velocity!=safevel);
    EXPECT_TRUE(velocity.x <= zero.x);
    EXPECT_TRUE(velocity.y <= zero.y);

}

AP_GTEST_MAIN()

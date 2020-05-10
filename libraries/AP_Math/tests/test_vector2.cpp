#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>

TEST(Vector2Test, IsEqual)
{
    Vector2l v_int1(1, 1);
    Vector2l v_int2(1, 0);
    Vector2f v_float1(1.0f, 1.0f);
    Vector2f v_float2(1.0f, 0.0f);

    EXPECT_FALSE(v_int1 == v_int2);
    EXPECT_TRUE(v_int1 == v_int1);
    EXPECT_FALSE(v_float1 == v_float2);
    EXPECT_TRUE(v_float1 == v_float1);
}

TEST(Vector2Test, Angle)
{
    Vector2f v_init1(1,1);
    Vector2f v_init2(4,3);

    float angle = v_init1.angle(v_init2);
    std::cout<<angle<<std::endl;
    EXPECT_FLOAT_EQ(angle, 0.14189726);
}


TEST(Vector2Test, closestPoint)
{
    Vector2f v_fistPoint(3.0,-1.0);
    Vector2f v_secoundPoint(4.0,2.0);
    Vector2f v_point(-4.0,2.0);
    Vector2f expectedResult(3.2, -0.4);

    Vector2f closestPoint =
          Vector2f::closest_point(v_point,v_fistPoint,v_secoundPoint);

    EXPECT_TRUE(closestPoint == expectedResult);

}
AP_GTEST_MAIN()

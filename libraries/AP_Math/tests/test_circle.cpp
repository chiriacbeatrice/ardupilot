/*
 * test_circle.cpp
 *
 *  Created on: Jun 11, 2019
 *      Author: beatricec
 */


#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_Math/Circle.h"


// this line is necessary for linking, according to
// http://ardupilot.org/dev/docs/learning-ardupilot-the-example-sketches.html
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

//cualgoritmul nou
//
TEST(CircleTest, testEnvironment)
{
    Vector2f centre(10.0f,8.0f);
    Circle circle(4.0f,centre);
    //method for verifying if the test enviroment works
    EXPECT_TRUE(is_equal(circle.methodaTest(0.0f),1));
   // EXPECT_FLOAT_EQ(1.0f, 0.0f);
}

TEST(CircleTest, ajustVelocityTest0)
{
    // Scenario:
    // Precondition and action:
    // The direction of the drone is not oriented to the target
    // The circle's radius does not intersect the target
    // Reaction: the velocity remains the same

    std::cout<<"=====================================================\n\n";
    std::cout<<"Test0\n";

    Vector2f centre(10.0f,8.0f); // the center of the target
    Circle circle(4.0f,centre); // create the target

    circle.setBehaviourSlide(); // set the behavior of the drone

    Vector2f currentP(15.0f,14.0f); // the current position of the drone
    Vector2f velocity(5.0f,2.0f); // the current velocity of the drone
    Vector2f safevel(velocity);
    Vector2f zero;

    // function for adjusting the velocity
    circle.adjust_velocity(1.0f,currentP,5.0f,velocity,1.0f);
    Vector2f stopping_point_new = circle.getStoppingPoint(1.0f,5.0f,currentP,velocity);

    std::cout<<"\n\nValoarea StoppingPointNew X"<<stopping_point_new.x<<"\n";
    std::cout<<"\nValoarea StoppingPointNew Y"<<stopping_point_new.y<<"\n";

    //Verify that the velocity is unchanged
    EXPECT_TRUE(velocity==safevel);
}

TEST(CircleTest, ajustVelocityTest0_1)
{

    // Scenario:
    // Precondition and action:
    // The direction of the drone is oriented to the target
    // The circle's radius does not intersect the target
    // Reaction: the velocity remains the same

    std::cout<<"=====================================================\n\n";
    std::cout<<"Test0_1\n";

    //current position of the drone
    Vector2f centre(10.0f,8.0f);
    Circle circle(4.0f,centre);
    circle.setBehaviourSlide();
    EXPECT_TRUE(is_equal(circle.methodaTest(0.0f),1));
    Vector2f currentP(15.0f,14.0f);
    Vector2f velocity(-1.0f,-1.0f);
    Vector2f safevel(velocity);
    Vector2f zero;

    // function for adjusting the velocity
    circle.adjust_velocity(1.0f,currentP,5.0f,velocity,1.0f);

    Vector2f stopping_point_new = circle.getStoppingPoint(1.0f,5.0f,currentP,velocity);

    std::cout<<"\n\nValoarea StoppingPointNew X"<<stopping_point_new.x<<"\n";
    std::cout<<"\nValoarea StoppingPointNew Y"<<stopping_point_new.y<<"\n";

    //Verify that the velocity is unchanged
    EXPECT_TRUE(velocity==safevel);
}


TEST(CircleTest, ajustVelocityTest1)
{
    // Scenario: The current position of the drone is on the edge of the target
    // even if the velocity vector is not oriented the opposite direction of the target
    // Reaction: the velocity become zero and the drone stopping to advance

    std::cout<<"=====================================================\n\n";
    std::cout<<"Test1\n";

    Vector2f centre(10.0f,8.0f);
    Circle circle(4.0f,centre);

    circle.setBehaviourSlide();

    Vector2f currentP(14.0f,8.0f);
    Vector2f velocity(10.0f,20.0f);
    Vector2f safevel(velocity);
    Vector2f zero;
    circle.adjust_velocity(1.0f,currentP,5.0f,velocity,1.0f);

    Vector2f stopping_point_new = circle.getStoppingPoint(1.0f,5.0f,currentP,velocity);

    std::cout<<"\n\nValoarea StoppingPointNew X"<<stopping_point_new.x<<"\n";
    std::cout<<"\nValoarea StoppingPointNew Y"<<stopping_point_new.y<<"\n";

    EXPECT_TRUE(velocity!=safevel);
    EXPECT_TRUE(velocity == zero);
    EXPECT_EQ(velocity,zero);

}

TEST(CircleTest, ajustVelocityTest2)
{
    // Scenario: The current position of the drone is on the edge of the target
    // the velocity vector is oriented in the direction of the target
    // Reaction: the velocity become zero and the drone stopping to advance

    std::cout<<"=====================================================\n\n";
    std::cout<<"Test2\n";
    Vector2f centre(10.0f,8.0f);
    Circle circle(4.0f,centre);
    circle.setBehaviourSlide();
    EXPECT_TRUE(is_equal(circle.methodaTest(0.0f),1));
    Vector2f currentP(15.0f,9.0f);
    Vector2f velocity(10.0f,20.0f);
    Vector2f safevel(velocity);
    Vector2f zero;

    circle.adjust_velocity(1.0f,currentP,5.0f,velocity,1.0f);

    Vector2f stopping_point_new = circle.getStoppingPoint(1.0f,5.0f,currentP,velocity);

    std::cout<<"\n\nValoarea StoppingPointNew X"<<stopping_point_new.x<<"\n";
    std::cout<<"\nValoarea StoppingPointNew Y"<<stopping_point_new.y<<"\n";

    EXPECT_TRUE(velocity!=safevel);
    EXPECT_TRUE(velocity == zero);
}

////caz in care este pe marginea si viteza spre tinta
TEST(CircleTest, ajustVelocityTest3)
{
    std::cout<<"=====================================================\n\n";
    std::cout<<"Test3\n";


    Vector2f centre(10.0f,8.0f);
    Circle circle(4.0f,centre);
    circle.setBehaviourSlide();
    EXPECT_TRUE(is_equal(circle.methodaTest(0.0f),1));
    Vector2f currentP(4.0f,8.0f);
    Vector2f velocity(10.0f,20.0f); //varf in (14,28)
    Vector2f safevel(velocity);
    Vector2f zero;

    circle.adjust_velocity(1.0f,currentP,5.0f,velocity,1.0f);

    EXPECT_TRUE(velocity!=safevel);
    EXPECT_TRUE(velocity == zero);

    Vector2f stopping_point_new = circle.getStoppingPoint(1.0f,5.0f,currentP,velocity);

    std::cout<<"\n\nValoarea StoppingPointNew X"<<stopping_point_new.x<<"\n";
    std::cout<<"\nValoarea StoppingPointNew Y"<<stopping_point_new.y<<"\n";

}


TEST(CircleTest, ajustVelocityTest4)
{
    // Scenario: before the edge of the target, and the orientation is changed to the
    // second intersection point
    // in concordance with calculation the intersection points are: (4.58, 10.58) and
    // (12.58, 2.58)

    std::cout<<"=====================================================\n\n";
    std::cout<<"Test4\n";
    Vector2f centre(10.0f,8.0f);
    Circle circle(4.0f,centre);
    circle.setBehaviourSlide();
    EXPECT_TRUE(is_equal(circle.methodaTest(0.0f),1));
    Vector2f currentP(4.0f,2.0f);
    Vector2f velocity(6.0f,5.0f);  //varf in (10,7)
    Vector2f safevel(velocity);
    Vector2f zero;


    circle.adjust_velocity(1.0f,currentP,5.0f,velocity,1.0f);

    EXPECT_TRUE(velocity!=safevel);
    EXPECT_TRUE(velocity.x > 7.8f);
    EXPECT_TRUE(velocity.x < 7.9f);
    EXPECT_TRUE(velocity.y > 0.0f);
    EXPECT_TRUE(velocity.y < 0.05f);

    Vector2f stopping_point_new = circle.getStoppingPoint(1.0f,5.0f,currentP,velocity);
    std::cout<<"\n\n StopingPointFinal X= "<<stopping_point_new.x;
    std::cout<<"\n StopingPointFinal Y= "<<stopping_point_new.y;

    EXPECT_TRUE(stopping_point_new.x>=12.5f && stopping_point_new.x<=12.61f);
    EXPECT_TRUE(stopping_point_new.y>=2.0f && stopping_point_new.y<=2.1f);


    std::cout<<"\nViteza in test ("<<velocity.x<<", "<<velocity.y<<")\n";

    std::cout<<"Norma inainte de procesare:"<<safevel.length()<<"\n";
    std::cout<<"Norma dupa procesare:" <<velocity.length()<<"\n";

    EXPECT_TRUE(is_equal(velocity.length(),safevel.length()));
}


TEST(CircleTest, ajustVelocityTest4_1)
{
    // Scenario: after the first rotation

    std::cout<<"=====================================================\n\n";
    std::cout<<"Test4_1\n";
    Vector2f centre(10.0f,8.0f);
    Circle circle(4.0f,centre);
    circle.setBehaviourSlide();
    EXPECT_TRUE(is_equal(circle.methodaTest(0.0f),1));
    Vector2f currentP(4.0f,2.0f);
    Vector2f velocity(9.18594f,5.34963f);
    Vector2f safevel(velocity);
    Vector2f zero;


    circle.adjust_velocity(1.0f,currentP,5.0f,velocity,1.0f);
    Vector2f stopping_point_new = circle.getStoppingPoint(1.0f,5.0f,currentP,velocity);
    std::cout<<"\n\n StopingPointFinal X= "<<stopping_point_new.x;
    std::cout<<"\n StopingPointFinal Y= "<<stopping_point_new.y;


    EXPECT_TRUE(velocity!=safevel);

    EXPECT_TRUE(stopping_point_new.x>=17.7f && stopping_point_new.x<=17.81f);
    EXPECT_TRUE(stopping_point_new.y>=2.0f && stopping_point_new.y<=2.01f);



    std::cout<<"\nViteza in test ("<<velocity.x<<", "<<velocity.y<<")\n";

    std::cout<<"Norma inainte de procesare:"<<safevel.length()<<"\n";
    std::cout<<"Norma dupa procesare:" <<velocity.length()<<"\n";

    EXPECT_TRUE(is_equal(velocity.length(),safevel.length()));
}


TEST(CircleTest, ajustVelocityTest5)
{

    // Scenarion: the drone is outside the edge of the target and the stopping_point
    // is in the radius circle(R + edge)
    // the rotation of the vector wiil be to the first intersaction point

    std::cout<<"=====================================================\n\n";
    std::cout<<"Test5\n";
    Vector2f centre(10.0f,8.0f);
    Circle circle(4.0f,centre);
    circle.setBehaviourSlide();
    EXPECT_TRUE(is_equal(circle.methodaTest(0.0f),1));
    Vector2f currentP(4.0f,2.0f);
    Vector2f velocity(6.0f,5.0f);  //varf in (7,8)
    Vector2f safevel(velocity);
    Vector2f zero;

    circle.adjust_velocity(1.0f,currentP,5.0f,velocity,1.0f);
    EXPECT_TRUE(!is_equal(velocity.x,safevel.x));
    EXPECT_TRUE(!is_equal(velocity.y,safevel.y));
    EXPECT_TRUE(velocity.x > 7.8);
    EXPECT_TRUE(velocity.x < 8.0);
    EXPECT_TRUE(velocity.y > 0.0);
    EXPECT_TRUE(velocity.y < 0.01);

    std::cout<<"\nViteza in test ("<<velocity.x<<", "<<velocity.y<<")\n";

    std::cout<<"Norma inainte de procesare:"<<safevel.length()<<"\n";
    std::cout<<"Norma dupa procesare:" <<velocity.length()<<"\n";

    EXPECT_TRUE(is_equal(velocity.length(),safevel.length()));

    Vector2f stopping_point_new = circle.getStoppingPoint(1.0f,5.0f,currentP,velocity);
    std::cout<<"\n\n StopingPointFinal X= "<<stopping_point_new.x;
    std::cout<<"\n StopingPointFinal Y= "<<stopping_point_new.y;

    EXPECT_TRUE(stopping_point_new.x>=12.5f && stopping_point_new.x<=12.61f);
    EXPECT_TRUE(stopping_point_new.y>=2.0f && stopping_point_new.y<=2.1f);

}


TEST(CircleTest, ajustVelocityTest6)
{
    // Scenario: outside the target and the circle is included in the target
    // Reaction: velocity remains unchanged

      std::cout<<"=====================================================\n\n";
      std::cout<<"Test6\n";
      Vector2f centre(10.0f,8.0f);
      Circle circle(4.0f,centre);
      circle.setBehaviourSlide();
      EXPECT_TRUE(is_equal(circle.methodaTest(0.0f),1));
      Vector2f currentP(16.0f,3.0f);
      Vector2f velocity(3.0f,3.0f);   // varful are coordonatele (19,6)
      Vector2f safevel(velocity);
      Vector2f zero;

      Vector2f stopping_point = circle.getStoppingPoint(1.0f,0.5f,currentP,velocity);

      circle.adjust_velocity(1.0f,currentP,0.5f,velocity,1.0f);
      EXPECT_TRUE(velocity == safevel);

      std::cout<<"\nViteza in test ("<<velocity.x<<", "<<velocity.y<<")\n";

      std::cout<<"Norma inainte de procesare:"<<(float)safevel.length()<<"\n";
      std::cout<<"Norma dupa procesare:" <<(float)velocity.length()<<"\n";

      Vector2f stopping_point_new = circle.getStoppingPoint(1.0f,0.5f,currentP,velocity);
      std::cout<<"\n\n StopingPointFinal X= "<<stopping_point_new.x;
      std::cout<<"\n StopingPointFinal Y= "<<stopping_point_new.y;

      EXPECT_TRUE(stopping_point_new == stopping_point);

      EXPECT_TRUE(is_equal((float)velocity.length(),(float)safevel.length()));
}

TEST(CircleTest, ajustVelocityTest7)
{
    //Scenario: the drone is outside the edge it is going to the target but is stil not
    //near it, so the velocity will be unchanged

    std::cout<<"=====================================================\n\n";
      std::cout<<"Test7\n";
      Vector2f centre(13.0f,12.0f);
      Circle circle(4.0f,centre);
      circle.setBehaviourSlide();
      EXPECT_TRUE(is_equal(circle.methodaTest(0.0f),1));
      Vector2f currentP(18.0f,22.0f);
      Vector2f velocity(3.0f,3.0f); //varful in (21,25)

      Vector2f safevel(velocity);
      Vector2f zero;
      circle.adjust_velocity(1.0f,currentP,0.5f,velocity,1.0f);
      EXPECT_TRUE(is_equal(velocity.x,safevel.x));
      EXPECT_TRUE(is_equal(velocity.y,safevel.y));

      Vector2f stopping_point = circle.getStoppingPoint(1.0f,0.5f,currentP,velocity);
      std::cout<<"\nViteza in test ("<<velocity.x<<", "<<velocity.y<<")\n";

      std::cout<<"Norma inainte de procesare:"<<(float)safevel.length()<<"\n";
      std::cout<<"Norma dupa procesare:" <<(float)velocity.length()<<"\n";

      EXPECT_TRUE(is_equal((float)velocity.length(),(float)safevel.length()));

      Vector2f stopping_point_new = circle.getStoppingPoint(1.0f,0.5f,currentP,velocity);
      std::cout<<"\n\n StopingPointFinal X= "<<stopping_point_new.x;
      std::cout<<"\n StopingPointFinal Y= "<<stopping_point_new.y;

      EXPECT_TRUE(stopping_point_new == stopping_point);

}


TEST(CircleTest, ajustVelocityTest8)
{
    //Scenario: is near the target and the orientation will be rotated to the left
      std::cout<<"=====================================================\n\n";
      std::cout<<"Test8\n";
      Vector2f centre(13.0f,12.0f);
      Circle circle(4.0f,centre);
      circle.setBehaviourSlide();
      EXPECT_TRUE(is_equal(circle.methodaTest(0.0f),1));
      Vector2f currentP(18.0f,22.0f);
      Vector2f velocity(-3.0f,-3.0f); //aici trebuie orientarea vitezei spre tinta

      Vector2f safevel(velocity);
      Vector2f zero;
      circle.adjust_velocity(1.0f,currentP,0.5f,velocity,1.0f);
      EXPECT_TRUE(!is_equal(velocity.x,safevel.x));
      EXPECT_TRUE(!is_equal(velocity.y,safevel.y));


      std::cout<<"\nViteza in test ("<<velocity.x<<", "<<velocity.y<<")\n";

      std::cout<<"Norma inainte de procesare:"<<(float)safevel.length()<<"\n";
      std::cout<<"Norma dupa procesare:" <<(float)velocity.length()<<"\n";


      Vector2f stopping_point_new = circle.getStoppingPoint(1.0f,0.5f,currentP,velocity);
      std::cout<<"\n\n StopingPointFinal X= "<<stopping_point_new.x;
      std::cout<<"\n StopingPointFinal Y= "<<stopping_point_new.y;

      EXPECT_TRUE(is_equal((float)velocity.length(),(float)safevel.length()));
      EXPECT_TRUE(stopping_point_new.x>=2.3f && stopping_point_new.x<=2.4f);
      EXPECT_TRUE(stopping_point_new.y>=12.6f && stopping_point_new.y<=12.7f);
}

TEST(CircleTest, adjustVelocityStop0)
{
  std::cout<<"=====================================================\n\n";
  std::cout<<"TestStop0\n";
  Vector2f centre(13.0f,12.0f);
  Circle circle(4.0f,centre);
  circle.setBehaviourStop();
  std::cout<<"\nBehaiviourl este "<<(int)circle.get_behavior()<<"\n";
  Vector2f currentP(6.0f,4.0f);
  Vector2f velocity(4.0f,4.0f);
  float Kp=1.0f;
  float acc=5.0f;
  float dt = 1.0f;

  circle.adjust_velocity(Kp,currentP,acc,velocity,dt);

  Vector2f stopping_point_new = circle.getStoppingPoint(Kp,acc,currentP,velocity);

  std::cout<<"\n\nValoarea StoppingPointNew X "<<stopping_point_new.x<<"\n";
  std::cout<<"Valoarea StoppingPointNew Y "<<stopping_point_new.y<<"\n";

  EXPECT_TRUE(stopping_point_new.x>=9.2f && stopping_point_new.x<=9.3f);
  EXPECT_TRUE(stopping_point_new.y>=7.2f && stopping_point_new.y<=7.3f);

}

TEST(CircleTest, adjustVelocityStop1)
{
  std::cout<<"=====================================================\n\n";
  std::cout<<"TestStop1\n";
  Vector2f centre(13.0f,0.0f);
  Circle circle(4.0f,centre);
  circle.setBehaviourStop();
  std::cout<<"\nBehaiviourl este "<<(int)circle.get_behavior()<<"\n";
  Vector2f currentP(4.0f,0.0f);
  Vector2f velocity(6.0f,0.0f);
  float Kp=1.0f;
  float acc=5.0f;
  float dt = 1.0f;

  circle.adjust_velocity(Kp,currentP,acc,velocity,dt);

  Vector2f stopping_point_new = circle.getStoppingPoint(Kp,acc,currentP,velocity);

  std::cout<<"\n\nValoarea StoppingPointNew X "<<stopping_point_new.x<<"\n";
  std::cout<<"Valoarea StoppingPointNew Y "<<stopping_point_new.y<<"\n";

  EXPECT_TRUE(is_equal(stopping_point_new.x,7.0f));
  EXPECT_TRUE(is_equal(stopping_point_new.y,0.0f));

}

TEST(CircleTest, adjustVelocityStop2)
{
  std::cout<<"=====================================================\n\n";
  std::cout<<"TestStop2\n";
  Vector2f centre(13.0f,0.0f);
  Circle circle(4.0f,centre);
  circle.setBehaviourStop();
  std::cout<<"\nBehaiviourl este "<<(int)circle.get_behavior()<<"\n";
  Vector2f currentP(23.0f,0.0f);
  Vector2f velocity(-7.0f,0.0f);
  float Kp=1.0f;
  float acc=5.0f;
  float dt = 1.0f;

  circle.adjust_velocity(Kp,currentP,acc,velocity,dt);

  Vector2f stopping_point_new = circle.getStoppingPoint(Kp,acc,currentP,velocity);

  std::cout<<"\n\nValoarea StoppingPointNew X "<<stopping_point_new.x<<"\n";
  std::cout<<"Valoarea StoppingPointNew Y "<<stopping_point_new.y<<"\n";

  EXPECT_TRUE(is_equal(stopping_point_new.x,19.0f));
  EXPECT_TRUE(is_equal(stopping_point_new.y,0.0f));

}

TEST(CircleTest, adjustVelocityStop3)
{
  std::cout<<"=====================================================\n\n";
  std::cout<<"TestStop3\n";
  Vector2f centre(13.0f,0.0f);
  Circle circle(4.0f,centre);
  circle.setBehaviourStop();
  std::cout<<"\nBehaiviourl este "<<(int)circle.get_behavior()<<"\n";
  Vector2f currentP(6.0f,0.0f);
  Vector2f velocity(-4.0f,0.0f);
  Vector2f safeVelocity(velocity);
  float Kp=1.0f;
  float acc=5.0f;
  float dt = 1.0f;

  circle.adjust_velocity(Kp,currentP,acc,velocity,dt);

  EXPECT_TRUE(safeVelocity == velocity);
  EXPECT_TRUE(safeVelocity == velocity);

}


AP_GTEST_MAIN()



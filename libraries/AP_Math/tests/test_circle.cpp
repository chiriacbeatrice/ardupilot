#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_Math/Circle.h"


// this line is necessary for linking, according to
// http://ardupilot.org/dev/docs/learning-ardupilot-the-example-sketches.html
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

//cualgoritmul nou
//
TEST(CircleTest, ajustVelocityTest0)
{
    //Scenariu: directia nu e orientata spre tinta si cercul de raza directiei nu intersecteza tinta
    //=> nu se modifica viteza
    std::cout<<"=====================================================\n\n";
    std::cout<<"Test0\n";
    //pozitia curenta in
    Vector2f centre(10.0f,8.0f);
    Circle circle(4.0f,centre);
    circle.setBehaviourSlide();
    EXPECT_TRUE(is_equal(circle.methodaTest(0.0f),1));
    Vector2f currentP(15.0f,14.0f);
    Vector2f velocity(0.5f,0.2f);
    Vector2f safevel(velocity);
    Vector2f zero;

    circle.adjust_velocity(1.0f,currentP,5.0f,velocity,1.0f);

    EXPECT_TRUE(velocity==safevel);
    EXPECT_TRUE(is_equal(0.0f,1.0f));//pus intentionat ca sa vedem  logurile
}

TEST(CircleTest, ajustVelocityTest0_1)
{
    //Scenariu: directia e orientata spre tinta si cercul de raza directiei nu intersecteza tinta
    //=> nu se modifica viteza
    std::cout<<"=====================================================\n\n";
    std::cout<<"Test0_1\n";
    //pozitia curenta in
    Vector2f centre(10.0f,8.0f);
    Circle circle(4.0f,centre);
    circle.setBehaviourSlide();
    EXPECT_TRUE(is_equal(circle.methodaTest(0.0f),1));
    Vector2f currentP(15.0f,14.0f);
    Vector2f velocity(0.5f,0.2f);
    Vector2f safevel(velocity);
    Vector2f zero;

    circle.adjust_velocity(1.0f,currentP,5.0f,velocity,1.0f);

    EXPECT_TRUE(velocity==safevel);
}


TEST(CircleTest, ajustVelocityTest1)
{   // caz in care pozitiacurenta e pe marginea cercului
    //chiar daca viteza are senst opus tintei, ea a fost atinsa => se opreste

    std::cout<<"=====================================================\n\n";
    std::cout<<"Test1\n";

    Vector2f centre(10.0f,8.0f);
    Circle circle(4.0f,centre);
    circle.setBehaviourSlide();
    EXPECT_TRUE(is_equal(circle.methodaTest(0.0f),1));
    Vector2f currentP(14.0f,8.0f);
    Vector2f velocity(10.0f,20.0f);
    Vector2f safevel(velocity);
    Vector2f zero;
    circle.adjust_velocity(1.0f,currentP,5.0f,velocity,1.0f);

    EXPECT_TRUE(velocity!=safevel);
    EXPECT_TRUE(velocity == zero);
    //EXPECT_TRUE(is_equal(0.0f,1.0f));//pus intentionat ca sa vedem  logurile
}



//caz in care este dupa marginea dar nu in tinta
TEST(CircleTest, ajustVelocityTest2)
{
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
   // EXPECT_TRUE(v==velocity);
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
    //EXPECT_TRUE(v==velocity);
    EXPECT_TRUE(velocity!=safevel);
    EXPECT_TRUE(velocity == zero);

   // EXPECT_TRUE(is_equal(0.0f,1.0f));//pus intentionat ca sa vedem  logurile
}


TEST(CircleTest, ajustVelocityTest4)
{ // Scenariu: in afarasi se face rotatia spre al doilea punct de intersectie;
  //Conform calcului online  punctele de intersectie sunt :(4.58,10.58) si (12.58,2.58)

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



    std::cout<<"\nViteza in test ("<<velocity.x<<", "<<velocity.y<<")\n";

    std::cout<<"Norma inainte de procesare:"<<safevel.length()<<"\n";
    std::cout<<"Norma dupa procesare:" <<velocity.length()<<"\n";

    EXPECT_TRUE(is_equal(velocity.length(),safevel.length()));
}


TEST(CircleTest, ajustVelocityTest4_1)
{ // Scenariu: se continua dupa rotatia din testul 4 cu datele rezultate de acolo

    std::cout<<"=====================================================\n\n";
    std::cout<<"Test4_1\n";
    Vector2f centre(10.0f,8.0f);
    Circle circle(4.0f,centre);
    circle.setBehaviourSlide();
    EXPECT_TRUE(is_equal(circle.methodaTest(0.0f),1));
    Vector2f currentP(4.0f,2.0f);
    Vector2f velocity(9.18594f,5.34963f);  //varf in (10,7)
    Vector2f safevel(velocity);
    Vector2f zero;


    circle.adjust_velocity(1.0f,currentP,5.0f,velocity,1.0f);

    EXPECT_TRUE(velocity!=safevel);
//    EXPECT_TRUE(velocity.x > 7.7f);
//    EXPECT_TRUE(velocity.x < 7.8f);
//    EXPECT_TRUE(velocity.y > 0.4f);
//    EXPECT_TRUE(velocity.y < 0.6f);



    std::cout<<"\nViteza in test ("<<velocity.x<<", "<<velocity.y<<")\n";

    std::cout<<"Norma inainte de procesare:"<<safevel.length()<<"\n";
    std::cout<<"Norma dupa procesare:" <<velocity.length()<<"\n";

    EXPECT_TRUE(is_equal(velocity.length(),safevel.length()));
}




//caz in care este in afara, iar stopping_point este in cercul de raza (R+margine)
//Scenariu intra pe primul punct de intersectie
TEST(CircleTest, ajustVelocityTest5)
{
    std::cout<<"=====================================================\n\n";
    std::cout<<"Test5\n";
    Vector2f centre(13.0f,12.0f);
    Circle circle(4.0f,centre);
    circle.setBehaviourSlide();
    EXPECT_TRUE(is_equal(circle.methodaTest(0.0f),1));
    Vector2f currentP(4.0f,2.0f);
    Vector2f velocity(6.0f,.0f);  //varf in (7,8)
    Vector2f safevel(velocity);
    Vector2f zero;
    circle.adjust_velocity(1.0f,currentP,5.0f,velocity,1.0f);
    EXPECT_TRUE(!is_equal(velocity.x,safevel.x));
    EXPECT_TRUE(!is_equal(velocity.y,safevel.y));
    EXPECT_TRUE(velocity.x > 1.94);
    EXPECT_TRUE(velocity.x < 1.95);
    EXPECT_TRUE(velocity.y > 3.77);
    EXPECT_TRUE(velocity.y < 3.78);

    std::cout<<"\nViteza in test ("<<velocity.x<<", "<<velocity.y<<")\n";

    std::cout<<"Norma inainte de procesare:"<<safevel.length()<<"\n";
    std::cout<<"Norma dupa procesare:" <<velocity.length()<<"\n";

    EXPECT_TRUE(is_equal(velocity.length(),safevel.length()));

   // EXPECT_TRUE(is_equal(0.0f,1.0f));//pus intentionat ca sa vedem  logurile
}


TEST(CircleTest, ajustVelocityTest6)
{
    //in afara si cercul este cuprins,dar viteza are orientarea pe langa tinta => nu se roteste, ramanene schimbata
    std::cout<<"=====================================================\n\n";
      std::cout<<"Test6\n";
      Vector2f centre(13.0f,12.0f);
      Circle circle(4.0f,centre);
      circle.setBehaviourSlide();
      EXPECT_TRUE(is_equal(circle.methodaTest(0.0f),1));
      Vector2f currentP(16.0f,3.0f);
      Vector2f velocity(3.0f,3.0f);   // varful are coordonatele (19,6)
      Vector2f safevel(velocity);
      Vector2f zero;
      circle.adjust_velocity(1.0f,currentP,0.5f,velocity,1.0f);
      EXPECT_TRUE(velocity == safevel);
//      EXPECT_TRUE(velocity.x > 6.7);
//      EXPECT_TRUE(velocity.x < 7.0);
//      EXPECT_TRUE(velocity.y > 4.0);
//      EXPECT_TRUE(velocity.y < 5.0);

      std::cout<<"\nViteza in test ("<<velocity.x<<", "<<velocity.y<<")\n";

      std::cout<<"Norma inainte de procesare:"<<(float)safevel.length()<<"\n";
      std::cout<<"Norma dupa procesare:" <<(float)velocity.length()<<"\n";

      EXPECT_TRUE(is_equal((float)velocity.length(),(float)safevel.length()));
}

TEST(CircleTest, ajustVelocityTest7)
{
    //in afara si cercul este cuprins, dar orientarea este opusa tintei  => nu se modifica viteza
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


      std::cout<<"\nViteza in test ("<<velocity.x<<", "<<velocity.y<<")\n";

      std::cout<<"Norma inainte de procesare:"<<(float)safevel.length()<<"\n";
      std::cout<<"Norma dupa procesare:" <<(float)velocity.length()<<"\n";

      EXPECT_TRUE(is_equal((float)velocity.length(),(float)safevel.length()));
}


TEST(CircleTest, ajustVelocityTest8)
{
    //in afara,cercul este cuprins, iar orientarea este spre  tinta => se roteste pe prima tangenta
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

      EXPECT_TRUE(is_equal((float)velocity.length(),(float)safevel.length()));
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

  EXPECT_TRUE(is_equal(1.0f,0.0f));
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

  EXPECT_TRUE(stopping_point_new.x<=7.0f);
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
  Vector2f velocity(-5.0f,0.0f);
  float Kp=1.0f;
  float acc=5.0f;
  float dt = 1.0f;

  circle.adjust_velocity(Kp,currentP,acc,velocity,dt);

  Vector2f stopping_point_new = circle.getStoppingPoint(Kp,acc,currentP,velocity);

  std::cout<<"\n\nValoarea StoppingPointNew X "<<stopping_point_new.x<<"\n";
  std::cout<<"Valoarea StoppingPointNew Y "<<stopping_point_new.y<<"\n";

  EXPECT_TRUE(stopping_point_new.x>=19.0f);
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
  Vector2f currentP(23.0f,0.0f);
  Vector2f velocity(4.0f,0.0f);
  Vector2f safeVelocity(velocity);
  float Kp=1.0f;
  float acc=5.0f;
  float dt = 1.0f;

  circle.adjust_velocity(Kp,currentP,acc,velocity,dt);

  EXPECT_TRUE(safeVelocity == velocity);
  EXPECT_TRUE(safeVelocity == velocity);

}


AP_GTEST_MAIN()

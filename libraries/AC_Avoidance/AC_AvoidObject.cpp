/*
 * AC_AvoidObject.cpp
 *
 *  Created on: Mar 5, 2019
 *      Author: beatrice
 */
#include "AC_AvoidObject.h"

#if APM_BUILD_TYPE(APM_BUILD_APMrover2)
# define AP_AVOID_BEHAVE_DEFAULT AC_Avoid::BehaviourType::BEHAVIOR_STOP
#else
# define AP_AVOID_BEHAVE_DEFAULT AC_Avoid::BehaviourType::BEHAVIOR_SLIDE
#endif



  void AC_AvoidObject::adjust_velocity(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt)
{
      DataFlash_Class::instance()->Log_Write("TEST", "TimeUS,Alt,BettyAC_AvoidObjInput",
                                                 "ms", // units: meters per second
                                                 "F", // mult: 1e-6, 1e-2
                                                 "Q", // format: uint64_t, float
                                                 desired_vel_cms);      // exit immediately if disabled
      AC_Avoid::adjust_velocity(kP,accel_cmss,desired_vel_cms,dt);

      //Aici se va aduga un nou parametru si se face o implementare de tipul celor din circle si polygon, dar
      //pentru o bara imaginara
      //chiar daca dupa mine deja face asta....

      DataFlash_Class::instance()->Log_Write("TEST", "TimeUS,Alt,BettyAC_AvoidObjOutput",
                                                   "ms", // units: meters per second
                                                   "F", // mult: 1e-6, 1e-2
                                                   "Q", // format: uint64_t, float
                                                   desired_vel_cms);      // exit immediately if disabled

}

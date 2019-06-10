//
// Created by beatrice(betty) on 15.01.2019.
//

#include "Copter.h"
#include "math.h"
#include "../libraries/AP_Math/ObjectAvoid.h"

#define MAX_VELOCITY             0.5f
/*
 * Init and run calls for loiter flight mode
 */

// loiter_init - initialise loiter controller





bool Copter::ModeModeTest::init(bool ignore_checks)
{
    /////////////////////////////cod adaugat de betty pentru loguri//////////////////////////////
    //Easy Way to add a log//////
    DataFlash_Class::instance()->Log_Write("TEST", "TimeUS,Alt,TESTBetty",
                                           "sm", // units: seconds, meters
                                           "FB", // mult: 1e-6, 1e-2
                                           "Qf", // format: uint64_t, float
                                           AP_HAL::micros64(),
                                           (double)alt_in_cm);  /// campurile astea le=am pus in header caci altfel nu stie de ele

                                           //Inca nu am setat alt_in_cm nicaieri. Ar trebui eventual un setter pentru el

    //Hard Way to add a log//////
    copter.Log_Write_Test();
    //////////////////////////////

    if (copter.position_ok() || ignore_checks) {
        if (!copter.failsafe.radio) {
            float target_roll, target_pitch;
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // convert pilot input to lean angles
            get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max());

            // process pilot's roll and pitch input
            loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch, G_Dt);
        } else {
            // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
            loiter_nav->clear_pilot_desired_acceleration();
        }
        loiter_nav->init_target();

        // initialise position and desired velocity
        if (!pos_control->is_active_z())
        {
            pos_control->set_alt_target_to_current_alt();
            pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
        }

        return true;
    } else {
        return false;
    }
}

#if PRECISION_LANDING == ENABLED
bool Copter::ModeModeTest::do_precision_loiter()
{
    if (!_precision_loiter_enabled) {
        return false;
    }
    if (ap.land_complete_maybe) {
        return false;        // don't move on the ground
    }
    // if the pilot *really* wants to move the vehicle, let them....
    if (loiter_nav->get_pilot_desired_acceleration().length() > 50.0f) {
        return false;
    }
    if (!copter.precland.target_acquired()) {
        return false; // we don't have a good vector
    }
    return true;
}

void Copter::ModeModeTest::precision_loiter_xy()
{
    loiter_nav->clear_pilot_desired_acceleration();
    Vector2f target_pos, target_vel_rel;
    if (!copter.precland.get_target_position_cm(target_pos))  //coment from AC_PrecLand  // returns target position relative to the EKF origin
    {
        target_pos.x = inertial_nav.get_position().x;
        target_pos.y = inertial_nav.get_position().y;
    }
    if (!copter.precland.get_target_velocity_relative_cms(target_vel_rel))  //coment from AC_PrecLand  // returns target velocity relative to vehicle
    {
        target_vel_rel.x = -inertial_nav.get_velocity().x;
        target_vel_rel.y = -inertial_nav.get_velocity().y;
    }
    pos_control->set_xy_target(target_pos.x, target_pos.y);
    pos_control->override_vehicle_velocity_xy(-target_vel_rel);
}
#endif

// loiter_run - runs the loiter controller
// should be called at 100hz or more
void Copter::ModeModeTest::run()
{
    LoiterModeState loiter_state;

    float target_roll, target_pitch;
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;
    float takeoff_climb_rate = 0.0f;
    //added by betty
    Vector2f currentPosition;
    //Location pos; // de decomentat la adaugarea codului
    // added by betty
    // initialize vertical speed and acceleration
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

    // process pilot inputs unless we are in radio failsafe
    if (!copter.failsafe.radio) {
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();  // update_simple_mode - rotates pilot input if we are in simple mode

        // convert pilot input to lean angles
        // get_pilot_desired_angle_rates - transform pilot's roll pitch and yaw input into a desired lean angle rates
        // returns desired angle rates in centi-degrees-per-second
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max());

        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch, G_Dt);

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

 /*//////////////////////code added by betty 02.03.2019//////////////////////////////////////////////////////////

        if  ((inertial_nav.get_latitude()>=(44.434753*powf(10,6)))
          && (inertial_nav.get_longitude()<=(26.046537*powf(10,6))))
        {
          position_ok = inertial_nav.get_position(); // remains set to the value of the last correct position which was recorded
        }

        DataFlash_Class::instance()->Log_Write("PosUAV", "Pos.x,Pos.y,TESTBetty",
                                                   "mm", // units: meters
                                                   "BB", // mult:  1e-2
                                                   "ff", // format: float
                                                    (double)position_ok.x,
                                                    (double)position_ok.y);
        //I verified if the UAV is orients:lat S and long W
        //In this part of code if the UAV current lat and long are not in the correct parameters then it will return to an accepted area
        if(inertial_nav.get_latitude()<(44.434753*powf(10,6)))
        {
            pos_control->set_xy_target(position_ok.x,inertial_nav.get_position().y);
        }
        if(inertial_nav.get_longitude()>(26.046537*powf(10,6)))
        {
            pos_control->set_xy_target(inertial_nav.get_position().x,position_ok.y);
        }
        ////code added by betty 02.03.2019//////////////////////////////////////////////////////////////
*/
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }

    // relax loiter target if we might be landed
    if (ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // Loiter State Machine Determination
    if (!motors->armed() || !motors->get_interlock()) {
        loiter_state = Loiter_MotorStopped;
    } else if (takeoff.running() || takeoff.triggered(target_climb_rate)) {
        loiter_state = Loiter_Takeoff;
    } else if (!ap.auto_armed || ap.land_complete) {
        loiter_state = Loiter_Landed;
    } else {
        loiter_state = Loiter_Flying;
    }

    // Loiter State Machine
    switch (loiter_state) {

        case Loiter_MotorStopped:

            motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
#if FRAME_CONFIG == HELI_FRAME
        // force descent rate and call position controller
        pos_control->set_alt_target_from_climb_rate(-abs(g.land_speed), G_Dt, false);  //apelata continuu spre a mentine pozitia
        if (ap.land_complete_maybe) //we may have landed (less strict version of land_complete)
        {
            pos_control->relax_alt_hold_controllers(0.0f);  //set all desired and targets to measured
        }
#else
            loiter_nav->init_target();
            attitude_control->reset_rate_controller_I_terms(); // reset I terms(rate_roll_pid, rate-pitch_pid,rate_yaw_pid)
            attitude_control->set_yaw_target_to_current_heading(); //// Sets yaw target to vehicle heading
            pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
#endif
            loiter_nav->update();  //calculate dt = time since last_xy_update
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);//Command an euler roll and pitch angle and an euler yaw rate with angular velocity feedforward and smoothing
            pos_control->update_z_controller();  // update_z_controller - fly to altitude in cm above home AC_PosControl.cpp
            break;

        case Loiter_Takeoff:
            // set motors to full range
            motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

            // initiate take-off
            if (!takeoff.running()) {
                takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
                // indicate we are taking off
                set_land_complete(false);
                // clear i term when we're taking off
                set_throttle_takeoff();
            }

            // get takeoff adjusted pilot and takeoff climb rates
            takeoff.get_climb_rates(target_climb_rate, takeoff_climb_rate);

            // get avoidance adjusted climb rate
            target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

            // run loiter controller
            loiter_nav->update();

            // call attitude controller
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);

            // update altitude target and call position controller
            pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
            pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
            pos_control->update_z_controller();
            break;

        case Loiter_Landed:
            // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
            if (target_climb_rate < 0.0f) {
                motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
            } else {
                motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
            }
            loiter_nav->init_target();
            attitude_control->reset_rate_controller_I_terms();
            attitude_control->set_yaw_target_to_current_heading();
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0);
            pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
            pos_control->update_z_controller();

            break;

        case Loiter_Flying:

            // set motors to full range
            motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

#if PRECISION_LANDING == ENABLED
            if (do_precision_loiter()) {
                precision_loiter_xy();
            }
#endif
          
            // run loiter controller
            loiter_nav->prepUpdate();


            //code added by betty

//           Vector3f curr_vel_des = pos_control->get_desired_velocity();
//           /////////////////////////////cod adaugat de betty pentru loguri//////////////////////////////
//              //Easy Way to add a log//////
//           DataFlash_Class::instance()->Log_Write("TEST_Viteza_inainte", "TimeUS,Alt,TEST_Viteza_Betty_pe_xyz_inainte",
//                                                  "cm/s", // units: seconds, meters
//                                                  "cm/s"
//                                                  "cm/s"
//                                                  "FB", // mult: 1e-6, 1e-2
//                                                  "fff", // format: uint64_t, float
//                                                  (double)pos_control->get_desired_velocity().x,
//                                                  (double)pos_control->get_desired_velocity().y,
//                                                  (double)pos_control->get_desired_velocity().z);  /// campurile astea le=am pus in header caci altfel nu stie de ele
//
//                                                     //Inca nu am setat alt_in_cm nicaieri. Ar trebui eventual un setter pentru el
//
//              //Hard Way to add a log//////
//           Vector2f desired_vel_cms(floorf(curr_vel_des.x)/floorf(100),floorf(curr_vel_des.y)/floorf(100));
//
//           //Vector2f desired_vel_cms(curr_vel_des.x,curr_vel_des.y);
//           float Kp=pos_control->get_pos_xy_p().kP();//mai este si  float Kp = ahrs._kp;
//
//           //ahrs.get_relative_position_NE_origin(currentPosition);  // returneaza pozitie relativa la origine
//           inertial_nav.get_location(pos);
//           currentPosition = objectAvoid.location_to_xy(pos.lat,pos.lng);
//
//           float acc = pos_control->get_max_accel_xy();// sau :float acc = loiter_nav->get_pilot_desired_acceleration().length();
//
 //          objectAvoid.adjust_velocity(Kp,currentPosition,
//                                       acc,desired_vel_cms, G_Dt);
//
//           curr_vel_des.x = desired_vel_cms.x*100;
//           curr_vel_des.x = desired_vel_cms.x*100;
//           pos_control->set_desired_velocity(curr_vel_des); // <-asta va trebui comentat
//
//           DataFlash_Class::instance()->Log_Write("TEST_Viteza_dupa", "TimeUS,Alt,TEST_Viteza_Betty_pe_xyz_dupa",
//                                                  "cm/s", // units: seconds, meters
//                                                  "cm/s"
//                                                  "cm/s"
//                                                  "FB", // mult: 1e-6, 1e-2
//                                                  "fff", // format: uint64_t, float
//                                                  (double)pos_control->get_desired_velocity().x,
//                                                  (double)pos_control->get_desired_velocity().y,
//                                                  (double)pos_control->get_desired_velocity().z);
           //code added by betty
           //secound test

            DataFlash_Class::instance()->Log_Write("TEST_Viteza_inainte", "TimeUS,Alt,TEST_Viteza_Betty_pe_xyz_inainte",
                                                   "cm/s", // units: seconds, meters
                                                   "cm/s"
                                                   "cm/s"
                                                   "FB", // mult: 1e-6, 1e-2
                                                   "fff", // format: uint64_t, float
                                                 (double)pos_control->get_desired_velocity().x,
                                                 (double)pos_control->get_desired_velocity().y,
                                                 (double)pos_control->get_desired_velocity().z);

            Vector2f desired_vel_cms(pos_control->get_desired_velocity().x, pos_control->get_desired_velocity().y);

            if(desired_vel_cms.length() > MAX_VELOCITY)
            {
               desired_vel_cms = (desired_vel_cms/desired_vel_cms.length())*MAX_VELOCITY;
               pos_control->set_desired_velocity_xy(desired_vel_cms.x,desired_vel_cms.y);
            }

            pos_control->update_xy_controller();

            DataFlash_Class::instance()->Log_Write("TEST_Viteza_dupa", "TimeUS,Alt,TEST_Viteza_Betty_pe_xyz_dupa",
                                                   "cm/s", // units: seconds, meters
                                                   "cm/s"
                                                   "cm/s"
                                                   "FB", // mult: 1e-6, 1e-2
                                                   "fff", // format: uint64_t, float
                                                 (double)pos_control->get_desired_velocity().x,
                                                 (double)pos_control->get_desired_velocity().y,
                                                 (double)pos_control->get_desired_velocity().z);
            // call attitude controller
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);

            // adjust climb rate using rangefinder
            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);

            // get avoidance adjusted climb rate
            target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

            // update altitude target and call position controller
            pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
            pos_control->update_z_controller();

///////////////////////////////code added by betty 02.03.2019//////////////////////////////////////////////////////////

//            if  ((inertial_nav.get_latitude()>=(44.434753*powf(10,6)))
//              && (inertial_nav.get_longitude()<=(26.046537*powf(10,6))))
//            {
//              position_ok = inertial_nav.get_position(); // remains set to the value of the last correct position which was recorded
//            }
//
//            DataFlash_Class::instance()->Log_Write("PosUAV", "Pos.x,Pos.y,TESTBetty",
//                                                       "mm", // units: meters
//                                                       "BB", // mult:  1e-2
//                                                       "ff", // format: float
//                                                        (double)position_ok.x,
//                                                        (double)position_ok.y);
//            //I verified if the UAV is orients:lat S and long W
//            //In this part of code if the UAV current lat and long are not in the correct parameters then it will return to an accepted area
//            if(inertial_nav.get_latitude()<(44.434753*powf(10,6)))
//            {
//                pos_control->set_xy_target(position_ok.x,inertial_nav.get_position().y);
//            }
//            if(inertial_nav.get_longitude()>(26.046537*powf(10,6)))
//            {
//                pos_control->set_xy_target(inertial_nav.get_position().x,position_ok.y);
//            }
//////////////////////////code added by betty 02.03.2019//////////////////////////////////////////////////////////////




            break;
    }
}

uint32_t Copter::ModeModeTest::wp_distance() const
{
    return loiter_nav->get_distance_to_target();
}

int32_t Copter::ModeModeTest::wp_bearing() const // distanta pana la tinta dorita
{
    return loiter_nav->get_bearing_to_target();
}


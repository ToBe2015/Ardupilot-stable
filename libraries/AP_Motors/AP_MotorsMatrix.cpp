// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_MotorsMatrix.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 */
#include <AP_HAL/AP_HAL.h>
#include "AP_MotorsMatrix.h"

extern const AP_HAL::HAL& hal;

// Init
void AP_MotorsMatrix::Init()
{
    // setup the motors
    setup_motors();

    // enable fast channels or instant pwm
    set_update_rate(_speed_hz);
}

// set update rate to motors - a value in hertz
void AP_MotorsMatrix::set_update_rate( uint16_t speed_hz )
{
    uint8_t i;

    // record requested speed
    _speed_hz = speed_hz;

    // check each enabled motor
    uint32_t mask = 0;
    for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
        if( motor_enabled[i] ) {
		mask |= 1U << i;
        }
    }
    hal.rcout->set_freq( mask, _speed_hz );
}

// set frame orientation (normally + or X)
void AP_MotorsMatrix::set_frame_orientation( uint8_t new_orientation )
{
    // return if nothing has changed
    if( new_orientation == _flags.frame_orientation ) {
        return;
    }

    // call parent
    AP_Motors::set_frame_orientation( new_orientation );

    // setup the motors
    setup_motors();

    // enable fast channels or instant pwm
    set_update_rate(_speed_hz);
}

// enable - starts allowing signals to be sent to motors
void AP_MotorsMatrix::enable()
{
    int8_t i;

    // enable output channels
    for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
        if( motor_enabled[i] ) {
            hal.rcout->enable_ch(i);
        }
    }
}

// output_min - sends minimum values out to the motors
void AP_MotorsMatrix::output_min()
{
    // not used any more
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint16_t AP_MotorsMatrix::get_motor_mask()
{
    uint16_t mask = 0;
    for (uint8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            mask |= 1U << i;
        }
    }
    return mask;
}

void AP_MotorsMatrix::output_armed_not_stabilizing()
{
//    todo: this must all be rewritten
    uint8_t i;
    float throttle_thrust;                        // total throttle thrust value, 0.0 - 1.0
    int16_t motor_out[AP_MOTORS_MAX_NUM_MOTORS];    // final outputs sent to the motors

    // initialize limits flags
    limit.roll_pitch = true;
    limit.yaw = true;
    limit.throttle_lower = false;
    limit.throttle_upper = false;

    throttle_thrust = calc_throttle_thrust();

    // Ensure throttle is within bounds of 0.0 to _throttle_thrust_max
    if (throttle_thrust <= 0.0f) {
        throttle_thrust = 0.0f;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= _throttle_thrust_max) {
        throttle_thrust = _throttle_thrust_max;
        limit.throttle_upper = true;
    }

    // apply thrust curve and voltage scaling
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = throttle_thrust;
        }
    }
}

// output_armed - sends commands to the motors
// includes new scaling stability patch
// TODO pull code that is common to output_armed_not_stabilizing into helper functions
void AP_MotorsMatrix::output_armed_stabilizing()
{
    int8_t i;
    float   roll_thrust;                // roll thrust value, initially calculated by calc_roll_thrust() but may be modified after, +/- 1.0
    float   pitch_thrust;               // pitch thrust value, initially calculated by calc_roll_thrust() but may be modified after, +/- 1.0
    float   yaw_thrust;                 // yaw thrust value, initially calculated by calc_yaw_thrust() but may be modified after, +/- 1.0
    float   throttle_thrust;            // throttle thrust value, summed onto throttle channel minimum, 0.0 - 1.0
    float   throttle_thrust_best_rpy;   // throttle providing maximum roll, pitch and yaw range without climbing
    float   rpy_scale = 1.0f;           // this is used to scale the roll, pitch and yaw to fit within the motor limits

    float   rpy_low = 0.0f;     // lowest motor value
    float   rpy_high = 0.0f;    // highest motor value
    float   yaw_allowed;        // amount of yaw we can fit in
    float   thr_adj;            // the difference between the pilot's desired throttle and throttle_thrust_best_rpy

    // initialize limits flags
    limit.roll_pitch = false;
    limit.yaw = false;
    limit.throttle_lower = false;
    limit.throttle_upper = false;

    roll_thrust = calc_roll_thrust() * get_compensation_gain();
    pitch_thrust = calc_pitch_thrust() * get_compensation_gain();
    yaw_thrust = calc_yaw_thrust() * get_compensation_gain();
    throttle_thrust = calc_throttle_thrust() * get_compensation_gain();

    // Ensure throttle is within bounds of 0.0 to _throttle_thrust_max
    if (throttle_thrust <= 0.0f) {
        throttle_thrust = 0.0f;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= _throttle_thrust_max) {
        throttle_thrust = _throttle_thrust_max;
        limit.throttle_upper = true;
    }

    // calculate roll and pitch for each motor
    // set rpy_low and rpy_high to the lowest and highest values of the motors
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = roll_thrust * _roll_factor[i] + pitch_thrust * _pitch_factor[i];

            // record lowest roll pitch command
            if (_thrust_rpyt_out[i] < rpy_low) {
                rpy_low = _thrust_rpyt_out[i];
            }
            // record highest roll pich command
            if (_thrust_rpyt_out[i] > rpy_high) {
                rpy_high = _thrust_rpyt_out[i];
            }
        }
    }

    // calculate throttle that gives most possible room for yaw (range 1000 ~ 2000) which is the lower of:
    //      1. 0.5f - (rpy_low+rpy_high)/2.0 - this would give the maximum possible room margin above the highest motor and below the lowest
    //      2. the higher of:
    //            a) the pilot's throttle input
    //            b) the point _throttle_rpy_mix between the pilot's input throttle and hover-throttle
    //      Situation #2 ensure we never increase the throttle above hover throttle unless the pilot has commanded this.
    //      Situation #2b allows us to raise the throttle above what the pilot commanded but not so far that it would actually cause the copter to rise.
    //      We will choose #1 (the best throttle for yaw control) if that means reducing throttle to the motors (i.e. we favour reducing throttle *because* it provides better yaw control)
    //      We will choose #2 (a mix of pilot and hover throttle) only when the throttle is quite low.  We favour reducing throttle instead of better yaw control because the pilot has commanded it

    throttle_thrust_best_rpy = min(0.5f - (rpy_low+rpy_high)/2.0, max(throttle_thrust, throttle_thrust*max(0.0f,1.0f-_throttle_rpy_mix)+_throttle_thrust_hover*_throttle_rpy_mix));

    // calculate amount of yaw we can fit into the throttle range
    // this is always equal to or less than the requested yaw from the pilot or rate controller
    yaw_allowed = min(1.0f - throttle_thrust_best_rpy, throttle_thrust_best_rpy) - (rpy_high-rpy_low)/2;
    yaw_allowed = max(yaw_allowed, _yaw_headroom);

    if (yaw_thrust > 0) {
        // if yawing right
        if (yaw_allowed < yaw_thrust) {
            yaw_thrust = yaw_allowed;
            limit.yaw = true;
        }
    }else{
        // if yawing left
        yaw_allowed = -yaw_allowed;
        if (yaw_allowed > yaw_thrust) {
            yaw_thrust = yaw_allowed;
            limit.yaw = true;
        }
    }

    // add yaw to intermediate numbers for each motor
    rpy_low = 0.0f;
    rpy_high = 0.0f;
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] =    _thrust_rpyt_out[i] +
                            yaw_allowed * _yaw_factor[i];

            // record lowest roll+pitch+yaw command
            if( _thrust_rpyt_out[i] < rpy_low ) {
                rpy_low = _thrust_rpyt_out[i];
            }
            // record highest roll+pitch+yaw command
            if( _thrust_rpyt_out[i] > rpy_high) {
                rpy_high = _thrust_rpyt_out[i];
            }
        }
    }

    // check everything fits
    thr_adj = throttle_thrust - throttle_thrust_best_rpy;

    // calculate upper and lower limits of thr_adj
    float thr_adj_max = max(1.0f-(throttle_thrust_best_rpy+rpy_high),0.0f);

    // if we are increasing the throttle (situation #2 above)..
    if (thr_adj > 0.0f) {
        // increase throttle as close as possible to requested throttle
        // without going over 1.0f
        if (thr_adj > thr_adj_max){
            thr_adj = thr_adj_max;
            // we haven't even been able to apply full throttle command
            limit.throttle_upper = true;
        }
    }else if(thr_adj < 0){
        // decrease throttle as close as possible to requested throttle
        // without going under 0.0f or over 1.0f
        // earlier code ensures we can't break both boundaries
        float thr_adj_min = min(-(throttle_thrust_best_rpy+rpy_low),0.0f);
        if (thr_adj > thr_adj_max) {
            thr_adj = thr_adj_max;
            limit.throttle_upper = true;
        }
        if (thr_adj < thr_adj_min) {
            thr_adj = thr_adj_min;
        }
    }

    // do we need to reduce roll, pitch, yaw command
    // earlier code does not allow both limit's to be passed simultaneously with abs(_yaw_factor)<1
    if ((rpy_low+throttle_thrust_best_rpy)+thr_adj < 0.0f){
        // protect against divide by zero
        if (!is_zero(rpy_low)) {
            rpy_scale = -(thr_adj+throttle_thrust_best_rpy)/rpy_low;
        }
        // we haven't even been able to apply full roll, pitch and minimal yaw without scaling
        limit.roll_pitch = true;
        limit.yaw = true;
    }else if((rpy_high+throttle_thrust_best_rpy)+thr_adj > 1.0f){
        // protect against divide by zero
        if (!is_zero(rpy_high)) {
            rpy_scale = (1.0f-thr_adj-throttle_thrust_best_rpy)/rpy_high;
        }
        // we haven't even been able to apply full roll, pitch and minimal yaw without scaling
        limit.roll_pitch = true;
        limit.yaw = true;
    }

    // add scaled roll, pitch, constrained yaw and throttle for each motor
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = throttle_thrust_best_rpy+thr_adj + rpy_scale*_thrust_rpyt_out[i];
        }
    }
}

// output_disarmed - sends commands to the motors
void AP_MotorsMatrix::output_disarmed()
{
    // Not used any more
}

// output_test - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsMatrix::output_test(uint8_t motor_seq, int16_t pwm)
{
    // exit immediately if not armed
    if (!armed()) {
        return;
    }

    // loop through all the possible orders spinning any motors that match that description
    hal.rcout->cork();
    for (uint8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i] && _test_order[i] == motor_seq) {
            // turn on this motor
            hal.rcout->write(i, pwm);
        }
    }
    hal.rcout->push();
}

// add_motor
void AP_MotorsMatrix::add_motor_raw(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, uint8_t testing_order)
{
    // ensure valid motor number is provided
    if( motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS ) {

        // increment number of motors if this motor is being newly motor_enabled
        if( !motor_enabled[motor_num] ) {
            motor_enabled[motor_num] = true;
        }

        // set roll, pitch, thottle factors and opposite motor (for stability patch)
        _roll_factor[motor_num] = roll_fac;
        _pitch_factor[motor_num] = pitch_fac;
        _yaw_factor[motor_num] = yaw_fac;

        // set order that motor appears in test
        _test_order[motor_num] = testing_order;

        // disable this channel from being used by RC_Channel_aux
        RC_Channel_aux::disable_aux_channel(motor_num);
    }
}

// add_motor using just position and prop direction - assumes that for each motor, roll and pitch factors are equal
void AP_MotorsMatrix::add_motor(int8_t motor_num, float angle_degrees, float yaw_factor, uint8_t testing_order)
{
    add_motor(motor_num, angle_degrees, angle_degrees, yaw_factor, testing_order);
}

// add_motor using position and prop direction. Roll and Pitch factors can differ (for asymmetrical frames)
void AP_MotorsMatrix::add_motor(int8_t motor_num, float roll_factor_in_degrees, float pitch_factor_in_degrees, float yaw_factor, uint8_t testing_order)
{
    add_motor_raw(
        motor_num,
        cosf(radians(roll_factor_in_degrees + 90)),
        cosf(radians(pitch_factor_in_degrees)),
        yaw_factor,
        testing_order);
}

// remove_motor - disabled motor and clears all roll, pitch, yaw factors for this motor
void AP_MotorsMatrix::remove_motor(int8_t motor_num)
{
    // ensure valid motor number is provided
    if( motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS ) {
        // disable the motor, set all factors to zero
        motor_enabled[motor_num] = false;
        _roll_factor[motor_num] = 0;
        _pitch_factor[motor_num] = 0;
        _yaw_factor[motor_num] = 0;
    }
}

// remove_all_motors - removes all motor definitions
void AP_MotorsMatrix::remove_all_motors()
{
    for( int8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
        remove_motor(i);
    }
}

// normalise_motors - Normalizes the roll, pitch and yaw factors so maximum magnitude is 0.5
void AP_MotorsMatrix::normalise_motors()
{
    float roll_fac = 0;
    float pitch_fac = 0;
    float yaw_fac = 0;

    for( int8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
        if(roll_fac < fabsf(_roll_factor[i])) {
            roll_fac = _roll_factor[i];
        }
        if(pitch_fac < fabsf(_pitch_factor[i])) {
            pitch_fac = _pitch_factor[i];
        }
        if(yaw_fac < fabsf(_yaw_factor[i])) {
            yaw_fac = _yaw_factor[i];
        }
    }

    for( int8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
        _roll_factor[i] = 0.5f*_roll_factor[i]/roll_fac;
        _pitch_factor[i] = 0.5f*_pitch_factor[i]/pitch_fac;
        _yaw_factor[i] = 0.5f*_yaw_factor[i]/yaw_fac;
    }
}

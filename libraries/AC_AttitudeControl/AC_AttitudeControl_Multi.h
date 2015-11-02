// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file    AC_AttitudeControl_Multi.h
/// @brief   ArduCopter attitude control library

#ifndef AC_AttitudeControl_Multi_H
#define AC_AttitudeControl_Multi_H

#include "AC_AttitudeControl.h"
#include <AP_Motors/AP_MotorsMulticopter.h>

class AC_AttitudeControl_Multi : public AC_AttitudeControl {
public:
	AC_AttitudeControl_Multi(AP_AHRS &ahrs,
                        const AP_Vehicle::MultiCopter &aparm,
                        AP_MotorsMulticopter& motors,
                        AC_P& pi_angle_roll, AC_P& pi_angle_pitch, AC_P& pi_angle_yaw,
                        AC_PID& pid_rate_roll, AC_PID& pid_rate_pitch, AC_PID& pid_rate_yaw
                        ) :
        AC_AttitudeControl(ahrs, aparm, motors, pi_angle_roll, pi_angle_pitch, pi_angle_yaw, pid_rate_roll, pid_rate_pitch, pid_rate_yaw),
        _motors_multi(motors)
        {}

	// empty destructor to suppress compiler warning
	virtual ~AC_AttitudeControl_Multi() {}

    // calculate total body frame throttle required to produce the given earth frame throttle
    float get_boosted_throttle(float throttle_in);

    // get or set angle boost enable/disable
    bool get_angle_boost_enabled() { return _angle_boost_enable; }
    void set_angle_boost_enabled(bool enabled) { _angle_boost_enable = enabled; }

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:

    AP_MotorsMulticopter& _motors_multi;
    AP_Int8               _angle_boost_enable;
};

#endif // AC_AttitudeControl_Multi_H

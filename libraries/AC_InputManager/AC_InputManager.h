// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AC_InputManager.h
/// @brief	Pilot manual control input library

#ifndef AC_INPUTMANAGER_H
#define AC_INPUTMANAGER_H

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

#define AC_ATTITUDE_400HZ_DT                            0.0025f // delta time in seconds for 400hz update rate

/// @class	AC_InputManager
/// @brief	Class managing the pilot's control inputs
class AC_InputManager{
public:
    AC_InputManager():
    _dt(AC_ATTITUDE_400HZ_DT)
    {
        // setup parameter defaults
        AP_Param::setup_object_defaults(this, var_info);
    }

    // set_dt - sets time delta in seconds for all controllers (i.e. 100hz = 0.01, 400hz = 0.0025)
    void set_dt(float delta_sec) {_dt = delta_sec;}

    static const struct AP_Param::GroupInfo        var_info[];

protected:

    // internal variables
    float               _dt;                    // time delta in seconds

    // parameters
    AP_Float _test;
};

#endif /* AC_INPUTMANAGER_H */

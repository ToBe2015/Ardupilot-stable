// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "AC_InputManager.h"
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AC_InputManager::var_info[] PROGMEM = {

    // @Param: TEST
    AP_GROUPINFO("TEST", 0, AC_InputManager, _test, 0),

    AP_GROUPEND
};

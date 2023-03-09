#pragma once

#if AP_XRCE_ENABLED

#include "Time.h"
#include <AP_HAL/AP_HAL.h>

static void update_topic(builtin_interfaces_msg_Time* msg)
{
    if (msg != nullptr) {
        // TODO to be ROS REP 103 compliant, this should use Unix Epoch time, not boot time
        const uint64_t u64 = AP_HAL::micros64();
        msg->sec = u64 / 1000000ULL;
        msg->nanosec = (u64 % 1000000ULL ) * 1000;
    }
}

#endif // AP_XRCE_ENABLED

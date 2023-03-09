#include "AP_XRCE_ROS2_Builtin_Interfaces_Topics.h"
#include "Time.h"
#include <AP_HAL/AP_HAL.h>

static void update_topic(ROS2_BuiltinInterfacesTimeTopic* topic)
{
    if (topic != nullptr) {

        // TODO to be ROS REP 103 compliant, this should use Unix Epoch time, not boot time
        const uint64_t u64 = AP_HAL::micros64();
        topic->sec = u64 / 1000000ULL;
        topic->nanosec = (u64 % 1000000ULL ) * 1000;
    }
}

// void update_topic(Time* msg)
// {
//     if (msg != nullptr) {
//         // TODO to be ROS REP 103 compliant, this should use Unix Epoch time, not boot time
//         msg->sec = AP_HAL::millis() / 1000;
//     }
// }

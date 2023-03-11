#if AP_XRCE_ENABLED

#include "AP_XRCE_Gather_Data.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_RTC/AP_RTC.h>

void update_topic(builtin_interfaces_msg_Time* msg)
{
    if (msg != nullptr) {

        uint64_t utc_usec;
        if (!AP::rtc().get_utc_usec(utc_usec)) {
            utc_usec = AP_HAL::micros64();
        }
        msg->sec = utc_usec / 1000000ULL;
        msg->nanosec = (utc_usec % 1000000ULL) * 1000UL;
    }
}

#endif // AP_XRCE_ENABLED

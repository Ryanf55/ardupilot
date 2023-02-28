#if AP_XRCE_ENABLED

#include "AP_XRCE_ROS2_Builtin_Interfaces_Topics.h"
#include <AP_HAL/AP_HAL.h>
#include "ucdr/microcdr.h"

constexpr char packageName[] = "builtin_interfaces";

ROS2_BuiltinInterfacesTimeTopic::ROS2_BuiltinInterfacesTimeTopic()
:XRCE_Generic_Topic()
{
}

bool ROS2_BuiltinInterfacesTimeTopic::serialize_topic(ucdrBuffer* writer)
{
    bool success = true;

    //sec
    ucdr_serialize_int32_t(writer, sec);
    //nanosec
    ucdr_serialize_uint32_t(writer, nanosec);
    
    return success & !writer->error;
}

bool ROS2_BuiltinInterfacesTimeTopic::deserialize_topic(ucdrBuffer* reader)
{   
    // TODO
    return !reader->error;
}

uint32_t ROS2_BuiltinInterfacesTimeTopic::size_of_topic(uint32_t size)
{
    const uint32_t previousSize = size;
    size += ucdr_alignment(size, 4) + 4;
    size += ucdr_alignment(size, 4) + 4;

    return size - previousSize;
}

void ROS2_BuiltinInterfacesTimeTopic::update_topic()
{
    // TODO to be ROS REP 103 compliant, this should use Unix Epoch time, not boot time
    sec = AP_HAL::millis() / 1000;
}

#endif // AP_XRCE_ENABLED
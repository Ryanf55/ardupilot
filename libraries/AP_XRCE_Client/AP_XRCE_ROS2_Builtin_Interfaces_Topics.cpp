#if AP_XRCE_ENABLED

#include "AP_XRCE_ROS2_Builtin_Interfaces_Topics.h"

#include "ucdr/microcdr.h"

ROS2_BuiltinInterfacesTimeTopic::ROS2_BuiltinInterfacesTimeTopic()
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

#endif // AP_XRCE_ENABLED
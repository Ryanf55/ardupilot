#if AP_XRCE_ENABLED

#include "AP_XRCE_ROS2_Builtin_Interfaces_Topics.h"
#include "ucdr/microcdr.h"

constexpr char packageName[] = "builtin_interfaces";

ROS2_BuiltinInterfacesTimeTopic::ROS2_BuiltinInterfacesTimeTopic()
:XRCE_Generic_Topic()
{
    topic_name = (char *)"ROS2_Time";
    datatype_name = (char *)"Time";
}

bool ROS2_BuiltinInterfacesTimeTopic::topic_initialize(uint8_t xrcetype)
{
    if(xrcetype == XRCE_TYPE::uROS){
        return (uros_initialize(packageName));
    }

    return true;
}

bool ROS2_BuiltinInterfacesTimeTopic::serialize_topic(ucdrBuffer* writer)
{
    bool success = true;

    //sec
    ucdr_serialize_int32_t(writer, 0);
    //nanosec
    ucdr_serialize_uint32_t(writer, 1);
    
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
    // TODO
}

#endif // AP_XRCE_ENABLED
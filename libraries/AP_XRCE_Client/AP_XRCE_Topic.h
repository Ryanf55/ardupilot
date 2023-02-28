#pragma once

#if AP_XRCE_ENABLED

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <AP_Common/ExpandingString.h>

enum XRCE_TYPE{
    DDS=0,
    uROS=1,
};

enum XRCE_TOPIC{
    AP_ROS2_Time=0,
    // AP_ROS2_BatteryState=1,
};

struct ucdrBuffer;

/*
XRCE_Custom_Topic
Info - Parent class for the Ardupilot's custom XRCE/ROS2 topics. 
*/
class XRCE_Generic_Topic {

public:

    XRCE_Generic_Topic();
    virtual bool serialize_topic(ucdrBuffer *writer) = 0;
    virtual bool deserialize_topic(ucdrBuffer *reader) = 0;
    virtual uint32_t size_of_topic(uint32_t size) = 0;
    virtual void update_topic() = 0;

};

XRCE_Generic_Topic* set_topic_instance(uint16_t topic_key);

#endif // AP_XRCE_ENABLED